/* ============================================================================
 * balance.ino — state feedback balance controller
 *
 *   u = +K @ [theta, theta_dot, x, x_dot]
 *
 * Poles placed at [-15,-18,-0.7,-0.9] on the measured plant. The angle pair is
 * set above the +9.9 rad/s open-loop instability; the position pair is kept
 * deliberately slow so station-keeping does not fight the angle loop.
 *
 * Telemetry: t_ms,theta,e,theta_dot,x,x_dot,u,armed,overruns  at 50 Hz, 115200.
 * ========================================================================= */

#include <Wire.h>

// ---------------------------------------------------------------- pins
#define ENC_L_A  2
#define ENC_L_B  4
#define ENC_R_A  3
#define ENC_R_B  5
#define AIN1     7
#define AIN2     8
#define PWM_L    9
#define PWM_R    10
#define BIN1     12
#define BIN2     13
#define STBY     6

// ---------------------------------------------------------------- measured
#define THETA_SIGN     (+1.0f)
#define GYRO_SIGN      (+1.0f)
#define MOTOR_L_SIGN   (-1)
#define MOTOR_R_SIGN   (+1)
#define ENC_L_SIGN     (-1)
#define ENC_R_SIGN     (+1)

const float THETA_OFFSET = 0.092f;   // midpoint of the fwd/rev drift band
const int   DEADBAND_L   = 24;
const int   DEADBAND_R   = 18;
const float COUNTS_PER_REV = 1000.0f;
const float WHEEL_RADIUS   = 0.0325f; // 65 mm dia, measured on the robot
const float U_MAX          = 1.7f;    // N at full duty - measured, not datasheet
const float U_MIN          = 0.10f;   // N - below this, command nothing

const float M_PER_COUNT = (2.0f * PI * WHEEL_RADIUS) / COUNTS_PER_REV;

// ---------------------------------------------------------------- gains
// [theta, theta_dot, x, x_dot]
const float K[4] = { 23.57f, 1.20f, -0.96f, -2.56f };

// ---------------------------------------------------------------- loop
const int   LOOP_HZ = 200;
const float DT      = 1.0f / LOOP_HZ;
const unsigned long LOOP_US = 1000000UL / LOOP_HZ;

const float TAU   = 0.5f;
const float ALPHA = TAU / (TAU + DT);
const float VEL_LPF = 0.30f;

// Gyro low-pass for the CONTROL path only (~4.8 Hz). The robot shakes at about
// 25 Hz; unfiltered, K[1]*theta_dot was 97% of u and the loop fed its own
// vibration. The complementary filter still integrates the raw rate.
const float GYRO_LPF = 0.30f;

const float TILT_CUTOFF = 0.52f;   // rad (30 deg) - disarm past this
const float REARM_TILT  = 0.05f;   // rad (3 deg)
const unsigned long REARM_MS = 400;

// ---------------------------------------------------------------- state
float theta = 0, theta_dot = 0, theta_dot_f = 0, pos = 0, vel = 0, vel_f = 0;
float gyro_bias = 0;
volatile long encL = 0, encR = 0;
long enc_prev = 0;
bool armed = false;
bool prev_armed = false;
unsigned long upright_since = 0;
unsigned long armed_at = 0;
unsigned long fell_at = 0;
unsigned int overruns = 0;

// ---------------------------------------------------------------- MPU
#define MPU_ADDR 0x68

void mpuWrite(uint8_t r, uint8_t v) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(r); Wire.write(v);
  Wire.endTransmission();
}

void mpuRead(float &ay, float &az, float &gx) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
  Wire.read(); Wire.read();                          // accel X
  int16_t ray = (Wire.read() << 8) | Wire.read();
  int16_t raz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                          // temp
  int16_t rgx = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); Wire.read(); Wire.read();
  ay = ray / 8192.0f;  az = raz / 8192.0f;  gx = rgx / 65.5f;
}

// ---------------------------------------------------------------- encoders
void isrL() { encL += (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) ? ENC_L_SIGN : -ENC_L_SIGN; }
void isrR() { encR += (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) ? ENC_R_SIGN : -ENC_R_SIGN; }

long encAvg() {
  long l, r;
  noInterrupts(); l = encL; r = encR; interrupts();   // 32-bit reads not atomic
  return (l + r) / 2;
}

// ---------------------------------------------------------------- motors
void setMotor(int pwmPin, int in1, int in2, float u, int deadband) {
  bool fwd = (u >= 0);
  float mag = fabsf(u);
  // Below U_MIN, command nothing. Without this the deadband map jumps any
  // non-zero u straight to duty=deadband, so near upright the motors slam
  // between +/-deadband every loop and chatter instead of controlling.
  if (mag < U_MIN) { analogWrite(pwmPin, 0); return; }
  int duty = (int)((mag / U_MAX) * 255.0f);
  duty = map(duty, 0, 255, deadband, 255);                 // skip the dead zone
  duty = constrain(duty, 0, 255);
  digitalWrite(in1,  fwd);
  digitalWrite(in2, !fwd);
  analogWrite(pwmPin, duty);
}

void drive(float u) {
  u = constrain(u, -U_MAX, U_MAX);
  setMotor(PWM_L, AIN1, AIN2, MOTOR_L_SIGN * u, DEADBAND_L);
  setMotor(PWM_R, BIN1, BIN2, MOTOR_R_SIGN * u, DEADBAND_R);
}

void motorsOff() { analogWrite(PWM_L, 0); analogWrite(PWM_R, 0); }

// ---------------------------------------------------------------- setup
void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWM_L, OUTPUT); pinMode(PWM_R, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  motorsOff();

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, CHANGE);

  TCCR1B = (TCCR1B & 0b11111000) | 0x01;    // ~31 kHz PWM on D9/D10

  Wire.begin(); Wire.setClock(400000L);
  mpuWrite(0x6B, 0x00); delay(100);
  mpuWrite(0x1A, 0x03);                     // gyro DLPF 44 Hz
  mpuWrite(0x1B, 0x08);                     // +/-500 deg/s
  mpuWrite(0x1C, 0x08);                     // +/-4g
  delay(100);

  Serial.println(F("# hold still - calibrating"));
  float ay, az, gx, s = 0;
  for (int i = 0; i < 1000; i++) { mpuRead(ay, az, gx); s += gx; delay(2); }
  gyro_bias = s / 1000.0f;

  mpuRead(ay, az, gx);
  theta = THETA_SIGN * atan2f(ay, az);

  Serial.print(F("# gyro_bias ")); Serial.println(gyro_bias, 4);
  Serial.println(F("# t_ms,theta,e,theta_dot,x,x_dot,u,armed,overruns"));
}

// ---------------------------------------------------------------- loop
void loop() {

    if (Serial.available()) {
    while (Serial.available()) Serial.read();
    Serial.println(F("# ----- RELEASED -----"));
  }

  static unsigned long next = micros();
  if ((long)(micros() - next) < 0) return;
  next += LOOP_US;
  // If the loop fell a whole period behind, resync rather than free-running:
  // catching up at max speed calls mpuRead far faster than DT assumes and
  // corrupts the integration.
  if ((long)(micros() - next) > (long)LOOP_US) {
    overruns++;
    next = micros() + LOOP_US;
  }

  // --- estimate ---
  float ay, az, gx;
  mpuRead(ay, az, gx);
  theta_dot = GYRO_SIGN * (gx - gyro_bias) * PI / 180.0f;
  theta_dot_f += GYRO_LPF * (theta_dot - theta_dot_f);
  float theta_acc = THETA_SIGN * atan2f(ay, az);
  theta = ALPHA * (theta + theta_dot * DT) + (1 - ALPHA) * theta_acc;

  long c = encAvg();
  float dpos = (c - enc_prev) * M_PER_COUNT;
  enc_prev = c;
  pos = c * M_PER_COUNT;
  vel_f += VEL_LPF * ((dpos / DT) - vel_f);
  vel = vel_f;

  float e = theta - THETA_OFFSET;

  // --- arm / disarm ---
  if (armed && fabsf(e) > TILT_CUTOFF) {
    armed = false; motorsOff(); upright_since = 0;
  } else if (!armed) {
    if (fabsf(e) < REARM_TILT) {
      if (upright_since == 0) upright_since = millis();
      if (millis() - upright_since > REARM_MS) {
        noInterrupts(); encL = 0; encR = 0; interrupts();
        enc_prev = 0; pos = 0; vel_f = 0;
        armed = true;
      }
    } else upright_since = 0;
  }

  // --- control ---
  // Sign found on hardware: with the plant's theta convention, +u drives the
  // wheels into the lean. Verified by the lift-and-tilt test.
  float u = 0;
  if (armed) {
    u = +(K[0]*e + K[1]*theta_dot_f + K[2]*pos + K[3]*vel);
    drive(u);
  } else {
    motorsOff();
  }

  // --- events ---
  // Mark the arm/disarm transitions so the run is obvious in the log, and
  // report how long it stayed up - that number is the actual test result.
  if (armed != prev_armed) {
    if (armed) {
      armed_at = millis();
      Serial.println(F("# ===== ARMED ====="));
    } else {
      fell_at = millis();
      Serial.print(F("# ===== FELL after "));
      Serial.print((fell_at - armed_at) / 1000.0f, 2);
      Serial.print(F(" s   tilt "));
      Serial.print(e * 180.0f / PI, 1);
      Serial.println(F(" deg ====="));
    }
    prev_armed = armed;
  }

  // --- telemetry, 50 Hz ---
  // Only while armed, plus 1 s after the fall. Stops the log filling with
  // thousands of identical lines of the robot lying on the floor.
  if (!armed && (fell_at == 0 || millis() - fell_at > 1000)) return;

  static int div = 0;
  if (++div < 2) return;
  div = 0;
  Serial.print(millis());     Serial.print(',');
  Serial.print(theta, 4);     Serial.print(',');
  Serial.print(e, 4);         Serial.print(',');
  Serial.print(theta_dot_f, 3); Serial.print(',');
  Serial.print(pos, 4);       Serial.print(',');
  Serial.print(vel, 3);       Serial.print(',');
  Serial.print(u, 3);         Serial.print(',');
  Serial.print(armed);        Serial.print(',');
  Serial.println(overruns);
}