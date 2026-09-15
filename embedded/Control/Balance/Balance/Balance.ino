/* balance.ino - two-wheeled self-balancing robot, state feedback
   u = K[0]*e + K[1]*theta_dot_f + K[2]*pos + k3*vel_f

   Normal operation prints only the boot banner and ARM / FELL lines.
   Uncomment testModeBegin() in setup() for a single instrumented run. */

#include <Wire.h>

// ---- BUILD TOGGLES ----------------------------------------------------

#define SCHED_ENABLE 1 // Gain Scheduling

// ---- PINS -------------------------------------------------------------

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

// ---- SIGN CONVENTIONS -------------------------------------------------

#define THETA_SIGN   (+1.0f)
#define GYRO_SIGN    (+1.0f)
#define MOTOR_L_SIGN (-1)
#define MOTOR_R_SIGN (+1)
#define ENC_L_SIGN   (-1)
#define ENC_R_SIGN   (+1)

// ---- MEASURED CONSTANTS -----------------------------------------------

const float THETA_OFFSET   = 0.0365f;
const int   DEADBAND_L     = 19;
const int   DEADBAND_R     = 12;
const float COUNTS_PER_REV = 1000.0f;
const float WHEEL_RADIUS   = 0.0325f;
const float U_MAX          = 11.1f;
const float U_MIN          = 0.10f;

// ---- Initialze States -------------------------------------------------

float theta = 0, theta_dot = 0, theta_dot_f = 0;
float pos = 0, vel_f = 0;
float gyro_bias = 0;
volatile long encL = 0, encR = 0;
long enc_prev = 0;
bool armed = false, prev_armed = false;
unsigned long upright_since = 0, armed_at = 0, fell_at = 0;

// ---- CONTROLLER (State feedback gains) --------------------------------

const float K[4] = { 13.703f, 0.879f, 0.405f, 16.000f }; 

// Low-pass on the rate term only. The raw gyro is noisy enough that K[1]
// amplifies it into audible chatter. Costs phase lag - keep it as high as
// the chatter allows. 1.0 = off.
const float GYRO_LPF = 0.15f;

// K3 reaches K3_FAST once wheel speed hits V_BLEND. Delivered force falls
// with speed (back-EMF), so more velocity gain is needed when moving.
const float K3_FAST = 26.0f;
const float V_BLEND = 0.5f;

// ---- TIMING AND FILTERS -----------------------------------------------
// ALPHA is the complementary filter split: gyro integration dominates short
// term, accelerometer pulls out the long-term drift.
const int   LOOP_HZ = 200;
const float DT      = 1.0f / LOOP_HZ;
const unsigned long LOOP_US = 1000000UL / LOOP_HZ;
const float TAU     = 0.5f;
const float ALPHA   = TAU / (TAU + DT);
const float VEL_LPF = 0.15f;

// ---- ARM / DISARM -----------------------------------------------------

const float TILT_CUTOFF = 0.52f;
const float REARM_TILT  = 0.05f;
const unsigned long REARM_MS = 300;


// ---- IMU --------------------------------------------------------------

#define MPU_ADDR 0x68

void mpuWrite(uint8_t r, uint8_t v) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(r); Wire.write(v);
  Wire.endTransmission();
}

// Reading bytes of the Imu
void mpuRead(float &ay, float &az, float &gx) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
  Wire.read(); Wire.read();
  int16_t ray = (Wire.read() << 8) | Wire.read();
  int16_t raz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t rgx = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); Wire.read(); Wire.read();
  ay = ray / 8192.0f;  az = raz / 8192.0f;  gx = rgx / 65.5f;
}

// ---- ENCODERS ---------------------------------------------------------

void isrL() { encL += (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) ? ENC_L_SIGN : -ENC_L_SIGN; }
void isrR() { encR += (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) ? ENC_R_SIGN : -ENC_R_SIGN; }


long encAvg() {
  long l, r;
  noInterrupts(); l = encL; r = encR; interrupts();
  return (l + r) / 2;
}

// ---- MOTORS -----------------------------------------------------------

// Force -> duty. Below U_MIN send nothing, else remap 0..255 onto
// deadband..255 so the smallest command still breaks static friction.
void setMotor(int pwmPin, int in1, int in2, float u, int deadband) {
  bool fwd = (u >= 0);
  float mag = fabsf(u);
  if (mag < U_MIN) { analogWrite(pwmPin, 0); return; }
  int duty = (int)((mag / U_MAX) * 255.0f);
  duty = map(duty, 0, 255, deadband, 255);
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

// - ESTIMATOR (Turns raw sensors into the four states the control law needs.)-------------

void estimateAngle() { 
  float ay, az, gx;
  mpuRead(ay, az, gx);
  theta_dot    = GYRO_SIGN * (gx - gyro_bias) * PI / 180.0f;
  theta_dot_f += GYRO_LPF * (theta_dot - theta_dot_f);
  float theta_acc = THETA_SIGN * atan2f(ay, az);
  theta = ALPHA * (theta + theta_dot * DT) + (1 - ALPHA) * theta_acc;
}


void estimateCart() {
  const float M_PER_COUNT = (2.0f * PI * WHEEL_RADIUS) / COUNTS_PER_REV;
  long c = encAvg();
  float dpos = (c - enc_prev) * M_PER_COUNT;
  enc_prev = c;
  pos    = c * M_PER_COUNT;
  vel_f += VEL_LPF * ((dpos / DT) - vel_f);
}

// Zero the cart states. Called on arming so every run starts from x = 0.
void resetCart() {
  noInterrupts(); encL = 0; encR = 0; interrupts();
  enc_prev = 0; pos = 0; vel_f = 0;
}

// ---- CONFIG BANNER ----------------------------------------------------

// Printed at boot and again at the head of every dump, so a saved log
// carries the constants that produced it. Bias is passed in because the
// dump must report the bias from the run being dumped, not this boot.
void printConfig(float bias) {
  Serial.print(F("# BUILD ")); Serial.print(F(__DATE__));
  Serial.print(' ');           Serial.println(F(__TIME__));
  Serial.print(F("# gyro_bias ")); Serial.println(bias, 4);
  Serial.print(F("# offset ")); Serial.print(THETA_OFFSET, 4);
  Serial.print(F("  K ")); Serial.print(K[0], 3); Serial.print(' ');
  Serial.print(K[1], 3); Serial.print(' '); Serial.print(K[2], 3);
  Serial.print(' '); Serial.print(K[3], 3);
  Serial.print(F("  GYRO_LPF ")); Serial.println(GYRO_LPF, 2);
  Serial.print(F("# U_MAX ")); Serial.print(U_MAX, 2);
  Serial.print(F("  K3_FAST ")); Serial.print(K3_FAST, 1);
  Serial.print(F("  V_BLEND ")); Serial.print(V_BLEND, 2);
  Serial.print(F("  SCHED ")); Serial.println(SCHED_ENABLE);
}

// ---- TEST MODE --------------------------------------------------------

// Enable/disable: the testModeBegin() call in setup().
// Keep the battery connected when plugging in USB, or the data is lost.
// TEST_SAMPLES x 4 bytes comes out of the same 2 KB as the stack. Raising
// either define means bumping TEST_MAGIC, or a stale buffer of the old
// geometry gets read as valid.
// Trace looks like noise -> raise TEST_HZ, cut TEST_SECONDS to match.

#define TEST_SECONDS 10
#define TEST_HZ      25
#define TEST_SAMPLES (TEST_SECONDS * TEST_HZ)
#define TEST_DIV     (LOOP_HZ / TEST_HZ)
#define TEST_MAGIC   0xB101u


#define SC_ANG  10000.0f   // rad    -> +/-3.27 rad
#define SC_U     1000.0f   // N      -> +/-32.7 N

int16_t  test_buf[TEST_SAMPLES][2] __attribute__((section(".noinit")));
uint16_t test_count __attribute__((section(".noinit")));
uint16_t test_magic __attribute__((section(".noinit")));
float    test_bias  __attribute__((section(".noinit")));

bool test_mode    = false;   
bool test_started = false;   
bool test_done    = false;   

int16_t testPack(float v, float scale) {
  long n = lroundf(v * scale);
  if (n >  32767L) n =  32767L;
  if (n < -32767L) n = -32767L;
  return (int16_t)n;
}


void testModeDump() {
  if (test_magic != TEST_MAGIC) return;
  if (test_count == 0 || test_count > TEST_SAMPLES) { test_magic = 0; return; }

  Serial.print(F("# ---- PREVIOUS RUN: "));
  Serial.print(test_count); Serial.print(F(" samples @ "));
  Serial.print(TEST_HZ);    Serial.println(F(" Hz ----"));
  printConfig(test_bias);
  Serial.println(F("# t_ms,e,u"));

  for (uint16_t i = 0; i < test_count; i++) {
    Serial.print((unsigned long)i * (1000UL / TEST_HZ)); Serial.print(',');
    Serial.print(test_buf[i][0] / SC_ANG, 4);            Serial.print(',');
    Serial.println(test_buf[i][1] / SC_U,  3);
  }
  Serial.println(F("# ---- END ----"));

  test_magic = 0;
}


void testModeBegin() {
  testModeDump();
  test_mode = true; test_started = false; test_done = false;
  test_count = 0; test_magic = TEST_MAGIC;
  test_bias = gyro_bias;
  Serial.print(F("# TEST MODE - "));
  Serial.print(TEST_SECONDS); Serial.print(F(" s @ "));
  Serial.print(TEST_HZ);      Serial.println(F(" Hz, one run"));
}

bool testLockout() { return test_done; }


void testModeUpdate(float e, float u) {
  if (!test_mode || test_done) return;

  if (!armed) {
    if (test_started) test_done = true;   
    return;
  }
  test_started = true;

  if (millis() - armed_at >= (unsigned long)TEST_SECONDS * 1000UL) {
    test_done = true;
    armed = false;
    motorsOff();
    Serial.println(F("# ===== TEST RUN COMPLETE ====="));
    return;
  }

  static uint8_t tick = 0;
  if (++tick < TEST_DIV) return;
  tick = 0;
  if (test_count >= TEST_SAMPLES) return;

  test_buf[test_count][0] = testPack(e, SC_ANG);
  test_buf[test_count][1] = testPack(u, SC_U);
  test_count++;
}

// ---- ARM / DISARM -----------------------------------------------------

void updateArming(float e) {
  
  if (testLockout()) { if (armed) { armed = false; motorsOff(); } return; }

  if (armed && fabsf(e) > TILT_CUTOFF) {
    armed = false; motorsOff(); upright_since = 0;
  } else if (!armed) {
    if (fabsf(e) < REARM_TILT) {
      if (upright_since == 0) upright_since = millis();
      if (millis() - upright_since > REARM_MS) {
        resetCart();
        armed = true;
      }
    } else upright_since = 0;
  }
}

// ---- CONTROLLER -------------------------------------------------------

float controlLaw(float e) {
  float k3 = K[3];
#if SCHED_ENABLE
  k3 += (K3_FAST - K[3]) * fminf(fabsf(vel_f) / V_BLEND, 1.0f);
#endif
  return K[0]*e + K[1]*theta_dot_f + K[2]*pos + k3*vel_f;
}

// ---- REPORTING --------------------------------------------------------

void reportTransitions() {
  if (armed == prev_armed) return;
  if (armed) {
    armed_at = millis();
    Serial.println(F("# ===== ARMED ====="));
  } else {
    fell_at = millis();
    Serial.print(F("# ===== FELL after "));
    Serial.print((fell_at - armed_at) / 1000.0f, 2);
    Serial.println(F(" s ====="));
  }
  prev_armed = armed;
}

// ---- SETUP ------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);   

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWM_L, OUTPUT); pinMode(PWM_R, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  motorsOff();

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, CHANGE);

  TCCR1B = (TCCR1B & 0b11111000) | 0x01;

  Wire.begin(); Wire.setClock(400000L);
  mpuWrite(0x6B, 0x00); delay(100);
  mpuWrite(0x1A, 0x03);
  mpuWrite(0x1B, 0x08);
  mpuWrite(0x1C, 0x08);
  delay(100);

  Serial.println(F("# hold still - calibrating"));
  float ay, az, gx, sum = 0;
  for (int i = 0; i < 1000; i++) { mpuRead(ay, az, gx); sum += gx; delay(2); }
  gyro_bias = sum / 1000.0f;

  mpuRead(ay, az, gx);
  theta = THETA_SIGN * atan2f(ay, az);

  printConfig(gyro_bias);

  // ---- TESTING MODE TOGGLE --------------------------------------------
  
  testModeBegin(); // Uncomment for one instrumented run; comment out for normal operation. <------------------------
}

// ---- CONTROL LOOP -----------------------------------------------------

void loop() {
  
  static unsigned long next = micros();
  if ((long)(micros() - next) < 0) return;
  next += LOOP_US;
  if ((long)(micros() - next) > (long)LOOP_US) next = micros() + LOOP_US;

  estimateAngle();
  estimateCart();

  float e = theta - THETA_OFFSET;   // error about the true balance point
  updateArming(e);

  float u = 0;
  if (armed) { u = controlLaw(e); drive(u); }
  else       { motorsOff(); }

  reportTransitions();
  testModeUpdate(e, u);
}