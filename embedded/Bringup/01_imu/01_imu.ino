/* ============================================================================
 * 01_imu.ino  —  Bring-up sketch 1 of 2
 *
 * Validates: I2C wiring, MPU6050 identity, gyro bias, axis orientation,
 *            sign conventions, and the complementary filter.
 *
 * Produces:  gyro_bias, THETA_SIGN, GYRO_SIGN, theta_offset, noise RMS.
 *
 * No motors are driven here. That is deliberate: if the angle looks wrong,
 * the only suspects are the IMU and the I2C wiring.
 *
 * Wiring (Nano):  SDA -> A4, SCL -> A5, VCC -> 5V, GND -> GND
 * Serial monitor: 115200 baud, line ending "Newline"
 * ========================================================================= */

#include <Wire.h>

#define MPU_ADDR      0x68
#define REG_WHO_AM_I  0x75
#define REG_PWR_MGMT  0x6B
#define REG_CONFIG    0x1A
#define REG_GYRO_CFG  0x1B
#define REG_ACCEL_CFG 0x1C
#define REG_ACCEL_XO  0x3B

const float ACCEL_LSB = 8192.0f;    // +/-4g
const float GYRO_LSB  = 65.5f;      // +/-500 deg/s

const int   LOOP_HZ = 100;
const float DT      = 1.0f / LOOP_HZ;
const float TAU     = 0.5f;
const float ALPHA   = TAU / (TAU + DT);

// PITCH AXIS on this robot: accel Y, gyro X — the IMU is mounted rotated
// 90 degrees from the usual orientation. Confirmed by the ORIENTATION test.
// Signs below are those FOUND on this robot.
#define THETA_SIGN  (+1.0f)
#define GYRO_SIGN   (+1.0f)

float gyro_bias = 0.0f;
float theta       = 0.0f;
float theta_offset= 0.0f;
char  mode        = 'a';

// ---------------------------------------------------------------- MPU driver
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

uint8_t mpuReadByte(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)true);
  return Wire.read();
}

void mpuReadAll(float &ax, float &ay, float &az,
                float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XO);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

  int16_t rax = (Wire.read() << 8) | Wire.read();
  int16_t ray = (Wire.read() << 8) | Wire.read();
  int16_t raz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                       // temperature
  int16_t rgx = (Wire.read() << 8) | Wire.read();
  int16_t rgy = (Wire.read() << 8) | Wire.read();
  int16_t rgz = (Wire.read() << 8) | Wire.read();

  ax = rax / ACCEL_LSB;  ay = ray / ACCEL_LSB;  az = raz / ACCEL_LSB;
  gx = rgx / GYRO_LSB;   gy = rgy / GYRO_LSB;   gz = rgz / GYRO_LSB;
}

void calibrateGyro() {
  Serial.println(F("Calibrating gyro - hold the robot COMPLETELY still..."));
  const int N = 1000;
  float ax, ay, az, gx, gy, gz, sum = 0;
  gyro_bias = 0;
  for (int i = 0; i < N; i++) {
    mpuReadAll(ax, ay, az, gx, gy, gz);
    sum += gx;
    delay(2);
  }
  gyro_bias = sum / N;
  Serial.print(F("gyro_bias = ")); Serial.print(gyro_bias, 4);
  Serial.println(F(" deg/s   <-- record this"));
}

// ------------------------------------------------------------------- helpers
void printMenu() {
  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F(" a - angle mode   (accel angle vs filtered angle)"));
  Serial.println(F(" o - orientation  (which axis is pitch?)"));
  Serial.println(F(" n - noise test   (2 s RMS - run with motors ON)"));
  Serial.println(F(" c - recalibrate gyro"));
  Serial.println(F(" z - zero the current angle"));
  Serial.println(F(" ? - this menu"));
  Serial.println(F("=================================================="));
}

void orientationTest() {
  Serial.println(F("\nTilt the robot FORWARD and hold. Reading for 5 s..."));
  delay(1500);
  float ax, ay, az, gx, gy, gz;
  float axs=0, ays=0, azs=0;
  for (int i = 0; i < 100; i++) {
    mpuReadAll(ax, ay, az, gx, gy, gz);
    axs += ax; ays += ay; azs += az;
    delay(50);
  }
  axs /= 100; ays /= 100; azs /= 100;
  Serial.print(F("  ax=")); Serial.print(axs, 3);
  Serial.print(F("  ay=")); Serial.print(ays, 3);
  Serial.print(F("  az=")); Serial.println(azs, 3);
  Serial.println(F("The axis that changed MOST from upright is your pitch axis."));
  Serial.println(F("atan2(ay, az) should be POSITIVE when leaning forward."));
  Serial.print(F("  atan2(ay,az) = "));
  Serial.print(atan2f(ays, azs) * 180.0f / PI, 2);
  Serial.println(F(" deg"));
  Serial.println(F("If that is negative, set THETA_SIGN to -1.0f"));
}

void noiseTest() {
  Serial.println(F("\nMeasuring angle noise for 2 s."));
  Serial.println(F("Run this again later with the motors spinning."));
  float ax, ay, az, gx, gy, gz;
  float sum = 0, sumsq = 0;
  const int N = 200;
  for (int i = 0; i < N; i++) {
    mpuReadAll(ax, ay, az, gx, gy, gz);
    float td = GYRO_SIGN * (gx - gyro_bias) * PI / 180.0f;
    float ta = THETA_SIGN * atan2f(ay, az);
    theta = ALPHA * (theta + td * DT) + (1 - ALPHA) * ta;
    float d = theta * 180.0f / PI;
    sum += d; sumsq += d * d;
    delay(10);
  }
  float mean = sum / N;
  float rms  = sqrt(sumsq / N - mean * mean);
  Serial.print(F("  mean = ")); Serial.print(mean, 3); Serial.print(F(" deg"));
  Serial.print(F("   noise RMS = ")); Serial.print(rms, 3); Serial.println(F(" deg"));
  Serial.println(F("  < 0.5 deg with motors running = good."));
  Serial.println(F("  > 1.0 deg = vibration is a problem: check DLPF, mounting, wheel balance."));
}

// ---------------------------------------------------------------------- main
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Wire.begin();
  Wire.setClock(400000L);

  Serial.println(F("\n=== 01_imu : IMU bring-up ==="));

  uint8_t who = mpuReadByte(REG_WHO_AM_I);
  Serial.print(F("WHO_AM_I = 0x")); Serial.println(who, HEX);
  if (who != 0x68 && who != 0x70 && who != 0x98) {
    Serial.println(F("!! Unexpected. Check SDA/SCL, power, and AD0 pin."));
    Serial.println(F("!! 0x00 or 0xFF usually means the bus is not connected."));
  } else {
    Serial.println(F("MPU6050 responding."));
  }

  mpuWrite(REG_PWR_MGMT,  0x00); delay(100);
  mpuWrite(REG_CONFIG,    0x03);        // DLPF 44 Hz
  mpuWrite(REG_GYRO_CFG,  0x08);        // +/-500 deg/s
  mpuWrite(REG_ACCEL_CFG, 0x08);        // +/-4g
  // This sensor reports 0x70 (MPU6500 family). Its ACCEL low-pass filter is a
  // separate register the MPU6050 does not have. Uncomment if the noise test
  // with motors running exceeds ~1 deg RMS:
  // mpuWrite(0x1D, 0x03);              // accel DLPF ~44 Hz
  delay(100);

  calibrateGyro();

  float ax, ay, az, gx, gy, gz;
  mpuReadAll(ax, ay, az, gx, gy, gz);
  theta = THETA_SIGN * atan2f(ay, az);

  printMenu();
  Serial.println(F("\nmode a: acc_deg, filt_deg, gyro_dps"));
}

void loop() {
  static unsigned long next = 0;

  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { /* ignore */ }
    else if (c == 'c') calibrateGyro();
    else if (c == 'o') orientationTest();
    else if (c == 'n') noiseTest();
    else if (c == 'z') { theta_offset = theta;
                         Serial.print(F("\ntheta_offset = "));
                         Serial.print(theta_offset, 4);
                         Serial.println(F(" rad  <-- record this")); }
    else if (c == '?') printMenu();
    else { mode = c;
           Serial.print(F("\nmode ")); Serial.println(mode); }
  }

  if (millis() < next) return;
  next = millis() + 10;                       // 100 Hz

  float ax, ay, az, gx, gy, gz;
  mpuReadAll(ax, ay, az, gx, gy, gz);

  float theta_acc = THETA_SIGN * atan2f(ay, az);
  float theta_dot = GYRO_SIGN * (gx - gyro_bias) * PI / 180.0f;
  theta = ALPHA * (theta + theta_dot * DT) + (1 - ALPHA) * theta_acc;

  static int div = 0;
  if (++div < 5) return;                      // print at 20 Hz
  div = 0;

  if (mode == 'a') {
    Serial.print(theta_acc * 180.0f / PI, 2);            Serial.print(',');
    Serial.print((theta - theta_offset) * 180.0f / PI, 2); Serial.print(',');
    Serial.println(theta_dot * 180.0f / PI, 2);
  }
}
