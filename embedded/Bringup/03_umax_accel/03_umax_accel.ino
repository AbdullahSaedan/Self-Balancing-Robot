/*
 * 03_umax_accel.ino — measure U_MAX from full-duty acceleration
 *
 * Robot sits on front/back skids so it cannot tip. Type 'f' or 'b' in the
 * Serial Monitor (115200 baud): both motors go to full duty from rest, both
 * encoders are logged every 1 ms for 200 ms into RAM, the motors brake, and a
 * CSV is printed. Paste every run into one text file and run fit_umax.py.
 *
 * U_MAX = m_total * a0, where a0 is the initial acceleration from the fit.
 */

// ===================== PINS (check against balance_min.ino) =====================
const uint8_t PIN_PWMA = 9;
const uint8_t PIN_AIN1 = 7;
const uint8_t PIN_AIN2 = 8;
const uint8_t PIN_STBY = 6;
const uint8_t PIN_BIN1 = 12;
const uint8_t PIN_BIN2 = 13;
const uint8_t PIN_PWMB = 10;
const uint8_t PIN_ENC_L_A = 2;   // must be an interrupt pin
const uint8_t PIN_ENC_L_B = 4;
const uint8_t PIN_ENC_R_A = 3;   // must be an interrupt pin
const uint8_t PIN_ENC_R_B = 5;

// ===================== SIGNS (confirmed on hardware) =====================
const int MOTOR_L_SIGN = -1;
const int MOTOR_R_SIGN = +1;
const int ENC_L_SIGN   = -1;
const int ENC_R_SIGN   = +1;

// ===================== TEST SETTINGS =====================
const uint8_t  TEST_DUTY = 255;   // full duty — this is what U_MAX means
const uint16_t N_SAMPLES = 200;   // 200 samples...
const uint16_t SAMPLE_US = 1000;  // ...at 1 kHz = 200 ms window

// ===================== ENCODERS =====================
// Interrupt on both edges of channel A; channel B gives direction.
volatile long encL = 0;
volatile long encR = 0;

void isrL() { if (digitalRead(PIN_ENC_L_A) == digitalRead(PIN_ENC_L_B)) encL++; else encL--; }
void isrR() { if (digitalRead(PIN_ENC_R_A) == digitalRead(PIN_ENC_R_B)) encR++; else encR--; }

// ===================== MOTOR DRIVE =====================
// cmd in -255..255. Direction via IN1/IN2, magnitude via PWM.
void setMotor(uint8_t pwm, uint8_t in1, uint8_t in2, int cmd) {
  if (cmd >= 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else          { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); cmd = -cmd; }
  analogWrite(pwm, constrain(cmd, 0, 255));
}

// Short brake (TB6612: IN1 = IN2 = HIGH) so the robot stops quickly after the run.
void brakeAll() {
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH); analogWrite(PIN_PWMA, 0);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH); analogWrite(PIN_PWMB, 0);
}

// ===================== LOG BUFFER =====================
// 2 x 200 x int16 = 800 bytes of RAM. Logged to RAM, printed after, so serial
// output cannot disturb the sample timing.
int16_t logL[N_SAMPLES];
int16_t logR[N_SAMPLES];

// ===================== BUILD STAMP =====================
// Check this against the wall clock — catches a stale binary (bootloader trap).
void printStamp() {
  Serial.print(F("# build ")); Serial.print(F(__DATE__));
  Serial.print(' ');           Serial.println(F(__TIME__));
}

// ===================== ONE TEST RUN =====================
void runTest(int dir) {
  noInterrupts(); encL = 0; encR = 0; interrupts();

  uint32_t t0 = micros();
  setMotor(PIN_PWMA, PIN_AIN1, PIN_AIN2, dir * MOTOR_L_SIGN * TEST_DUTY);
  setMotor(PIN_PWMB, PIN_BIN1, PIN_BIN2, dir * MOTOR_R_SIGN * TEST_DUTY);

  for (uint16_t i = 0; i < N_SAMPLES; i++) {
    while ((micros() - t0) < (uint32_t)i * SAMPLE_US) { }   // wait for sample slot
    noInterrupts(); long l = encL; long r = encR; interrupts();
    logL[i] = (int16_t)(ENC_L_SIGN * l);
    logR[i] = (int16_t)(ENC_R_SIGN * r);
  }
  brakeAll();

  printStamp();
  Serial.print(F("# dir ")); Serial.println(dir > 0 ? 'f' : 'b');
  Serial.println(F("t_ms,encL,encR"));
  for (uint16_t i = 0; i < N_SAMPLES; i++) {
    Serial.print(i); Serial.print(',');
    Serial.print(logL[i]); Serial.print(',');
    Serial.println(logR[i]);
  }
  Serial.println(F("# end"));
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_ENC_L_A, INPUT_PULLUP); pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP); pinMode(PIN_ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrR, CHANGE);

  brakeAll();
  digitalWrite(PIN_STBY, HIGH);

  printStamp();
  Serial.println(F("# U_MAX test ready. Robot on skids, cable slack. Type f or b."));
}

// ===================== LOOP =====================
void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'f') runTest(+1);
    if (c == 'b') runTest(-1);
  }
}
