/*
 * arduino2_slave.ino
 * ---------------------------------------------------------------------
 * Role: ARDUINO 2 (Slave)
 *   - I2C slave at address I2C_SLAVE_ADDR, talks only to Arduino 1 over
 *     the shared I2C bus (A4=SDA, A5=SCL). No UART / USB link to the Pi.
 *   - Drives the two wheel motors (JGA37 #1 = left, JGA37 #2 = right)
 *     through their BTS7960 drivers, with a simple closed-loop speed
 *     controller fed by the quadrature encoders.
 *
 * Wiring (from ketnoi.xlsx, "Arduino 2 (Slaver)" sheet):
 *   BTS7960 #1 (left)   EN->D5(PWM)  RPWM->D4   LPWM->D6(PWM)
 *   BTS7960 #2 (right)  EN->D11(PWM) RPWM->D12  LPWM->D10(PWM)
 *   JGA37 #1 (left)     Kenh A->D2 (INT0)   Kenh B->D7
 *   JGA37 #2 (right)    Kenh A->D3 (INT1)   Kenh B->D8
 *   Arduino 1           SDA<->A4   SCL<->A5   (shared I2C bus)
 *
 * NOTE on the RPWM/LPWM pins: as wired on this board, RPWM is a plain
 * digital pin (D4 / D12) and only LPWM is PWM-capable (D6 / D10), with a
 * separate PWM "EN" pin (D5 / D11) used to modulate overall speed. The
 * driver below follows exactly that wiring: EN carries the speed PWM,
 * RPWM/LPWM just select direction (HIGH/LOW).
 *
 * I2C protocol (Arduino1 = master):
 *   Write 5 bytes: [0x10, leftMMs_hi, leftMMs_lo, rightMMs_hi, rightMMs_lo]
 *                  -> CMD_SET_VEL, int16 target speed per wheel, mm/s,
 *                     signed, big-endian.
 *   Write 2 bytes: [0x30, enable]   -> CMD_SET_ENABLE (0 = stop & hold)
 *   Read  8 bytes: [leftTicks(int32 BE), rightTicks(int32 BE)]
 *                  -> encoder delta since the previous read; counters are
 *                     reset to 0 as soon as they are read.
 *
 * CALIBRATION: set TICKS_PER_REV to match your JGA37 gearbox/encoder.
 * The value below is only a starting point — measure real ticks for one
 * full wheel revolution and adjust.
 * ---------------------------------------------------------------------
 */

#include <Wire.h>

// =======================================================================
// Configuration — tune these to match the real robot
// =======================================================================
static const uint8_t I2C_SLAVE_ADDR = 0x08;

// Left motor (JGA37 #1 / BTS7960 #1)
static const uint8_t PIN_L_EN   = 5;   // PWM, speed
static const uint8_t PIN_L_RPWM = 4;   // digital, direction
static const uint8_t PIN_L_LPWM = 6;   // PWM, direction (driven HIGH/LOW only)
static const uint8_t PIN_L_ENC_A = 2;  // INT0
static const uint8_t PIN_L_ENC_B = 7;

// Right motor (JGA37 #2 / BTS7960 #2)
static const uint8_t PIN_R_EN   = 11;  // PWM, speed
static const uint8_t PIN_R_RPWM = 12;  // digital, direction
static const uint8_t PIN_R_LPWM = 10;  // PWM, direction (driven HIGH/LOW only)
static const uint8_t PIN_R_ENC_A = 3;  // INT1
static const uint8_t PIN_R_ENC_B = 8;

static const float WHEEL_RADIUS_MM = 30.0f;          // fullrobotassem.urdf
static const float TICKS_PER_REV   = 990.0f;         // CALIBRATE ME
static const float MM_PER_TICK     = (2.0f * PI * WHEEL_RADIUS_MM) / TICKS_PER_REV;

static const float MAX_WHEEL_SPEED_MMS = 400.0f;     // used to scale PI output
static const unsigned long CONTROL_PERIOD_MS = 20;   // speed-loop rate
static const unsigned long CMD_TIMEOUT_MS    = 500;  // stop if master goes quiet

// Speed-loop PI gains (mm/s error -> PWM counts). Start conservative and tune.
static const float KP = 0.8f;
static const float KI = 0.15f;
static const int   PWM_MAX = 255;

// =======================================================================
// State
// =======================================================================
volatile long encTicksLeft  = 0;
volatile long encTicksRight = 0;

long encSnapshotLeftForI2C  = 0;   // "last read" markers, one per consumer
long encSnapshotRightForI2C = 0;

volatile int16_t targetLeftMMs  = 0;
volatile int16_t targetRightMMs = 0;
volatile bool    motorsEnabled  = true;
volatile unsigned long lastI2cCmdMs = 0;

long lastTicksLeftForControl  = 0;
long lastTicksRightForControl = 0;
float integralLeft  = 0.0f;
float integralRight = 0.0f;
unsigned long lastControlMs = 0;

// =======================================================================
// Encoder ISRs (channel-A edge, read channel-B to get direction)
// =======================================================================
void isrLeftEncoder() {
  bool b = digitalRead(PIN_L_ENC_B);
  if (b) encTicksLeft--; else encTicksLeft++;
}

void isrRightEncoder() {
  bool b = digitalRead(PIN_R_ENC_B);
  if (b) encTicksRight--; else encTicksRight++;
}

// =======================================================================
// Motor driver — EN carries speed PWM, RPWM/LPWM select direction only.
// speedMMs: signed target speed in mm/s. pwmOut: 0-255 magnitude.
// =======================================================================
void driveMotor(uint8_t enPin, uint8_t rpwmPin, uint8_t lpwmPin,
                 int pwmOut, bool forward) {
  pwmOut = constrain(pwmOut, 0, PWM_MAX);
  if (!motorsEnabled) {
    analogWrite(enPin, 0);
    digitalWrite(rpwmPin, LOW);
    digitalWrite(lpwmPin, LOW);
    return;
  }
  digitalWrite(rpwmPin, forward ? HIGH : LOW);
  digitalWrite(lpwmPin, forward ? LOW : HIGH);
  analogWrite(enPin, pwmOut);
}

void stopAllMotors() {
  analogWrite(PIN_L_EN, 0);
  analogWrite(PIN_R_EN, 0);
  digitalWrite(PIN_L_RPWM, LOW);
  digitalWrite(PIN_L_LPWM, LOW);
  digitalWrite(PIN_R_RPWM, LOW);
  digitalWrite(PIN_R_LPWM, LOW);
  integralLeft = 0.0f;
  integralRight = 0.0f;
}

// One PI step for a single wheel. Returns signed PWM-equivalent command.
float piStep(float targetMMs, float actualMMs, float &integral, float dt_s) {
  float error = targetMMs - actualMMs;
  integral += error * dt_s;
  integral = constrain(integral, -200.0f, 200.0f);   // anti-windup clamp
  float output = KP * error + KI * integral;
  return output;
}

void updateSpeedControl() {
  unsigned long now = millis();
  unsigned long dt_ms = now - lastControlMs;
  if (dt_ms < CONTROL_PERIOD_MS) return;
  float dt_s = dt_ms / 1000.0f;
  lastControlMs = now;

  noInterrupts();
  long curLeft = encTicksLeft;
  long curRight = encTicksRight;
  interrupts();

  float actualLeftMMs  = ((curLeft  - lastTicksLeftForControl)  * MM_PER_TICK) / dt_s;
  float actualRightMMs = ((curRight - lastTicksRightForControl) * MM_PER_TICK) / dt_s;
  lastTicksLeftForControl  = curLeft;
  lastTicksRightForControl = curRight;

  int16_t tgtLeft, tgtRight;
  bool enabled;
  noInterrupts();
  tgtLeft = targetLeftMMs;
  tgtRight = targetRightMMs;
  enabled = motorsEnabled;
  interrupts();

  if (!enabled) { stopAllMotors(); return; }

  float outLeft  = piStep((float)tgtLeft,  actualLeftMMs,  integralLeft,  dt_s);
  float outRight = piStep((float)tgtRight, actualRightMMs, integralRight, dt_s);

  // Feed-forward term so the loop doesn't start from zero PWM.
  float ffLeft  = (tgtLeft  / MAX_WHEEL_SPEED_MMS) * PWM_MAX;
  float ffRight = (tgtRight / MAX_WHEEL_SPEED_MMS) * PWM_MAX;

  int pwmLeft  = (int)(ffLeft  + outLeft);
  int pwmRight = (int)(ffRight + outRight);

  driveMotor(PIN_L_EN, PIN_L_RPWM, PIN_L_LPWM, abs(pwmLeft),  pwmLeft  >= 0);
  driveMotor(PIN_R_EN, PIN_R_RPWM, PIN_R_LPWM, abs(pwmRight), pwmRight >= 0);
}

// =======================================================================
// I2C callbacks
// =======================================================================
void onI2CReceive(int numBytes) {
  if (numBytes <= 0) return;
  uint8_t cmd = Wire.read();
  lastI2cCmdMs = millis();

  if (cmd == 0x10 && numBytes >= 5) {       // CMD_SET_VEL
    uint8_t b1 = Wire.read();
    uint8_t b2 = Wire.read();
    uint8_t b3 = Wire.read();
    uint8_t b4 = Wire.read();
    targetLeftMMs  = (int16_t)((b1 << 8) | b2);
    targetRightMMs = (int16_t)((b3 << 8) | b4);
  } else if (cmd == 0x30 && numBytes >= 2) { // CMD_SET_ENABLE
    uint8_t en = Wire.read();
    motorsEnabled = (en != 0);
    if (!motorsEnabled) { targetLeftMMs = 0; targetRightMMs = 0; }
  }
  // drain anything unexpected left in the buffer
  while (Wire.available()) Wire.read();
}

void onI2CRequest() {
  long left, right;
  noInterrupts();
  left  = encTicksLeft  - encSnapshotLeftForI2C;
  right = encTicksRight - encSnapshotRightForI2C;
  encSnapshotLeftForI2C  = encTicksLeft;
  encSnapshotRightForI2C = encTicksRight;
  interrupts();

  uint8_t buf[8];
  buf[0] = (uint8_t)((left >> 24) & 0xFF);
  buf[1] = (uint8_t)((left >> 16) & 0xFF);
  buf[2] = (uint8_t)((left >> 8) & 0xFF);
  buf[3] = (uint8_t)(left & 0xFF);
  buf[4] = (uint8_t)((right >> 24) & 0xFF);
  buf[5] = (uint8_t)((right >> 16) & 0xFF);
  buf[6] = (uint8_t)((right >> 8) & 0xFF);
  buf[7] = (uint8_t)(right & 0xFF);
  Wire.write(buf, 8);
}

// =======================================================================
// Arduino entry points
// =======================================================================
void setup() {
  pinMode(PIN_L_EN, OUTPUT);
  pinMode(PIN_L_RPWM, OUTPUT);
  pinMode(PIN_L_LPWM, OUTPUT);
  pinMode(PIN_R_EN, OUTPUT);
  pinMode(PIN_R_RPWM, OUTPUT);
  pinMode(PIN_R_LPWM, OUTPUT);
  stopAllMotors();

  pinMode(PIN_L_ENC_A, INPUT_PULLUP);
  pinMode(PIN_L_ENC_B, INPUT_PULLUP);
  pinMode(PIN_R_ENC_A, INPUT_PULLUP);
  pinMode(PIN_R_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_L_ENC_A), isrLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_R_ENC_A), isrRightEncoder, CHANGE);

  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  lastControlMs = millis();
  lastI2cCmdMs = millis();
}

void loop() {
  if (millis() - lastI2cCmdMs > CMD_TIMEOUT_MS) {
    // Lost contact with Arduino 1: stop for safety.
    noInterrupts();
    targetLeftMMs = 0;
    targetRightMMs = 0;
    interrupts();
  }
  updateSpeedControl();
}