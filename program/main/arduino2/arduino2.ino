/*
 * arduino2_slave.ino  (v2 — fixes forward direction, see comments below)
 * ---------------------------------------------------------------------
 * Role: ARDUINO 2 (Slave)
 *
 * HARDWARE NOTE (confirmed against the working standalone test sketch):
 *   RPWM pins (D4 left, D12 right) are NOT PWM-capable on this wiring —
 *   analogWrite() on them silently falls back to HIGH/LOW. So forward
 *   drive is on/off only (full speed when active), same as the proven
 *   test sketch effectively did. LPWM pins (D6 left, D10 right) ARE
 *   real PWM pins, so backward drive gets true variable speed, with a
 *   minimum-duty floor (MIN_EFFECTIVE_PWM) so a modest commanded speed
 *   can't land below the duty the motor needs to actually start moving.
 * ---------------------------------------------------------------------
 */

#include <Wire.h>

// =======================================================================
// Configuration
// =======================================================================
static const uint8_t I2C_SLAVE_ADDR = 0x08;

static const uint8_t PIN_L_EN    = 5;
static const uint8_t PIN_L_RPWM  = 4;   // NOT PWM-capable -> digital only
static const uint8_t PIN_L_LPWM  = 6;   // PWM-capable
static const uint8_t PIN_L_ENC_A = 2;   // INT0
static const uint8_t PIN_L_ENC_B = 7;

static const uint8_t PIN_R_EN    = 11;
static const uint8_t PIN_R_RPWM  = 12;  // NOT PWM-capable -> digital only
static const uint8_t PIN_R_LPWM  = 10;  // PWM-capable
static const uint8_t PIN_R_ENC_A = 3;   // INT1
static const uint8_t PIN_R_ENC_B = 8;

static const float MAX_WHEEL_SPEED_MMS = 400.0f;
static const int   PWM_MAX = 255;
// Floor duty for the PWM (backward) side, close to the values your
// proven sketch used (150-160), so any nonzero command actually moves
// the motor instead of landing in a too-weak-to-turn duty range.
static const int   MIN_EFFECTIVE_PWM = 110;

static const unsigned long CONTROL_PERIOD_MS = 20;
static const unsigned long CMD_TIMEOUT_MS    = 500;

// =======================================================================
// State
// =======================================================================
volatile long encTicksLeft  = 0;
volatile long encTicksRight = 0;

long encSnapshotLeftForI2C  = 0;
long encSnapshotRightForI2C = 0;

volatile int16_t targetLeftMMs  = 0;
volatile int16_t targetRightMMs = 0;
volatile bool    motorsEnabled  = true;
volatile bool    i2cLinkEstablished = false;
volatile unsigned long lastI2cCmdMs = 0;

volatile bool    debugNewMsgFlag  = false;
volatile uint8_t debugCmd         = 0;
volatile int16_t debugTargetLeft  = 0;
volatile int16_t debugTargetRight = 0;
volatile uint8_t debugNumBytes    = 0;

unsigned long lastControlMs = 0;

// =======================================================================
// Encoder ISRs
// =======================================================================
void isrLeftEncoder() {
  if (digitalRead(PIN_L_ENC_B) == LOW) encTicksLeft++;
  else encTicksLeft--;
}

void isrRightEncoder() {
  if (digitalRead(PIN_R_ENC_B) == LOW) encTicksRight++;
  else encTicksRight--;
}

// =======================================================================
// Motor driver
//   Forward (speedMMs > 0): RPWM pin driven as plain digital HIGH (the
//     pin can't PWM anyway) — full-speed-only, same as proven behavior.
//   Backward (speedMMs < 0): LPWM pin gets real analogWrite PWM, scaled
//     between MIN_EFFECTIVE_PWM and PWM_MAX.
// =======================================================================
void setWheelSpeed(uint8_t enPin, uint8_t rpwmPin, uint8_t lpwmPin, int16_t speedMMs) {
  bool active = i2cLinkEstablished && motorsEnabled;
  digitalWrite(enPin, active ? HIGH : LOW);

  if (!active || speedMMs == 0) {
    digitalWrite(rpwmPin, LOW);
    analogWrite(lpwmPin, 0);
    return;
  }

  if (speedMMs > 0) {
    // Forward: digital-only pin -> full on. (No partial speed possible
    // here without rewiring RPWM to a PWM-capable pin or moving speed
    // control onto the EN pin instead — see note at the end.)
    digitalWrite(rpwmPin, HIGH);
    analogWrite(lpwmPin, 0);
  } else {
    long mag = constrain((long)abs(speedMMs), 0L, (long)MAX_WHEEL_SPEED_MMS);
    int pwm = (int)map(mag, 0L, (long)MAX_WHEEL_SPEED_MMS,
                        (long)MIN_EFFECTIVE_PWM, (long)PWM_MAX);
    digitalWrite(rpwmPin, LOW);
    analogWrite(lpwmPin, pwm);
  }
}

void stopAllMotors() {
  digitalWrite(PIN_L_EN, LOW);
  digitalWrite(PIN_R_EN, LOW);
  digitalWrite(PIN_L_RPWM, LOW);
  analogWrite(PIN_L_LPWM, 0);
  digitalWrite(PIN_R_RPWM, LOW);
  analogWrite(PIN_R_LPWM, 0);
}

void updateMotors() {
  unsigned long now = millis();
  if (now - lastControlMs < CONTROL_PERIOD_MS) return;
  lastControlMs = now;

  if (!i2cLinkEstablished) {
    stopAllMotors();
    return;
  }

  if (now - lastI2cCmdMs > CMD_TIMEOUT_MS) {
    noInterrupts();
    targetLeftMMs = 0;
    targetRightMMs = 0;
    interrupts();
    Serial.println("!! Watchdog: no I2C command for 500ms -> forcing stop.");
  }

  int16_t tgtLeft, tgtRight;
  noInterrupts();
  tgtLeft = targetLeftMMs;
  tgtRight = targetRightMMs;
  interrupts();

  setWheelSpeed(PIN_L_EN, PIN_L_RPWM, PIN_L_LPWM, tgtLeft);
  setWheelSpeed(PIN_R_EN, PIN_R_RPWM, PIN_R_LPWM, tgtRight);
}

// =======================================================================
// I2C callbacks — KEEP THESE FAST, no Serial calls in here.
// =======================================================================
void onI2CReceive(int numBytes) {
  if (numBytes <= 0) return;
  uint8_t cmd = Wire.read();

  i2cLinkEstablished = true;
  lastI2cCmdMs = millis();

  int16_t newLeft = targetLeftMMs;
  int16_t newRight = targetRightMMs;

  if (cmd == 0x10 && numBytes >= 5) {
    uint8_t b1 = Wire.read();
    uint8_t b2 = Wire.read();
    uint8_t b3 = Wire.read();
    uint8_t b4 = Wire.read();
    newLeft  = (int16_t)((b1 << 8) | b2);
    newRight = (int16_t)((b3 << 8) | b4);
    targetLeftMMs  = newLeft;
    targetRightMMs = newRight;
  } else if (cmd == 0x30 && numBytes >= 2) {
    uint8_t en = Wire.read();
    motorsEnabled = (en != 0);
    if (!motorsEnabled) { targetLeftMMs = 0; targetRightMMs = 0; }
    newLeft = targetLeftMMs;
    newRight = targetRightMMs;
  }
  while (Wire.available()) Wire.read();

  debugCmd = cmd;
  debugTargetLeft = newLeft;
  debugTargetRight = newRight;
  debugNumBytes = (uint8_t)numBytes;
  debugNewMsgFlag = true;
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
  Serial.begin(115200);
  while (!Serial) { ; }

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
  attachInterrupt(digitalPinToInterrupt(PIN_L_ENC_A), isrLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_R_ENC_A), isrRightEncoder, RISING);

  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  lastControlMs = millis();

  Serial.println("ARDUINO2_SLAVE_READY (addr 0x08), waiting for I2C...");
}

void loop() {
  if (debugNewMsgFlag) {
    noInterrupts();
    uint8_t cmd = debugCmd;
    int16_t left = debugTargetLeft;
    int16_t right = debugTargetRight;
    uint8_t n = debugNumBytes;
    debugNewMsgFlag = false;
    interrupts();

    Serial.print("Received from Arduino 1: ");
    if (cmd == 0x10) {
      Serial.print("CMD_SET_VEL left=");
      Serial.print(left);
      Serial.print("mm/s right=");
      Serial.print(right);
      Serial.println("mm/s");
    } else if (cmd == 0x30) {
      Serial.print("CMD_SET_ENABLE enabled=");
      Serial.println(motorsEnabled ? 1 : 0);
    } else {
      Serial.print("unknown cmd=0x");
      Serial.print(cmd, HEX);
      Serial.print(" numBytes=");
      Serial.println(n);
    }
  }

  updateMotors();
}