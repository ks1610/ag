// /*
//  * arduino2_slave.ino  (v3)
//  * ---------------------------------------------------------------------
//  * Role: ARDUINO 2 (Slave), I2C address 0x08. USB used only for debug.
//  *
//  * v3 FIXES
//  *  - onI2CRequest() no longer calls noInterrupts()/interrupts(). That code
//  *    runs inside the TWI interrupt; calling interrupts() there re-entered
//  *    the handler recursively and hung the I2C bus (SCL held low).
//  *  - Watchdog reads lastI2cCmdMs atomically and prints only once.
//  *  - Debug printing is rate-limited; ENC totals printed for verification.
//  *
//  * Motor behaviour is unchanged from the version that moved the robot:
//  *   Forward  : RPWM pin (D4/D12, not PWM-capable) = digital HIGH (full speed)
//  *   Backward : LPWM pin (D6/D10, PWM) = analogWrite, floor MIN_EFFECTIVE_PWM
//  *
//  * I2C protocol (Arduino1 = master):
//  *   Write 5 bytes: [0x10, leftMMs_hi, leftMMs_lo, rightMMs_hi, rightMMs_lo]
//  *   Write 2 bytes: [0x30, enable]
//  *   Read  8 bytes: [leftTicks(int32 BE), rightTicks(int32 BE)] = delta since
//  *                  the previous read (counters advance on every read).
//  * ---------------------------------------------------------------------
//  */

#include <Wire.h>
#include <util/atomic.h>

// =======================================================================
// Configuration (pins unchanged from current wiring)
// =======================================================================
static const uint8_t I2C_SLAVE_ADDR = 0x08;

static const uint8_t PIN_L_EN    = 5;
static const uint8_t PIN_L_RPWM  = 4;    // forward, software PWM (PD4)
static const uint8_t PIN_L_LPWM  = 6;    // backward, hardware PWM
static const uint8_t PIN_L_ENC_A = 2;    // INT0
static const uint8_t PIN_L_ENC_B = 7;

static const uint8_t PIN_R_EN    = 11;
static const uint8_t PIN_R_RPWM  = 12;   // forward, software PWM (PB4)
static const uint8_t PIN_R_LPWM  = 10;   // backward, hardware PWM
static const uint8_t PIN_R_ENC_A = 3;    // INT1
static const uint8_t PIN_R_ENC_B = 8;

static const float MAX_WHEEL_SPEED_MMS = 400.0f;
static const int   PWM_MAX           = 255;
static const int   MIN_EFFECTIVE_PWM = 110;   // 0-255 scale, used for both directions

// Software PWM for forward: 64 steps at about 496 Hz.
static const uint8_t SW_STEPS    = 64;
static const long    SW_MIN_DUTY = MIN_EFFECTIVE_PWM / 4;   // same starting point as backward

// Per-wheel, per-direction gains (1.0 = no change). Use these to correct drift.
static const float GAIN_LEFT_FWD  = 1.00f;
static const float GAIN_RIGHT_FWD = 1.00f;
static const float GAIN_LEFT_BWD  = 1.00f;
static const float GAIN_RIGHT_BWD = 1.00f;

static const unsigned long CONTROL_PERIOD_MS   = 20;
static const unsigned long CMD_TIMEOUT_MS      = 500;
static const unsigned long DEBUG_MIN_PERIOD_MS = 1000;
static const unsigned long ENC_PRINT_PERIOD_MS = 500;

// =======================================================================
// State
// =======================================================================
volatile long encTicksLeft  = 0;
volatile long encTicksRight = 0;

// Only touched inside onI2CRequest() (interrupt context).
long encSnapshotLeft  = 0;
long encSnapshotRight = 0;

volatile int16_t targetLeftMMs  = 0;
volatile int16_t targetRightMMs = 0;
volatile bool    motorsEnabled  = true;
volatile bool    i2cLinkEstablished = false;
volatile unsigned long lastI2cCmdMs = 0;

// Software PWM state (written by the Timer2 ISR)
volatile uint8_t swPhase = 0;
volatile uint8_t swDutyL = 0;   // 0..SW_STEPS, 0 = off
volatile uint8_t swDutyR = 0;

// ISR -> loop() debug hand-off
volatile bool    debugNewMsgFlag  = false;
volatile uint8_t debugCmd         = 0;
volatile int16_t debugTargetLeft  = 0;
volatile int16_t debugTargetRight = 0;
volatile uint8_t debugNumBytes    = 0;

unsigned long lastControlMs = 0;
unsigned long lastDebugPrintMs = 0;
unsigned long lastEncPrintMs = 0;
int16_t lastPrintedLeft = 0, lastPrintedRight = 0;
long lastPrintedEncL = 0, lastPrintedEncR = 0;
bool watchdogTripped = false;

// =======================================================================
// Encoder ISRs (channel A RISING, channel B gives direction)
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
// Software PWM: Timer2 interrupt, about 31.7 kHz / 64 steps = about 496 Hz
// =======================================================================
ISR(TIMER2_COMPA_vect) {
  swPhase = (swPhase + 1) & (SW_STEPS - 1);

  if (swPhase < swDutyL) PORTD |=  (1 << PD4);    // pin 4
  else                   PORTD &= ~(1 << PD4);

  if (swPhase < swDutyR) PORTB |=  (1 << PB4);    // pin 12
  else                   PORTB &= ~(1 << PB4);
}

void setupSoftPwmTimer() {
  TCCR2A = (1 << WGM21);     // CTC mode
  TCCR2B = (1 << CS21);      // prescaler 8
  OCR2A  = 62;               // 16 MHz / 8 / 63 = about 31.7 kHz
  TIMSK2 |= (1 << OCIE2A);   // enable compare-match interrupt
}

// =======================================================================
// Motor driver
// =======================================================================
void stopAllMotors() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    digitalWrite(PIN_L_EN, LOW);
    digitalWrite(PIN_R_EN, LOW);
    swDutyL = 0;
    swDutyR = 0;
  }
  analogWrite(PIN_L_LPWM, 0);
  analogWrite(PIN_R_LPWM, 0);
}

void driveWheel(uint8_t enPin, uint8_t lpwmPin, bool isLeft, int16_t speedMMs) {
  bool active = i2cLinkEstablished && motorsEnabled;
  uint8_t fwdDuty = 0;
  int bwdPwm = 0;

  if (active && speedMMs != 0) {
    long mag = constrain((long)abs(speedMMs), 0L, (long)MAX_WHEEL_SPEED_MMS);

    if (speedMMs > 0) {
      float gain = isLeft ? GAIN_LEFT_FWD : GAIN_RIGHT_FWD;
      long d = map(mag, 0L, (long)MAX_WHEEL_SPEED_MMS, SW_MIN_DUTY, (long)SW_STEPS);
      fwdDuty = (uint8_t)constrain((long)(d * gain), 0L, (long)SW_STEPS);
    } else {
      float gain = isLeft ? GAIN_LEFT_BWD : GAIN_RIGHT_BWD;
      long p = map(mag, 0L, (long)MAX_WHEEL_SPEED_MMS,
                   (long)MIN_EFFECTIVE_PWM, (long)PWM_MAX);
      bwdPwm = constrain((int)(p * gain), 0, PWM_MAX);
    }
  }

  // Keep the shared PORT registers consistent with the ISR.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    digitalWrite(enPin, active ? HIGH : LOW);
    if (isLeft) swDutyL = fwdDuty;
    else        swDutyR = fwdDuty;
    analogWrite(lpwmPin, bwdPwm);   // backward; 0 when going forward or stopped
  }
}

void updateMotors() {
  unsigned long now = millis();
  if (now - lastControlMs < CONTROL_PERIOD_MS) return;
  lastControlMs = now;

  if (!i2cLinkEstablished) {
    stopAllMotors();
    return;
  }

  // Watchdog: atomic copy of the ISR-written timestamp, signed difference.
  unsigned long lastCmd;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { lastCmd = lastI2cCmdMs; }

  if ((long)(now - lastCmd) > (long)CMD_TIMEOUT_MS) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      targetLeftMMs = 0;
      targetRightMMs = 0;
    }
    if (!watchdogTripped) {
      Serial.println("!! Watchdog: no I2C command for 500ms -> stopped.");
      watchdogTripped = true;
    }
  } else {
    watchdogTripped = false;
  }

  int16_t tgtLeft, tgtRight;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    tgtLeft = targetLeftMMs;
    tgtRight = targetRightMMs;
  }

  driveWheel(PIN_L_EN, PIN_L_LPWM, true, tgtLeft);
  driveWheel(PIN_R_EN, PIN_R_LPWM, false, tgtRight);
}

// =======================================================================
// I2C callbacks (run inside the TWI interrupt: keep short, no Serial)
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
  // Encoder ISRs are masked while this handler runs, so these reads are safe.
  long curLeft  = encTicksLeft;
  long curRight = encTicksRight;
  long dLeft  = curLeft  - encSnapshotLeft;
  long dRight = curRight - encSnapshotRight;
  encSnapshotLeft  = curLeft;
  encSnapshotRight = curRight;

  uint8_t buf[8];
  buf[0] = (uint8_t)((dLeft >> 24) & 0xFF);
  buf[1] = (uint8_t)((dLeft >> 16) & 0xFF);
  buf[2] = (uint8_t)((dLeft >> 8) & 0xFF);
  buf[3] = (uint8_t)(dLeft & 0xFF);
  buf[4] = (uint8_t)((dRight >> 24) & 0xFF);
  buf[5] = (uint8_t)((dRight >> 16) & 0xFF);
  buf[6] = (uint8_t)((dRight >> 8) & 0xFF);
  buf[7] = (uint8_t)(dRight & 0xFF);
  Wire.write(buf, 8);
}

// =======================================================================
// Debug printing (main loop only)
// =======================================================================
void printDebug() {
  unsigned long now = millis();

  if (debugNewMsgFlag) {
    uint8_t cmd, n;
    int16_t left, right;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      cmd = debugCmd;
      left = debugTargetLeft;
      right = debugTargetRight;
      n = debugNumBytes;
      debugNewMsgFlag = false;
    }

    bool changed = (left != lastPrintedLeft) || (right != lastPrintedRight);
    if (changed || (now - lastDebugPrintMs >= DEBUG_MIN_PERIOD_MS)) {
      lastDebugPrintMs = now;
      lastPrintedLeft = left;
      lastPrintedRight = right;

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
  }

  if (now - lastEncPrintMs >= ENC_PRINT_PERIOD_MS) {
    lastEncPrintMs = now;
    long l, r;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      l = encTicksLeft;
      r = encTicksRight;
    }
    if (l != lastPrintedEncL || r != lastPrintedEncR) {
      lastPrintedEncL = l;
      lastPrintedEncR = r;
      Serial.print("ENC total left=");
      Serial.print(l);
      Serial.print(" right=");
      Serial.println(r);
    }
  }
}

// =======================================================================
// Arduino entry points
// =======================================================================
void setup() {
  Serial.begin(115200);

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

  setupSoftPwmTimer();

  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  lastControlMs = millis();
  Serial.println("ARDUINO2_SLAVE_V4_READY (addr 0x08), waiting for I2C...");
}

void loop() {
  printDebug();
  updateMotors();
}