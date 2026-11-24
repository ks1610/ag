/*
 * arduino2_slave.ino  (v5, closed-loop wheel speed)
 * ---------------------------------------------------------------------
 * Role: ARDUINO 2 (Slave), I2C address 0x08. USB used only for debug.
 *
 * v5 CHANGES
 *  - The wheel speed command (mm/s) is now regulated with a feedforward +
 *    PI loop using the encoders. The old fixed minimum PWM (110) is gone,
 *    so slow commands produce slow motion instead of ~0.12 m/s.
 *  - Commands below MIN_TARGET_MMS are treated as zero (they are below
 *    what the motors can do and would only cause stick-slip).
 *  - Separate mm-per-tick per wheel, so a veer can be trimmed here.
 *  - Output is capped by PWM_LIMIT as a safety if an encoder sign is wrong.
 *
 * Kept from v3: no noInterrupts()/interrupts() inside the I2C handlers,
 * atomic watchdog read, rate-limited debug printing.
 *
 * Motor outputs (unchanged wiring):
 *   Forward  : RPWM pin (D4/D12) = software PWM from the Timer2 ISR
 *   Backward : LPWM pin (D6/D10) = analogWrite
 *
 * I2C protocol (Arduino1 = master):
 *   Write 5 bytes: [0x10, leftMMs_hi, leftMMs_lo, rightMMs_hi, rightMMs_lo]
 *   Write 2 bytes: [0x30, enable]
 *   Read  8 bytes: [leftTicks(int32 BE), rightTicks(int32 BE)] = delta since
 *                  the previous read (counters advance on every read).
 * ---------------------------------------------------------------------
 */

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
static const int   PWM_MAX             = 255;

// Software PWM for forward: 64 steps at about 496 Hz.
static const uint8_t SW_STEPS = 64;

// ---- Closed-loop wheel speed ----------------------------------------
// 2 * pi * 30 mm wheel radius / 1342.5 ticks per rev = 0.1404 mm per tick.
// If the robot veers after the Pi's LEFT_SLOWER_PERCENT is set to 0:
//   veers toward the LEFT  -> lower MM_PER_TICK_LEFT slightly (e.g. 0.1404 -> 0.136)
//   veers toward the RIGHT -> lower MM_PER_TICK_RIGHT slightly
static const float MM_PER_TICK_LEFT  = 0.1404f;
static const float MM_PER_TICK_RIGHT = 0.1404f;

// Encoder direction: +1 if positive ticks mean the wheel moves forward.
// From the logs: left counts up going forward, right counts DOWN going forward.
static const int ENC_SIGN_LEFT  = 1;
static const int ENC_SIGN_RIGHT = -1;

static const float KP        = 0.30f;   // PWM per mm/s of error
static const float KI        = 2.0f;    // PWM per (mm/s * s)
static const float FF_OFFSET = 60.0f;   // feedforward start (PWM units)
static const float FF_SLOPE  = 0.5f;    // feedforward PWM per mm/s
static const float INTEG_MIN = -60.0f;
static const float INTEG_MAX = 120.0f;
static const int   PWM_LIMIT = 220;     // safety cap (max 255)

static const int   MIN_TARGET_MMS = 10; // |target| below this is treated as 0
static const float SPEED_FILTER   = 0.4f;

static const unsigned long CONTROL_PERIOD_MS   = 20;
static const unsigned long CMD_TIMEOUT_MS      = 500;
static const unsigned long DEBUG_MIN_PERIOD_MS = 1000;
static const unsigned long ENC_PRINT_PERIOD_MS = 500;
static const unsigned long CTL_PRINT_PERIOD_MS = 500;

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
unsigned long lastCtlPrintMs = 0;
int16_t lastPrintedLeft = 0, lastPrintedRight = 0;
long lastPrintedEncL = 0, lastPrintedEncR = 0;
bool watchdogTripped = false;

// Per-wheel controller state
struct WheelCtl {
  long   prevTicks;
  float  speed;      // filtered measured speed, mm/s, + = forward
  float  integ;      // integral term, PWM units
  int8_t lastSign;   // direction of the last nonzero target
};
WheelCtl ctlL = {0, 0.0f, 0.0f, 0};
WheelCtl ctlR = {0, 0.0f, 0.0f, 0};

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

void resetCtl(WheelCtl &c, long ticks) {
  c.prevTicks = ticks;
  c.speed = 0.0f;
  c.integ = 0.0f;
  c.lastSign = 0;
}

// Updates the measured speed and returns a PWM magnitude (0..PWM_LIMIT)
// for the commanded direction. Returns 0 when the target is (near) zero.
int computePwm(WheelCtl &c, long ticksNow, int encSign, float mmPerTick,
               int16_t targetMMs, float dt) {
  long d = ticksNow - c.prevTicks;
  c.prevTicks = ticksNow;

  float meas = (float)encSign * (float)d * mmPerTick / dt;
  c.speed += SPEED_FILTER * (meas - c.speed);

  if (abs(targetMMs) < MIN_TARGET_MMS) {
    c.integ = 0.0f;
    c.lastSign = 0;
    return 0;
  }

  int8_t sgn = (targetMMs > 0) ? 1 : -1;
  if (sgn != c.lastSign) {          // direction change: restart the integral
    c.integ = 0.0f;
    c.lastSign = sgn;
  }

  float tgt = fabsf((float)constrain((long)targetMMs,
                                     -(long)MAX_WHEEL_SPEED_MMS,
                                     (long)MAX_WHEEL_SPEED_MMS));
  float spd = (sgn > 0) ? c.speed : -c.speed;  // speed along the commanded direction
  float err = tgt - spd;

  float out = FF_OFFSET + FF_SLOPE * tgt + KP * err + c.integ;

  // Anti-windup: do not integrate further into a saturated output.
  bool satHigh = (out >= PWM_LIMIT) && (err > 0.0f);
  bool satLow  = (out <= 0.0f)      && (err < 0.0f);
  if (!satHigh && !satLow) {
    c.integ += err * KI * dt;
    c.integ = constrain(c.integ, INTEG_MIN, INTEG_MAX);
    out = FF_OFFSET + FF_SLOPE * tgt + KP * err + c.integ;
  }

  return (int)constrain(out, 0.0f, (float)PWM_LIMIT);
}

void driveWheelPwm(uint8_t enPin, uint8_t lpwmPin, bool isLeft,
                   int16_t speedMMs, int pwm) {
  bool enabled = i2cLinkEstablished && motorsEnabled;
  uint8_t fwdDuty = 0;
  int bwdPwm = 0;

  if (enabled && abs(speedMMs) >= MIN_TARGET_MMS) {
    if (speedMMs > 0) fwdDuty = (uint8_t)constrain((pwm * SW_STEPS) / 255, 0, SW_STEPS);
    else              bwdPwm  = constrain(pwm, 0, PWM_MAX);
  }

  // Keep the shared PORT registers consistent with the ISR.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    digitalWrite(enPin, enabled ? HIGH : LOW);
    if (isLeft) swDutyL = fwdDuty;
    else        swDutyR = fwdDuty;
    analogWrite(lpwmPin, bwdPwm);   // backward; 0 when going forward or stopped
  }
}

void updateMotors() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastControlMs;
  if (elapsed < CONTROL_PERIOD_MS) return;
  lastControlMs = now;
  float dt = elapsed / 1000.0f;
  if (dt <= 0.0f) return;

  long ticksL, ticksR;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    ticksL = encTicksLeft;
    ticksR = encTicksRight;
  }

  if (!i2cLinkEstablished) {
    stopAllMotors();
    resetCtl(ctlL, ticksL);
    resetCtl(ctlR, ticksR);
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

  int pwmL = computePwm(ctlL, ticksL, ENC_SIGN_LEFT,  MM_PER_TICK_LEFT,  tgtLeft,  dt);
  int pwmR = computePwm(ctlR, ticksR, ENC_SIGN_RIGHT, MM_PER_TICK_RIGHT, tgtRight, dt);

  driveWheelPwm(PIN_L_EN, PIN_L_LPWM, true,  tgtLeft,  pwmL);
  driveWheelPwm(PIN_R_EN, PIN_R_LPWM, false, tgtRight, pwmR);

  // Controller debug (only while a wheel command is active).
  if ((tgtLeft != 0 || tgtRight != 0) && now - lastCtlPrintMs >= CTL_PRINT_PERIOD_MS) {
    lastCtlPrintMs = now;
    Serial.print("CTL tgt L="); Serial.print(tgtLeft);
    Serial.print(" R=");        Serial.print(tgtRight);
    Serial.print(" meas L=");   Serial.print(ctlL.speed, 0);
    Serial.print(" R=");        Serial.print(ctlR.speed, 0);
    Serial.print(" pwm L=");    Serial.print(pwmL);
    Serial.print(" R=");        Serial.println(pwmR);
  }
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
  Serial.println("ARDUINO2_SLAVE_V5_CLOSED_LOOP_READY (addr 0x08), waiting for I2C...");
}

void loop() {
  printDebug();
  updateMotors();
}