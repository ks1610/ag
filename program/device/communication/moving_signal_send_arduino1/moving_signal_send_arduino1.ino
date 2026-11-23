/*
 * arduino1_forward_test.ino  (v2 — resends velocity to satisfy the slave's watchdog)
 * ---------------------------------------------------------------------
 * STANDALONE BENCH TEST for Arduino 1 — no UART/PC/Pi involved.
 * Open Serial Monitor at 115200 baud to watch progress.
 * ---------------------------------------------------------------------
 */


/*
 * arduino1_forward_test.ino  (v3)
 * ---------------------------------------------------------------------
 * Bench test, no PC/Pi protocol. Arduino 1 drives Arduino 2 over I2C:
 *   FORWARD  ROUNDS wheel revolutions -> pause -> BACKWARD ROUNDS (optional)
 * Each wheel stops on its OWN tick count (same as the motor-only sketch).
 * Open Serial Monitor at 115200 baud to watch progress.
 * ---------------------------------------------------------------------
 */

#include <Wire.h>

static const uint8_t I2C_SLAVE_ADDR = 0x08;

static const int16_t SPEED_MMS         = 150;
static const float   TICKS_PER_REV     = 990.0f;
static const float   ROUNDS            = 5.0f;
static const long    TARGET_TICKS      = (long)(ROUNDS * TICKS_PER_REV);
static const bool    RUN_BACKWARD_TOO  = true;

static const unsigned long CONTROL_PERIOD_MS = 100;    // send + poll period
static const unsigned long START_DELAY_MS    = 2000;
static const unsigned long PAUSE_MS          = 1000;   // between forward/backward
static const unsigned long PHASE_TIMEOUT_MS  = 20000;  // safety stop per phase
static const uint8_t       MAX_READ_FAILURES = 5;      // consecutive failed reads

enum Phase : uint8_t { PH_FORWARD, PH_PAUSE, PH_BACKWARD, PH_DONE };

Phase phase = PH_FORWARD;
int8_t phaseDir = 1;

long accumLeft = 0, accumRight = 0;
bool leftDone = false, rightDone = false;
uint8_t readFailures = 0;

unsigned long lastPollMs = 0;
unsigned long phaseStartMs = 0;
unsigned long pauseEndMs = 0;

// ---------------------------------------------------------------------
// I2C helpers
// ---------------------------------------------------------------------
void i2cSendVelocity(int16_t leftMMs, int16_t rightMMs) {
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(0x10);
  Wire.write((uint8_t)(leftMMs >> 8));
  Wire.write((uint8_t)(leftMMs & 0xFF));
  Wire.write((uint8_t)(rightMMs >> 8));
  Wire.write((uint8_t)(rightMMs & 0xFF));
  uint8_t result = Wire.endTransmission();
  if (result != 0) {
    Serial.print("!! i2cSendVelocity write failed, error code: ");
    Serial.println(result);
  }
}

bool i2cReadEncoders(int32_t &leftTicks, int32_t &rightTicks) {
  uint8_t n = Wire.requestFrom((int)I2C_SLAVE_ADDR, 8);
  if (n < 8) {
    Serial.print("!! i2cReadEncoders: got ");
    Serial.print(n);
    Serial.println(" bytes (expected 8).");
    return false;
  }
  uint8_t buf[8];
  for (uint8_t i = 0; i < 8; i++) buf[i] = Wire.read();
  leftTicks  = ((int32_t)buf[0] << 24) | ((int32_t)buf[1] << 16) |
               ((int32_t)buf[2] << 8)  | (int32_t)buf[3];
  rightTicks = ((int32_t)buf[4] << 24) | ((int32_t)buf[5] << 16) |
               ((int32_t)buf[6] << 8)  | (int32_t)buf[7];
  return true;
}

// ---------------------------------------------------------------------
// Phase control
// ---------------------------------------------------------------------
void startPhase(Phase p, int8_t dir) {
  // Discard any ticks accumulated before this phase (hand-spinning, coasting).
  int32_t dropL, dropR;
  i2cReadEncoders(dropL, dropR);

  phase = p;
  phaseDir = dir;
  accumLeft = 0;
  accumRight = 0;
  leftDone = false;
  rightDone = false;
  readFailures = 0;
  phaseStartMs = millis();

  Serial.print("=== Phase: ");
  Serial.println(p == PH_FORWARD ? "FORWARD" : "BACKWARD");
}

void emergencyStop(const char *reason) {
  i2cSendVelocity(0, 0);
  Serial.print("!! STOP: ");
  Serial.println(reason);
  phase = PH_DONE;
}

void finishPhase() {
  i2cSendVelocity(0, 0);
  if (phase == PH_FORWARD && RUN_BACKWARD_TOO) {
    Serial.println("Forward complete -> pause.");
    phase = PH_PAUSE;
    pauseEndMs = millis() + PAUSE_MS;
  } else {
    Serial.println("DONE.");
    phase = PH_DONE;
  }
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);   // never hang if the bus misbehaves

  Serial.println("ARDUINO1_FORWARD_TEST_V3_READY");
  Serial.print("Target per wheel: ");
  Serial.print(ROUNDS);
  Serial.print(" rounds = ");
  Serial.print(TARGET_TICKS);
  Serial.println(" ticks");

  delay(START_DELAY_MS);
  startPhase(PH_FORWARD, 1);
  lastPollMs = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastPollMs < CONTROL_PERIOD_MS) return;
  lastPollMs = now;

  if (phase == PH_DONE) {
    i2cSendVelocity(0, 0);          // keep the slave in a known stopped state
    return;
  }

  if (phase == PH_PAUSE) {
    i2cSendVelocity(0, 0);
    if (now >= pauseEndMs) startPhase(PH_BACKWARD, -1);
    return;
  }

  // ---- active phase (FORWARD or BACKWARD) ----
  if (now - phaseStartMs > PHASE_TIMEOUT_MS) {
    emergencyStop("phase timeout (encoders not reaching target)");
    return;
  }

  int16_t cmdLeft  = leftDone  ? 0 : (int16_t)(phaseDir * SPEED_MMS);
  int16_t cmdRight = rightDone ? 0 : (int16_t)(phaseDir * SPEED_MMS);
  i2cSendVelocity(cmdLeft, cmdRight);
  delay(5);

  int32_t dLeft = 0, dRight = 0;
  if (!i2cReadEncoders(dLeft, dRight)) {
    if (++readFailures >= MAX_READ_FAILURES) {
      emergencyStop("encoder read failed repeatedly");
    }
    return;
  }
  readFailures = 0;

  accumLeft  += labs(dLeft);
  accumRight += labs(dRight);

  Serial.print("left=");  Serial.print(accumLeft);
  Serial.print(" right="); Serial.print(accumRight);
  Serial.print(" target="); Serial.println(TARGET_TICKS);

  if (!leftDone && accumLeft >= TARGET_TICKS) {
    leftDone = true;
    Serial.println("-> left wheel reached target, stopping it.");
  }
  if (!rightDone && accumRight >= TARGET_TICKS) {
    rightDone = true;
    Serial.println("-> right wheel reached target, stopping it.");
  }

  if (leftDone && rightDone) {
    finishPhase();
  }
}