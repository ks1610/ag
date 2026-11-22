/*
 * arduino1_forward_test.ino  (v2 — resends velocity to satisfy the slave's watchdog)
 * ---------------------------------------------------------------------
 * STANDALONE BENCH TEST for Arduino 1 — no UART/PC/Pi involved.
 * Open Serial Monitor at 115200 baud to watch progress.
 * ---------------------------------------------------------------------
 */

#include <Wire.h>

static const uint8_t I2C_SLAVE_ADDR = 0x08;

static const int16_t FORWARD_SPEED_MMS = 150;
static const float   TICKS_PER_REV     = 990.0f;
static const float   ROUNDS            = 5.0f;
static const long    TARGET_TICKS      = (long)(ROUNDS * TICKS_PER_REV);

static const unsigned long CONTROL_PERIOD_MS = 100;  // poll + resend period
static const unsigned long START_DELAY_MS    = 2000;

long accumLeft = 0;
long accumRight = 0;
bool done = false;
unsigned long lastPollMs = 0;

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
    Serial.print("!! i2cReadEncoders: requestFrom got ");
    Serial.print(n);
    Serial.println(" bytes (expected 8, no/short reply).");
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

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Wire.begin();
  Wire.setClock(100000);

  Serial.println("ARDUINO1_FORWARD_TEST_READY");
  Serial.print("Target: ");
  Serial.print(ROUNDS);
  Serial.print(" rounds = ");
  Serial.print(TARGET_TICKS);
  Serial.println(" ticks");

  delay(START_DELAY_MS);
  lastPollMs = millis();
}

void loop() {
  if (done) return;

  if (millis() - lastPollMs < CONTROL_PERIOD_MS) return;
  lastPollMs = millis();

  // Keep resending the velocity command every cycle so Arduino 2's
  // 500ms command watchdog never times out mid-run.
  i2cSendVelocity(FORWARD_SPEED_MMS, FORWARD_SPEED_MMS);
  delay(5); // tiny gap before the request, mirrors the hello/hi test's delay(50) but shorter

  int32_t dLeft = 0, dRight = 0;
  if (i2cReadEncoders(dLeft, dRight)) {
    accumLeft += labs(dLeft);
    accumRight += labs(dRight);
    long avg = (accumLeft + accumRight) / 2;

    Serial.print("left=");
    Serial.print(accumLeft);
    Serial.print(" right=");
    Serial.print(accumRight);
    Serial.print(" avg=");
    Serial.print(avg);
    Serial.print("/");
    Serial.println(TARGET_TICKS);

    if (avg >= TARGET_TICKS) {
      Serial.println("Target reached -> sending STOP.");
      i2cSendVelocity(0, 0);
      done = true;
      Serial.println("DONE.");
    }
  }
}