/*
 * arduino1_uart_bridge.ino
 * ---------------------------------------------------------------------
 * Role: ARDUINO 1 (Master) — UART bridge to PC/Pi, I2C master to Arduino 2.
 *
 * Confirmed-working I2C protocol (matches arduino2_slave.ino v2 exactly):
 *   Write 5 bytes: [0x10, leftMMs_hi, leftMMs_lo, rightMMs_hi, rightMMs_lo]
 *                  CMD_SET_VEL, int16 mm/s per wheel, signed, big-endian.
 *   Write 2 bytes: [0x30, enable]        CMD_SET_ENABLE (0 = stop & hold)
 *   Read  8 bytes: [leftTicks(int32 BE), rightTicks(int32 BE)]
 *
 * UART (PC/Pi <-> Arduino1), 115200 8N1, ASCII lines ending in '\n':
 *
 *   PC/Pi -> Arduino1:
 *     "V <lin_mm_s> <ang_mrad_s>"   wheel command (angular.z optional, mrad/s)
 *     "S <left_x10deg> <right_x10deg>"  arm servo targets, degrees x10 (if PCA9685 wired)
 *     "L <0|1>"                    lift target: 0 = down, 1 = up (if lift wired)
 *     "E <0|1>"                    motor enable / e-stop (0 = disable all)
 *
 *   Arduino1 -> PC/Pi (pushed automatically every TELEMETRY_PERIOD_MS):
 *     "O <dt_ms> <left_ticks> <right_ticks> <btn_upper> <btn_lower> <lift_state>"
 *
 * KEY BEHAVIOR for bench testing: Arduino1 caches the last commanded
 * wheel speed and RE-SENDS it to Arduino 2 every I2C_RESEND_PERIOD_MS,
 * not just when a new "V" line arrives. This is required because
 * Arduino 2 has its own 500ms command watchdog — if the PC/Pi only
 * sends "V" once per phase (as test_forward_backward.py does), Arduino
 * 2 would otherwise stop the wheels under a second later even though
 * the intended command never changed.
 *
 * A SEPARATE, longer watchdog (UART_LINK_TIMEOUT_MS) forces a real stop
 * if no UART traffic has been seen at all for that long — this is the
 * actual "PC/Pi has gone away" safety stop.
 *
 * Servo (PCA9685) and lift (BTS7960 #3) code is included for completeness
 * of the original design, but only runs if that hardware is wired -- it
 * is inert (never called) unless you send "S" or "L" lines.
 * Library dependency: Adafruit PWM Servo Driver Library (only needed if
 * you actually use "S" commands).
 * ---------------------------------------------------------------------
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// =======================================================================
// Configuration
// =======================================================================
static const uint8_t  I2C_SLAVE_ADDR      = 0x08;
static const long     UART_BAUD           = 115200;

static const uint8_t  PIN_LIFT_RPWM       = 5;
static const uint8_t  PIN_LIFT_LPWM       = 6;
static const uint8_t  PIN_BTN_UPPER       = 2;
static const uint8_t  PIN_BTN_LOWER       = 3;

static const uint8_t  SERVO_CH_LEFT_ARM   = 0;
static const uint8_t  SERVO_CH_RIGHT_ARM  = 1;
static const int      SERVO_PULSE_MIN_US  = 500;
static const int      SERVO_PULSE_MAX_US  = 2500;
static const float    SERVO_ANGLE_OFFSET  = 90.0f;
static const float    PCA9685_FREQ_HZ     = 50.0f;

static const uint8_t  LIFT_PWM_SPEED      = 180;
static const unsigned long LIFT_TIMEOUT_MS = 8000;

static const unsigned long TELEMETRY_PERIOD_MS   = 50;   // push rate to PC/Pi
static const unsigned long I2C_RESEND_PERIOD_MS  = 100;  // re-send cached V to Arduino2
static const unsigned long UART_LINK_TIMEOUT_MS  = 3000; // real "PC/Pi gone" stop

static const float HALF_TRACK_MM = 359.0f / 2.0f;  // fullrobotassem.urdf wheel_separation/2

// =======================================================================
// State
// =======================================================================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

enum LiftState : uint8_t {
  LIFT_IDLE = 0, LIFT_MOVING_UP = 1, LIFT_MOVING_DOWN = 2,
  LIFT_AT_UPPER = 3, LIFT_AT_LOWER = 4, LIFT_FAULT = 5
};
LiftState liftState = LIFT_IDLE;
unsigned long liftMoveStartMs = 0;

bool motorsEnabled = true;

// Cached last commanded wheel speeds, resent to Arduino2 continuously.
int16_t cachedLeftMMs  = 0;
int16_t cachedRightMMs = 0;

unsigned long lastUartRxMs   = 0;  // any byte at all -> link alive
unsigned long lastI2cResendMs = 0;
unsigned long lastTelemetryMs = 0;

char lineBuf[64];
uint8_t lineLen = 0;

// =======================================================================
// Servo / lift helpers (unchanged from the original design)
// =======================================================================
void writeServoAngleDeg(uint8_t channel, float angleDeg) {
  angleDeg = constrain(angleDeg, -90.0f, 90.0f);
  float mapped = angleDeg + SERVO_ANGLE_OFFSET;
  long pulse_us = map((long)(mapped * 10), 0, 1800, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  float period_us = 1000000.0f / PCA9685_FREQ_HZ;
  uint16_t ticks = (uint16_t)((pulse_us / period_us) * 4096.0f);
  pwm.setPWM(channel, 0, ticks);
}

void stopLiftMotor() {
  analogWrite(PIN_LIFT_RPWM, 0);
  analogWrite(PIN_LIFT_LPWM, 0);
}

void startLift(uint8_t target) {
  liftMoveStartMs = millis();
  if (target == 1) {
    if (digitalRead(PIN_BTN_UPPER) == LOW) { liftState = LIFT_AT_UPPER; stopLiftMotor(); return; }
    analogWrite(PIN_LIFT_RPWM, LIFT_PWM_SPEED);
    analogWrite(PIN_LIFT_LPWM, 0);
    liftState = LIFT_MOVING_UP;
  } else {
    if (digitalRead(PIN_BTN_LOWER) == LOW) { liftState = LIFT_AT_LOWER; stopLiftMotor(); return; }
    analogWrite(PIN_LIFT_LPWM, LIFT_PWM_SPEED);
    analogWrite(PIN_LIFT_RPWM, 0);
    liftState = LIFT_MOVING_DOWN;
  }
}

void updateLift() {
  bool upperPressed = (digitalRead(PIN_BTN_UPPER) == LOW);
  bool lowerPressed  = (digitalRead(PIN_BTN_LOWER) == LOW);

  if (liftState == LIFT_MOVING_UP) {
    if (upperPressed) { stopLiftMotor(); liftState = LIFT_AT_UPPER; }
    else if (millis() - liftMoveStartMs > LIFT_TIMEOUT_MS) { stopLiftMotor(); liftState = LIFT_FAULT; }
  } else if (liftState == LIFT_MOVING_DOWN) {
    if (lowerPressed) { stopLiftMotor(); liftState = LIFT_AT_LOWER; }
    else if (millis() - liftMoveStartMs > LIFT_TIMEOUT_MS) { stopLiftMotor(); liftState = LIFT_FAULT; }
  }
}

// =======================================================================
// I2C helpers — byte layout confirmed working against arduino2_slave.ino v2
// =======================================================================
void i2cSendVelocity(int16_t leftMMs, int16_t rightMMs) {
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(0x10);
  Wire.write((uint8_t)(leftMMs >> 8));
  Wire.write((uint8_t)(leftMMs & 0xFF));
  Wire.write((uint8_t)(rightMMs >> 8));
  Wire.write((uint8_t)(rightMMs & 0xFF));
  uint8_t result = Wire.endTransmission();
  if (result != 0) {
    Serial.print("!! i2cSendVelocity failed, error code: ");
    Serial.println(result);
  }
}

void i2cSendEnable(bool enabled) {
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(0x30);
  Wire.write(enabled ? 1 : 0);
  Wire.endTransmission();
}

bool i2cReadEncoders(int32_t &leftTicks, int32_t &rightTicks) {
  uint8_t n = Wire.requestFrom((int)I2C_SLAVE_ADDR, 8);
  if (n < 8) return false;
  uint8_t buf[8];
  for (uint8_t i = 0; i < 8; i++) buf[i] = Wire.read();
  leftTicks  = ((int32_t)buf[0] << 24) | ((int32_t)buf[1] << 16) | ((int32_t)buf[2] << 8) | (int32_t)buf[3];
  rightTicks = ((int32_t)buf[4] << 24) | ((int32_t)buf[5] << 16) | ((int32_t)buf[6] << 8) | (int32_t)buf[7];
  return true;
}

void applyVelocityCommand(int16_t linMMs, int16_t angMradS) {
  float delta = ((float)angMradS / 1000.0f) * HALF_TRACK_MM;
  int16_t leftMMs  = (int16_t)(linMMs - delta);
  int16_t rightMMs = (int16_t)(linMMs + delta);
  if (!motorsEnabled) { leftMMs = 0; rightMMs = 0; }
  cachedLeftMMs = leftMMs;
  cachedRightMMs = rightMMs;
  i2cSendVelocity(cachedLeftMMs, cachedRightMMs);   // send immediately too
  lastI2cResendMs = millis();                        // and reset the resend timer
}

// =======================================================================
// UART line parsing
// =======================================================================
void handleLine(char *line) {
  if (line[0] == '\0') return;
  char cmd = line[0];

  switch (cmd) {
    case 'V': {
      int lin = 0, ang = 0;
      int matched = sscanf(line + 1, "%d %d", &lin, &ang);
      if (matched >= 1) {
        if (matched == 1) ang = 0;  // allow "V <speed>" with no angular term
        applyVelocityCommand((int16_t)lin, (int16_t)ang);
      }
      break;
    }
    case 'S': {
      int left10 = 0, right10 = 0;
      if (sscanf(line + 1, "%d %d", &left10, &right10) == 2) {
        writeServoAngleDeg(SERVO_CH_LEFT_ARM, left10 / 10.0f);
        writeServoAngleDeg(SERVO_CH_RIGHT_ARM, right10 / 10.0f);
      }
      break;
    }
    case 'L': {
      int target = 0;
      if (sscanf(line + 1, "%d", &target) == 1) startLift(target ? 1 : 0);
      break;
    }
    case 'E': {
      int en = 1;
      if (sscanf(line + 1, "%d", &en) == 1) {
        motorsEnabled = (en != 0);
        i2cSendEnable(motorsEnabled);
        if (!motorsEnabled) {
          cachedLeftMMs = 0; cachedRightMMs = 0;
          stopLiftMotor();
          liftState = LIFT_IDLE;
        }
      }
      break;
    }
    default:
      break;  // unknown command: ignore
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    lastUartRxMs = millis();  // any byte counts as "link alive"

    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;  // overflow: drop the line
    }
  }
}

void sendTelemetry() {
  int32_t leftTicks = 0, rightTicks = 0;
  i2cReadEncoders(leftTicks, rightTicks);

  unsigned long now = millis();
  unsigned long dt = now - lastTelemetryMs;
  lastTelemetryMs = now;

  bool upperPressed = (digitalRead(PIN_BTN_UPPER) == LOW);
  bool lowerPressed  = (digitalRead(PIN_BTN_LOWER) == LOW);

  Serial.print('O'); Serial.print(' ');
  Serial.print(dt); Serial.print(' ');
  Serial.print(leftTicks); Serial.print(' ');
  Serial.print(rightTicks); Serial.print(' ');
  Serial.print(upperPressed ? 1 : 0); Serial.print(' ');
  Serial.print(lowerPressed ? 1 : 0); Serial.print(' ');
  Serial.println((int)liftState);
}

// =======================================================================
// Arduino entry points
// =======================================================================
void setup() {
  Serial.begin(UART_BAUD);
  Serial.println("ARDUINO1_UART_BRIDGE_READY");

  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);

  pinMode(PIN_LIFT_RPWM, OUTPUT);
  pinMode(PIN_LIFT_LPWM, OUTPUT);
  pinMode(PIN_BTN_UPPER, INPUT_PULLUP);
  pinMode(PIN_BTN_LOWER, INPUT_PULLUP);
  stopLiftMotor();

  pwm.begin();
  pwm.setPWMFreq(PCA9685_FREQ_HZ);
  delay(10);

  lastUartRxMs = millis();
  lastI2cResendMs = millis();
  lastTelemetryMs = millis();
}

void loop() {
  pollSerial();
  updateLift();

  unsigned long now = millis();

  // Real safety stop: PC/Pi UART link has gone completely silent.
  if (now - lastUartRxMs > UART_LINK_TIMEOUT_MS) {
    if (cachedLeftMMs != 0 || cachedRightMMs != 0) {
      Serial.println("!! UART link timeout -> forcing wheel stop.");
    }
    cachedLeftMMs = 0;
    cachedRightMMs = 0;
  }

  // Keep Arduino2's command watchdog satisfied, regardless of UART traffic rate.
  if (now - lastI2cResendMs >= I2C_RESEND_PERIOD_MS) {
    i2cSendVelocity(cachedLeftMMs, cachedRightMMs);
    lastI2cResendMs = now;
  }

  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    sendTelemetry();
  }
}
