/*
 * arduino1_master.ino
 * ---------------------------------------------------------------------
 * Role: ARDUINO 1 (Master)
 *   - UART  : talks to the Raspberry Pi (replaces /cmd_vel, /left_arm/cmd_pos,
 *             /right_arm/cmd_pos, /upper_body/cmd_pos that used to go through
 *             ros_gz_bridge + bridge.yaml in simulation).
 *   - I2C   : bus master. Shares the bus with:
 *               - PCA9685 servo driver   (addr 0x40, fixed by the board)
 *               - Arduino 2 ("Slaver")   (addr I2C_SLAVE_ADDR, see below)
 *   - Drives the two arm servos (via PCA9685) and the upper-body lift motor
 *     (via BTS7960 #3) directly, and relays wheel velocity commands /
 *     wheel encoder deltas to/from Arduino 2.
 *
 * Wiring (from ketnoi.xlsx, "Arduino 1 (Master)" sheet):
 *   PCA9685      VCC->5V  GND->GND  SDA->A4  SCL->A5
 *   BTS7960 #3   VCC->5V  GND->GND  EN(R_EN&L_EN)->5V (always on)
 *                RPWM->D5(PWM)      LPWM->D6(PWM)        [upper-body lift]
 *   Arduino 2    VCC->5V  GND->GND  SDA<->A4  SCL<->A5   [shared I2C bus]
 *   Button #1    OUT1->D2   (upper-body UPPER limit switch)
 *   Button #2    OUT2->D3   (upper-body LOWER limit switch)
 *
 * UART (Pi <-> Arduino1), 115200 8N1, ASCII lines terminated by '\n':
 *
 *   Pi -> Arduino1:
 *     "V <lin_mm_s> <ang_mrad_s>"   cmd_vel: linear.x [mm/s], angular.z [mrad/s]
 *     "S <left_x10deg> <right_x10deg>"  arm servo targets, degrees x10
 *     "L <0|1>"                    lift target: 0 = down, 1 = up
 *     "E <0|1>"                    motor enable / e-stop (0 = disable all)
 *
 *   Arduino1 -> Pi (pushed automatically every TELEMETRY_PERIOD_MS):
 *     "O <dt_ms> <left_ticks> <right_ticks> <btn_upper> <btn_lower> <lift_state>"
 *       left_ticks/right_ticks : encoder delta since the previous telemetry line
 *       btn_upper/btn_lower     : 1 = pressed, 0 = released
 *       lift_state              : 0 idle, 1 moving up, 2 moving down,
 *                                  3 at upper limit, 4 at lower limit, 5 fault(timeout)
 *
 * IMPORTANT WIRING NOTE:
 *   The Pi's UART pins are 3.3V-only. Arduino TX is 5V.
 *   Arduino1 TX -> Pi RX MUST go through a level shifter (or a simple
 *   resistor divider, e.g. 10k/20k) to protect the Pi.
 *   Pi TX -> Arduino1 RX can normally be wired directly.
 *
 * Library dependency: Adafruit PWM Servo Driver Library (PCA9685).
 * ---------------------------------------------------------------------
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// =======================================================================
// Configuration — tune these to match the real robot
// =======================================================================
static const uint8_t  I2C_SLAVE_ADDR      = 0x08;   // Arduino 2 address
static const long     UART_BAUD           = 115200;

static const uint8_t  PIN_LIFT_RPWM       = 5;      // BTS7960 #3 RPWM (up)
static const uint8_t  PIN_LIFT_LPWM       = 6;      // BTS7960 #3 LPWM (down)
static const uint8_t  PIN_BTN_UPPER       = 2;       // Button #1 OUT1
static const uint8_t  PIN_BTN_LOWER       = 3;       // Button #2 OUT2

static const uint8_t  SERVO_CH_LEFT_ARM   = 0;       // PCA9685 channel
static const uint8_t  SERVO_CH_RIGHT_ARM  = 1;       // PCA9685 channel
static const int       SERVO_PULSE_MIN_US = 500;     // 0 deg
static const int       SERVO_PULSE_MAX_US = 2500;    // 180 deg
static const float     SERVO_ANGLE_OFFSET = 90.0f;   // maps [-90,90] -> [0,180]
static const float     PCA9685_FREQ_HZ    = 50.0f;

static const uint8_t  LIFT_PWM_SPEED      = 180;     // 0-255, lift motor speed
static const unsigned long LIFT_TIMEOUT_MS = 8000;   // fault if no limit hit

static const unsigned long TELEMETRY_PERIOD_MS = 50; // push rate to Pi
static const unsigned long CMD_WATCHDOG_MS      = 500; // stop wheels if Pi silent

// Wheelbase geometry (from fullrobotassem.urdf): wheel_separation, used to
// convert angular.z into left/right wheel speed deltas.
static const float HALF_TRACK_MM = 359.0f / 2.0f;

// =======================================================================
// State
// =======================================================================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

enum LiftState : uint8_t {
  LIFT_IDLE = 0,
  LIFT_MOVING_UP = 1,
  LIFT_MOVING_DOWN = 2,
  LIFT_AT_UPPER = 3,
  LIFT_AT_LOWER = 4,
  LIFT_FAULT = 5
};

LiftState liftState = LIFT_IDLE;
uint8_t   liftTarget = 0;         // last commanded target (0=down,1=up)
unsigned long liftMoveStartMs = 0;

bool motorsEnabled = true;
int16_t targetLinMMs = 0;         // last cmd_vel linear.x in mm/s
int16_t targetAngMradS = 0;       // last cmd_vel angular.z in mrad/s

unsigned long lastVelCmdMs = 0;
unsigned long lastTelemetryMs = 0;

char lineBuf[64];
uint8_t lineLen = 0;

// =======================================================================
// Helpers
// =======================================================================
void writeServoAngleDeg(uint8_t channel, float angleDeg) {
  angleDeg = constrain(angleDeg, -90.0f, 90.0f);
  float mapped = angleDeg + SERVO_ANGLE_OFFSET;           // -> [0,180]
  long pulse_us = map((long)(mapped * 10), 0, 1800,
                       SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  // Convert microseconds to PCA9685 "ticks" (12-bit, 4096 steps per period).
  float period_us = 1000000.0f / PCA9685_FREQ_HZ;
  uint16_t ticks = (uint16_t)((pulse_us / period_us) * 4096.0f);
  pwm.setPWM(channel, 0, ticks);
}

void stopLiftMotor() {
  analogWrite(PIN_LIFT_RPWM, 0);
  analogWrite(PIN_LIFT_LPWM, 0);
}

void startLift(uint8_t target /*0=down,1=up*/) {
  liftTarget = target;
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
    if (upperPressed) {
      stopLiftMotor();
      liftState = LIFT_AT_UPPER;
    } else if (millis() - liftMoveStartMs > LIFT_TIMEOUT_MS) {
      stopLiftMotor();
      liftState = LIFT_FAULT;
    }
  } else if (liftState == LIFT_MOVING_DOWN) {
    if (lowerPressed) {
      stopLiftMotor();
      liftState = LIFT_AT_LOWER;
    } else if (millis() - liftMoveStartMs > LIFT_TIMEOUT_MS) {
      stopLiftMotor();
      liftState = LIFT_FAULT;
    }
  }
  // LIFT_IDLE / LIFT_AT_UPPER / LIFT_AT_LOWER / LIFT_FAULT: nothing to do
  // until the next "L" command arrives.
}

// Send the latest velocity targets + enable flag to Arduino 2 over I2C.
void i2cSendVelocity(int16_t leftMMs, int16_t rightMMs) {
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(0x10);                       // CMD_SET_VEL
  Wire.write((uint8_t)(leftMMs >> 8));
  Wire.write((uint8_t)(leftMMs & 0xFF));
  Wire.write((uint8_t)(rightMMs >> 8));
  Wire.write((uint8_t)(rightMMs & 0xFF));
  Wire.endTransmission();
}

void i2cSendEnable(bool enabled) {
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(0x30);                       // CMD_SET_ENABLE
  Wire.write(enabled ? 1 : 0);
  Wire.endTransmission();
}

// Pull the encoder delta (and ack) back from Arduino 2.
bool i2cReadEncoders(int32_t &leftTicks, int32_t &rightTicks) {
  uint8_t n = Wire.requestFrom((int)I2C_SLAVE_ADDR, 8);
  if (n < 8) return false;
  uint8_t buf[8];
  for (uint8_t i = 0; i < 8; i++) buf[i] = Wire.read();
  leftTicks  = ((int32_t)buf[0] << 24) | ((int32_t)buf[1] << 16) |
               ((int32_t)buf[2] << 8)  | (int32_t)buf[3];
  rightTicks = ((int32_t)buf[4] << 24) | ((int32_t)buf[5] << 16) |
               ((int32_t)buf[6] << 8)  | (int32_t)buf[7];
  return true;
}

void applyVelocityCommand(int16_t linMMs, int16_t angMradS) {
  // angular.z [mrad/s] * half_track [mm] / 1000 = wheel speed delta [mm/s]
  float delta = ((float)angMradS / 1000.0f) * HALF_TRACK_MM;
  int16_t leftMMs  = (int16_t)(linMMs - delta);
  int16_t rightMMs = (int16_t)(linMMs + delta);
  if (!motorsEnabled) { leftMMs = 0; rightMMs = 0; }
  i2cSendVelocity(leftMMs, rightMMs);
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
      if (sscanf(line + 1, "%d %d", &lin, &ang) == 2) {
        targetLinMMs = (int16_t)lin;
        targetAngMradS = (int16_t)ang;
        lastVelCmdMs = millis();
        applyVelocityCommand(targetLinMMs, targetAngMradS);
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
      if (sscanf(line + 1, "%d", &target) == 1) {
        startLift(target ? 1 : 0);
      }
      break;
    }
    case 'E': {
      int en = 1;
      if (sscanf(line + 1, "%d", &en) == 1) {
        motorsEnabled = (en != 0);
        i2cSendEnable(motorsEnabled);
        if (!motorsEnabled) {
          stopLiftMotor();
          liftState = LIFT_IDLE;
        }
      }
      break;
    }
    default:
      // unknown command: ignore
      break;
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      // overflow: drop the line
      lineLen = 0;
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
  Wire.begin();                 // I2C master
  Wire.setClock(100000);

  pinMode(PIN_LIFT_RPWM, OUTPUT);
  pinMode(PIN_LIFT_LPWM, OUTPUT);
  pinMode(PIN_BTN_UPPER, INPUT_PULLUP);
  pinMode(PIN_BTN_LOWER, INPUT_PULLUP);
  stopLiftMotor();

  pwm.begin();
  pwm.setPWMFreq(PCA9685_FREQ_HZ);
  delay(10);

  lastVelCmdMs = millis();
  lastTelemetryMs = millis();
}

void loop() {
  pollSerial();
  updateLift();

  // Safety watchdog: if the Pi stops sending velocity commands, stop wheels.
  if (millis() - lastVelCmdMs > CMD_WATCHDOG_MS) {
    i2cSendVelocity(0, 0);
    lastVelCmdMs = millis(); // avoid spamming the bus every loop
  }

  if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    sendTelemetry();
  }
}