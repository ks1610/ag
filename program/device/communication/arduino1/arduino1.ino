/*
 * arduino1_i2c_test.ino
 * ---------------------------------------------------------------------
 * Minimal I2C test — MASTER side.
 *
 * Every 1 second:
 *   1. Sends the ASCII bytes "hello" to Arduino 2 (slave address 0x08).
 *   2. Requests up to 16 bytes back and prints whatever it got.
 *
 * Open Tools -> Serial Monitor at 115200 baud, line ending "Newline",
 * to watch this happen.
 *
 * Wiring: Arduino1 A4(SDA) <-> Arduino2 A4(SDA)
 *         Arduino1 A5(SCL) <-> Arduino2 A5(SCL)
 *         Arduino1 GND     <-> Arduino2 GND   (MUST be connected)
 * If you don't already have 4.7k pull-up resistors from SDA->5V and
 * SCL->5V, add them — many boards don't include them on-board.
 * ---------------------------------------------------------------------
 */

#include <Wire.h>

static const uint8_t SLAVE_ADDR = 0x08;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }  // wait for USB serial on boards that need it
  Wire.begin();           // join I2C bus as master
  Serial.println("Arduino 1 (MASTER) ready.");
}

void loop() {
  // ---- 1. Send "hello" to Arduino 2 ----
  Serial.println("Sending: hello");
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write("hello");
  uint8_t result = Wire.endTransmission();

  if (result != 0) {
    Serial.print("!! I2C write failed, error code: ");
    Serial.println(result);
    // Error codes: 1=data too long, 2=NACK on address (slave not
    // responding/wrong address/no wiring), 3=NACK on data, 4=other.
  }

  delay(50); // tiny gap so Arduino 2 has time to prepare its reply

  // ---- 2. Ask Arduino 2 for a reply ----
  uint8_t n = Wire.requestFrom(SLAVE_ADDR, (uint8_t)16);
  if (n == 0) {
    Serial.println("!! No reply received (requestFrom got 0 bytes).");
  } else {
    char buf[17];
    uint8_t i = 0;
    while (Wire.available() && i < sizeof(buf) - 1) {
      buf[i++] = (char)Wire.read();
    }
    buf[i] = '\0';
    Serial.print("Received reply: ");
    Serial.println(buf);
  }

  Serial.println("------");
  delay(950);
}