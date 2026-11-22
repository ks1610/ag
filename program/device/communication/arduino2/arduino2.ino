/*
 * arduino2_i2c_test.ino
 * ---------------------------------------------------------------------
 * Minimal I2C test — SLAVE side, address 0x08.
 *
 * Whenever Arduino 1 sends data, this prints what it received.
 * Whenever Arduino 1 requests data, this replies with "hi".
 *
 * Open Tools -> Serial Monitor at 115200 baud (on Arduino 2's own USB
 * connection) to watch this happen too — useful to confirm both sides
 * see the same transaction.
 *
 * Wiring: same as arduino1_i2c_test.ino (shared SDA/SCL/GND).
 * ---------------------------------------------------------------------
 */

#include <Wire.h>

static const uint8_t MY_ADDR = 0x08;

void onReceiveHandler(int numBytes) {
  char buf[32];
  uint8_t i = 0;
  while (Wire.available() && i < sizeof(buf) - 1) {
    buf[i++] = (char)Wire.read();
  }
  buf[i] = '\0';
  Serial.print("Received from master: ");
  Serial.println(buf);
}

void onRequestHandler() {
  Wire.write("hi");
  Serial.println("Sent reply: hi");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Wire.begin(MY_ADDR);           // join I2C bus as slave at address 0x08
  Wire.onReceive(onReceiveHandler);
  Wire.onRequest(onRequestHandler);
  Serial.println("Arduino 2 (SLAVE, addr 0x08) ready.");
}

void loop() {
  // nothing to do here — everything happens in the I2C callbacks above
}