// Sweep example to drive a STS3215 servo with an interface board such as
// FE-URT-1.
//
// This code will simply make the servo move in circle from (0, 180, 360)deg.
// Requirements:
// - ESP32 board including M5StackCore2 & M5Atom
// - Serial Interface board. FE-URT-1 is recommended.
// - STS servo. STS3215 is recommended
//
// Connections:
// | M5Atom  |  --   |  FE-URT-1  |
// | :-----: | :---: | :--------: |
// |   5V    |  --   |     5V     |
// |   GND   |  --   |    GND     |
// | 32 (RX) |  --   | RXD (Silk) |
// | 26 (TX) |  --   | TXD (Silk) |
//
// Caution !!!
// This sketch cannot work with Arduino UNO R3 because it has only single serial
// port. This sketch needs two ports; one is for serial with PC, the another is
// for servo control The UNO board is not suitable for debugging servo control.

#include <Arduino.h>

#include "STSServoDriver.h"

// define serial pins for servo control
#if defined(ARDUINO_M5Stack_ATOM)
#define RXD 32
#define TXD 26

#elif defined(ARDUINO_XIAO_ESP32C3)
#define RXD 7
#define TXD 6
#endif

STSServoDriver servos;

// ID of the servo currently being tested.
const byte SERVO_ID = 1;

void setup() {
  // An ESP32 module communicates with an interface board using a UART port
  // (Serial port using TX & RX pins)
  // "Serial" is used for communication with a PC (a.k.a Serial Monitor)
  // "Serial1" is for servo control

  Serial.begin(115200);  // serial for Serial Monitor
  Serial1.begin(1000000, SERIAL_8N1, RXD, TXD);
  delay(1000);  // waiting for connection

  if (Serial1) {
    Serial.printf("Serial1(%d,%d) is ready\n\r", RXD, TXD);
  } else {
    Serial.printf("Serial1(%d,%d) was not found\n\r", RXD, TXD);
  }

  if (servos.init(&Serial1)) {
    Serial.printf("Servos are ready\n\r");
  }

  // Reset all servos to position mode: servos have three modes (position,
  // velocity, step position). Position is the default mode so this shouldn't be
  // needed but it's here just to make sure (depending on what you've run
  // before, the servos could be in a different mode)
  servos.setMode(0xFE, STSMode::POSITION);  // 0xFE is broadcast address and
                                            // applies to all servos.
}

void loop() {
  if (!servos.ping(SERVO_ID)) {
    Serial.printf("servo-%02d response nothing\n\r", SERVO_ID);
  }

  // Move servo to 0.
  servos.setTargetPosition(SERVO_ID, 0);
  // Wait for servo to start moving, then wait for end of motion
  delay(100);
  while (servos.isMoving(SERVO_ID)) {
    delay(50);
  }
  // Wait a bit more to see it stop
  delay(500);

  // Move to 180deg.
  servos.setTargetPosition(SERVO_ID, 2048);
  delay(100);
  while (servos.isMoving(SERVO_ID)) {
    delay(50);
  }
  delay(500);

  // Move to 360deg, at a slower speed (second argument is steps/s).
  servos.setTargetPosition(SERVO_ID, 4095, 500);
  delay(100);
  while (servos.isMoving(SERVO_ID)) {
    delay(50);
  }
  delay(500);
}
