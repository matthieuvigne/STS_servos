// Operate the STS3215 servo in velocity mode
//
// This code makes the motor spin continuously at a given velocity

#include "STSServoDriver.h"

STSServoDriver servos;

// ID of the servo currently being tested.
byte SERVO_ID = 1;

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  // Since the serial port is taken by the servo, we can't easily send debug messages, so
  // we use the on-board led instead.
  // Try to connect with the servos, using pin 2 as direction pin and the default (only) serial
  // interface of an Arduino Uno.
  if (!servos.init(2))
  {
    // Failed to get a ping reply, turn on the led.
    digitalWrite(13, HIGH);
  }
  // Set the servo to velocity mode.
  servos.setMode(SERVO_ID, STSMode::VELOCITY);
}

void loop()
{
  // Spin clockwise at 500steps/s, i.e one turn every 8 seconds
  servos.setTargetVelocity(SERVO_ID, 500);
  delay(4000);
  // Spin counter-clockwise at 2048steps/s, i.e one turn every 2 seconds
  servos.setTargetVelocity(SERVO_ID, -2048);
  delay(2000);
}
