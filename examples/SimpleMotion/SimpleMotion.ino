// Basic example of making a STS3215 servo move.
//
// This code will simply make the servo move in circle from (0, 180, 360)deg.

#include "STSServoDriver.h"

STSServoDriver servos;

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
}

void loop()
{
  // Move servo to 0.
  servos.setTargetPosition(1, 0);
  // Wait for servo to start moving, then wait for end of motion
  delay(100);
  while (servos.isMoving(1))
    delay(50);
  // Wait a bit more to see it stop
  delay(500);

  // Move to 180deg.
  servos.setTargetPosition(1, 2048);
  delay(100);
  while (servos.isMoving(1))
    delay(50);
  delay(500);

  // Move to 360deg.
  servos.setTargetPosition(1, 4095);
  delay(100);
  while (servos.isMoving(1))
    delay(50);
  delay(500);
}
