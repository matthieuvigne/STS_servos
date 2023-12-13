// Basic example of making a STS3215 servo move.
//
// This code will simply make the servo move in circle from (0, 180, 360)deg.

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

  // Reset all servos to position mode: servos have three modes (position, velocity, step position).
  // Position is the default mode so this shouldn't be needed but it's here just to make sure
  // (depending on what you've run before, the servos could be in a different mode)
  servos.setMode(0xFE, STSMode::POSITION); // 0xFE is broadcast address and applies to all servos.
}

void loop()
{
  // Move servo to 0.
  servos.setTargetPosition(SERVO_ID, 0);
  // Wait for servo to start moving, then wait for end of motion
  delay(100);
  while (servos.isMoving(SERVO_ID))
    delay(50);
  // Wait a bit more to see it stop
  delay(500);

  // Move to 180deg.
  servos.setTargetPosition(SERVO_ID, 2048);
  delay(100);
  while (servos.isMoving(SERVO_ID))
    delay(50);
  delay(500);

  // Move to 360deg, at a slower speed (second argument is steps/s).
  servos.setTargetPosition(SERVO_ID, 4095, 500);
  delay(100);
  while (servos.isMoving(SERVO_ID))
    delay(50);
  delay(500);
}
