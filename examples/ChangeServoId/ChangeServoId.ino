// Change the Id of a servo

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
  else
  {
    // Try to change servo 1 ID to 2 - lit led if this fails (servo 1 does not exist or servo 2 already exists)
    if (!servos.setId(1, 2))
        digitalWrite(13, HIGH);
  }
}

void loop()
{
  // Nothing to do.
}