// Basic example of making a STS3215 servos move synchronously.
// This code will simply make the two servos move from (180, 360)deg.

#include "STSServoDriver.h"

STSServoDriver servos;

byte ids[] = {1, 2};
int positions[2];
int speeds[] = {2400, 2400};

void setup()
{
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
    // Move to 180deg.
    positions[0] = 2048;
    positions[1] = 2048;
    servos.setTargetPositions(2, ids, positions, speeds);
    // Wait for servo to start moving, then wait for end of motion
    delay(100);
    while (servos.isMoving(1))
        delay(50);
    // Wait a bit more to see it stop
    delay(500);

    // Move to 360deg.
    positions[0] = 4095;
    positions[1] = 4095;
    servos.setTargetPositions(2, ids, positions, speeds);
    delay(100);
    while (servos.isMoving(1))
        delay(50);
    delay(500);
}
