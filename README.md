# Arduino STS_Servos driver

A library to drive [Feetech's STS smart servos](https://www.google.com/search?client=firefox-b-d&q=feetech+sts3215) - in particular the STS 3215.
This low-cost servo offers very interesting properties at a quite cheap price (approx. $15 at the time of writing):
 - Stall torque 19kg.cm
 - Running speed 50rpm @ 7V.
 - 360° servo mode - plus multiturn and step mode
 - Open and close loop velocity commands
 - Position, current and temperature sensors

This library was made mostly with simple, "Hello World" applications in mind. Thus, its high-level API only provides access to
the main, basic functions. However, all commands and registers are exposed, thus more complexe applications can be easily
written from it.
