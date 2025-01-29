# Arduino STS_Servos driver

[![arduino-library-badge](https://www.ardu-badge.com/badge/STS_Servos.svg?)](https://www.ardu-badge.com/STS_Servos)
![ci status badge](https://github.com/matthieuvigne/STS_Servos/actions/workflows/platformio-ci.yml/badge.svg)

<https://github.com/user-attachments/assets/4ff4030c-b8c4-4994-ae51-3bceac055187>

A library to drive [Feetech's STS smart servos](https://www.feetechrc.com/74v-19-kgcm-plastic-case-metal-tooth-magnetic-code-double-axis-ttl-series-steering-gear.html) - in particular the STS 3215.
This low-cost servo offers very interesting properties at a quite cheap price (approx. $20 at 2024-10-05):

- Stall torque 19kg.cm
- Running speed 50rpm @ 7V.
- 360° servo mode - plus multiturn and step mode
- Open and close loop velocity commands
- Position, current and temperature sensors

This library was made mostly with simple, "Hello World" applications in mind. Thus, its high-level API only provides access to
the main, basic functions. However, all commands and registers are exposed, thus more complex applications can be easily
written from it.

## Example

To debug STS servo control, official interface board, [FE-URT-1](https://www.feetechrc.com/FE-URT1-C001.html), is essential. You can change Servo ID with this board.

The I/F board has pins to communicate with a micro-controller. The following table shows sample connections between ESP32 and the I/F. [SimpleSweepWithInterfaceBoard.ino](./examples/SimpleSweepWithInterfaceBoard/SimpleSweepWithInterfaceBoard.ino) is a sample sketch to drive a STS servo. The video in this README is results of this sketch.

| ESP32   |  --   |  FE-URT-1  |
| :-----: | :---: | :--------: |
|   5V    |  --   |     5V     |
|   GND   |  --   |    GND     |
| 32 (RX) |  --   | RXD (Silk) |
| 26 (TX) |  --   | TXD (Silk) |
