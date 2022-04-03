# TwipBot

Two Wheeled Inverted Pendulum Robot

## Introduction

3D printed balance bot project using 2 microcontrollers, one for controlling the motor and the other for balancing control.
The two microcontrollers communicate using a protocol over a serial/UART connexion.
The Arduino code related to the motor control is in the folder: TwipMotorController.
The Arduino code related to the balance control is in the folder: TwipBalanceController.

## Bill of material

- 2x Pololu gearmotor HPCB 6V 75:1 with extended shaft (ref #3074)
- Magnetic encoder pair kit 12 CPR (ref #3081)
- Pololu Micro Metal Gearmotor Bracket Extended Pair (ref #1089)
- Pololu Wheel 90×10mm Pair - Blue (ref #1438)
- Pololu Dual MC33926 Motor Driver Shield for Arduino (ref #2503)
- 1x ACS724 Current Sensor Carrier -5A to +5A (ref #4041)
- 1x Arduino Leonardo
- 1x Arduino proto shield
- 1x Arduino Pro Micro 5V
- 1x Invensense MPU9250
- 1x battery holder
- 6x LR6 1.2V battery
- 1x LCD 16x2 display
- 1x switch button
- 3D printed parts

## Status

The balancing controller must be tuned and tested.

## License

MIT license.
