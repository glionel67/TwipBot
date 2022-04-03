# TwipBot

Two Wheeled Inverted Pendulum Robot

## Description

3D printed balance bot project using 2 microcontrollers, one for controlling the motor and the other for balancing control.
The two microcontrollers communicate using a protocol over a serial/UART connexion.\
The Arduino code related to the motor control is in the folder: TwipMotorController.\
The Arduino code related to the balance control is in the folder: TwipBalanceController.

## Usage

TODO

## Roadmap

The balancing controller must be tuned and tested (PID, LQR...).
Adding a remote control of the bot using a Bluetooth link.
Integrating the LCD display.
Improve initialization (IMU biases) and robot state machine.

## Project status

Motor controller tested but not fine tuned.
Balance controller has to be tuned.

## License

This project is licensed under the MIT license.
