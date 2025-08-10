# Controlling Multiple Servos for a Robotic Arm

This guide covers the essential steps for setting up, wiring, and programming multiple servo motors, which is a common requirement for building a robotic arm.


## Wiring Multiple Servos

When using multiple servos, you cannot power them directly from the Arduino's 5V pin. The combined current draw will be too high for the Arduino's onboard voltage regulator, leading to unpredictable behavior or damage. You must use a separate, external power supply.

The diagram shows five servos connected to an Arduino Uno and an external 5.00V power supply via a breadboard.

### Wiring Steps:
1.  **Power Rails:** Connect the external 5V power supply to the power (+) and ground (-) rails of your breadboard.
2.  **Servo Power:** Connect the power (red) and ground (brown/black) wires of **all** servos to the corresponding power and ground rails on the breadboard.
3.  **Common Ground:** Connect a wire from the Arduino's `GND` pin to the ground (-) rail of the breadboard. This creates a "common ground," which is essential for the signal to be interpreted correctly.
4.  **Signal Wires:** Connect the signal wire (orange/yellow) of each servo to a separate digital PWM pin on the Arduino (e.g., pins 6, 9, 10, 11).

## Arduino Code for Multiple Servos

The Arduino `Servo.h` library makes it easy to control multiple servos. The code below shows an example of controlling four servos attached to pins 6, 9, 10, and 11, moving them in unison.
