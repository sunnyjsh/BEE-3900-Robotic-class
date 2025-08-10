# 2. Servo Motors

This guide provides an introduction to servo motors, including their types, how they work, and how to connect them to an Arduino.

## Understanding Servo Motors

A servo motor is a self-contained electrical device that rotates parts of a machine with high precision. Internally, a servo consists of a DC motor, a gear system, a position sensor (like a potentiometer), and an embedded control board.

### Servo Wiring

Servos typically have a three-wire connector for power and control. The color-coding is generally consistent:

* **Signal or PWM (Orange wire):** This wire receives the control signal from the microcontroller.
* **+5V or +ve supply (Red wire):** This provides the positive power supply for the motor.
* **GND or -ve supply (Brown wire):** This is the ground or negative connection.

## How Servos Are Controlled

Servo motors are controlled by sending a series of pulses through the signal wire, a technique known as Pulse Width Modulation (PWM). The width of the electrical pulse determines the angular position of the servo's output shaft.

The standard for most hobby servos is:
* **0 degrees:** A 1 millisecond (ms) pulse.
* **90 degrees:** A 1.5 ms pulse.
* **180 degrees:** A 2 ms pulse.

These pulses are typically sent every 20 ms.

## Types of Servos

There are two primary types of servo motors:

* **Closed-Loop (Standard) Servo:**
    * This type rotates to a specific angle (e.g., 90 to 180 degrees) and holds that position firmly.
    * Most standard servos can rotate about 180 degrees, while some high-torque models may be limited to a smaller range, such as 120 degrees.
* **Open-Loop (Continuous) Servo:**
    * This type is modified to offer continuous, infinite rotation instead of moving to a set position.
    * The PWM signal controls the speed and direction of rotation rather than a specific angle.

## Servo Specifications

Servos are often rated by their torque, commonly measured in kilogram-centimeters (kg-cm). For example, you might see servos rated at 20, 25, or 35 kg-cm. However, it is noted that these values may not always be reliable.

## Example Wiring to an Arduino Uno

Connecting a servo to an Arduino is straightforward. The following example uses digital pin 9, which is one of the PWM-capable pins on the Uno.

1.  **Ground Connection:** Connect the servo's **GND** wire (usually brown) to one of the `GND` pins on the Arduino.
2.  **Power Connection:** Connect the servo's **+5V** wire (usually red) to the `5V` pin on the Arduino.
3.  **Signal Connection:** Connect the servo's **Signal** wire (usually orange) to a digital PWM pin on the Arduino, such as `pin 9`.