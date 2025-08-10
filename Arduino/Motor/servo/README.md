# 1. Servo Motors

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

# 2. PWM Servo Motor Control

When a project requires controlling many servo motors simultaneously, using a dedicated PWM (Pulse Width Modulation) servo driver is the best approach. An Arduino can only control a limited number of servos precisely on its own before running into processing and timing issues that cause "jitter" or erratic movement. A dedicated driver offloads this work, allowing for smooth and simultaneous control of many servos.

## Required Hardware

There are two common hardware solutions for this, both of which require an external power supply to handle the high current demand of multiple motors.

* **Option A: PWM/Servo Driver Breakout Board**
    This is a small, separate circuit board (like the purple board shown) that connects to the Arduino using I2C communication pins (SDA and SCL). The servos are then plugged directly into the 3-pin headers on the driver board.

* **Option B: PWM/Servo Shield**
    A shield (like the blue board shown) is a convenient option that plugs directly on top of an Arduino Uno. It provides dedicated pins for connecting many servos and includes a terminal block for connecting the required external power supply.

### External Power is Essential
Both hardware options **must** be powered by an external power source that is separate from the Arduino's USB connection or 5V pin. The servos connect to this external power through the driver board, while the Arduino simply sends low-power control signals. This prevents the Arduino from being overloaded and ensures the servos have enough power to operate correctly.

## Required Library: Adafruit PWM Servo Driver

To communicate with these specialized driver boards from your Arduino, you need to install a specific software library. The standard library used for this hardware is the **Adafruit PWM Servo Driver Library**.

### How to Install the Library:
1.  Open your Arduino IDE.
2.  Navigate to the Library Manager by clicking `Sketch` > `Include Library` > `Manage Libraries...`.
3.  In the search box at the top of the Library Manager window, type `pwm servo driver`.
4.  From the list of results, locate the **Adafruit PWM Servo Driver Library**.
5.  Click the "Install" button to add it to your Arduino IDE.

## General Wiring (for Breakout Board)

While the shield is a plug-and-play solution, the breakout board requires the following wiring:

1.  **I2C Communication (Arduino to Driver):**
    * Connect the Arduino's **SDA** pin to the driver's **SDA** pin.
    * Connect the Arduino's **SCL** pin to the driver's **SCL** pin.
2.  **Logic Power (Arduino to Driver):**
    * Connect the Arduino's **5V** pin to the driver's **VCC** pin (this powers the driver's chip).
    * Connect the Arduino's **GND** pin to the driver's **GND** pin (to create a common ground).
3.  **Motor Power (External Supply to Driver):**
    * Connect the **positive (+)** wire from your external power supply to the driver's **V+** terminal.
    * Connect the **negative (-)** wire from your external power supply to the driver's **GND** terminal.
4.  **Servos to Driver:**
    * Plug each of your servos directly into the 3-pin headers on the driver board.