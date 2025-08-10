## Motors

This course will cover three main types of motors: DC motors, stepper motors, and servo motors.

### Motor Types

* **DC Motor:**
    * **Pros:** Inexpensive, efficient, simple to control, and provides high torque.
    * **Cons:** Less accurate.
    * **Applications:** Computer fans, drone propellers.
* **Stepper Motor:**
    * **Pros:** Inexpensive and offers precise positioning.
    * **Cons:** Low torque at low speeds and inefficient energy consumption.
    * **Applications:** Moving joints in robotics.
* **Servo Motor:**
    * **Pros:** High torque at high speeds and energy-efficient.
    * **Cons:** Less accurate in positioning and has a limited rotation angle.
    * **Applications:** Opening and closing valves, car or airplane steering.

### Servo Motors

Servo motors have three wires: signal, +5V, and GND. They are controlled by sending a Pulse Width Modulation (PWM) signal to the signal wire.

#### Types of Servos:
* **Closed-loop servos:** Rotate to a specific angle, typically 90 or 180 degrees.
* **Open-loop (continuous) servos:** Can rotate infinitely in either direction.

#### Important Considerations for Servo Motors:
* Before assembly, it's crucial to center your servos by commanding them to the 90-degree position. This prevents damage and ensures the correct range of motion.
* You may need to calibrate the angle values for your specific servos.

### PWM Servo Motor Control

A 16-channel PWM servo driver can be used to control multiple servos. You will need to install the "Adafruit PWM Servo Driver Library" in the Arduino IDE.

## Robotic Arm and Grabbing Mechanisms

This course will involve building and programming a robotic arm with a grabbing mechanism.

### Grabbing Mechanisms

Various grabbing mechanisms can be designed and built, including three-bar and four-bar linkages.

### Claw Grabber

For the provided claw grabber, you will need to determine the full range of motion by starting with a small angular range (e.g., 90 to 100 degrees) and gradually expanding it. Record these values.