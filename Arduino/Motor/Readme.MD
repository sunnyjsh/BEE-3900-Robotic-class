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


## Important Pre-Assembly Step: Centering Servos

Before you physically attach any arms, brackets, or horns to your servo motors, it is critical to command each servo to its neutral, center position (90 degrees).

### Why This is So Important
Following this step is crucial for two main reasons:
* **Prevents Damage:** If you attach a servo horn while the motor is at its 0-degree position and then command it to move 90 degrees, the arm could be forced beyond its physical limits. This can strip the servo's delicate internal gears, permanently damaging the motor.
* **Ensures Correct Range of Motion:** Centering the servos first establishes a proper neutral starting point. This guarantees that your assembled robotic arm will have its full, intended range of movement on both sides of the center position.

### How to Properly Center a Servo
1.  **Connect the Servo:** Wire a single servo to your Arduino. You'll need to connect the power, ground, and signal wires.
2.  **Send the Command:** Upload a simple sketch that commands the servo to move to 90 degrees and hold that position. A basic "servo sweep" example sketch can be modified for this; just have it write a constant 90-degree value.
3.  **Assemble:** Once the servo is holding its center position, you can securely attach the horn or bracket, ensuring it is aligned correctly for your project (e.g., U-shaped brackets would be assembled into a straight line at this point).


### PWM Servo Motor Control

A 16-channel PWM servo driver can be used to control multiple servos. You will need to install the "Adafruit PWM Servo Driver Library" in the Arduino IDE.

## Robotic Arm and Grabbing Mechanisms

This course will involve building and programming a robotic arm with a grabbing mechanism.

### Grabbing Mechanisms

Various grabbing mechanisms can be designed and built, including three-bar and four-bar linkages.

### Claw Grabber

For the provided claw grabber, you will need to determine the full range of motion by starting with a small angular range (e.g., 90 to 100 degrees) and gradually expanding it. Record these values.

