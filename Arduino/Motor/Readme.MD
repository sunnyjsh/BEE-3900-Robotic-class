## Motors

[cite_start]This course will cover three main types of motors: DC motors, stepper motors, and servo motors[cite: 66, 67].

### Motor Types

* **DC Motor:**
    * [cite_start]**Pros:** Inexpensive, efficient, simple to control, and provides high torque[cite: 58].
    * [cite_start]**Cons:** Less accurate[cite: 58].
    * [cite_start]**Applications:** Computer fans, drone propellers[cite: 59].
* **Stepper Motor:**
    * [cite_start]**Pros:** Inexpensive and offers precise positioning[cite: 61].
    * [cite_start]**Cons:** Low torque at low speeds and inefficient energy consumption[cite: 61].
    * [cite_start]**Applications:** Moving joints in robotics[cite: 62].
* **Servo Motor:**
    * [cite_start]**Pros:** High torque at high speeds and energy-efficient[cite: 64].
    * [cite_start]**Cons:** Less accurate in positioning and has a limited rotation angle[cite: 64].
    * [cite_start]**Applications:** Opening and closing valves, car or airplane steering[cite: 65].

### Servo Motors

[cite_start]Servo motors have three wires: signal, +5V, and GND[cite: 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78]. [cite_start]They are controlled by sending a Pulse Width Modulation (PWM) signal to the signal wire[cite: 68].

#### Types of Servos:
* [cite_start]**Closed-loop servos:** Rotate to a specific angle, typically 90 or 180 degrees[cite: 80, 81].
* [cite_start]**Open-loop (continuous) servos:** Can rotate infinitely in either direction[cite: 82, 83, 88].

#### Important Considerations for Servo Motors:
* [cite_start]Before assembly, it's crucial to center your servos by commanding them to the 90-degree position[cite: 131, 132]. [cite_start]This prevents damage and ensures the correct range of motion[cite: 134, 135].
* [cite_start]You may need to calibrate the angle values for your specific servos[cite: 144].

### PWM Servo Motor Control

[cite_start]A 16-channel PWM servo driver can be used to control multiple servos[cite: 141]. [cite_start]You will need to install the "Adafruit PWM Servo Driver Library" in the Arduino IDE[cite: 142].
