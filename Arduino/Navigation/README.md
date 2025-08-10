# Guide to Mecanum Wheels and Stepper Motor Control

This guide covers the principles of Mecanum wheels and how to control the necessary stepper motors using an Arduino and a CNC Shield.

## 1. Mecanum Wheels

Mecanum wheels are a special type of wheel designed for omnidirectional movement. This means a vehicle using these wheels can move in any direction (forward, backward, sideways, diagonally) without needing to turn first.

### Design
Each wheel has a series of rollers mounted at a 45-degree angle around its circumference. It's the angle and free-spinning nature of these rollers that allows for complex and precise movements. Mecanum wheels come in many sizes, from a few inches to over two feet in diameter, to suit different vehicle sizes and load requirements.

### Holonomic Movement
By controlling the speed and direction of each of the four wheels independently, a wide range of motions can be achieved.

* **Straight Ahead/Backward:** All four wheels rotate in the same direction (either forward or backward). The side rollers are not engaged, and the wheel acts like a conventional one.
* **Sideways (Strafe):** To move sideways to the right, the wheels on the front-left and back-right spin forward, while the wheels on the front-right and back-left spin backward. This engages the 45-degree rollers, pushing the entire vehicle sideways. The opposite rotation is used to move left.
* **Diagonal:** To move diagonally, two adjacent wheels (e.g., front-left and front-right) are kept still while the other two rotate, or all four wheels are rotated at different speeds.
* **On-the-Spot Rotation (Turn Round):** To rotate the vehicle in place, the wheels on the left side are spun in the opposite direction of the wheels on the right side. For example, to turn clockwise, the left wheels spin forward and the right wheels spin backward.
* **Arc Turn / Cornering:** The vehicle can perform a turn while moving forward or backward by spinning the wheels on one side faster than the wheels on the other.

According to the slides, the two most useful motions for many applications are the ability to move **straight/sideways** and to **turn on the spot**.

---

## 2. Stepper Motor Control Using a CNC Shield

To precisely control the four motors required for a Mecanum wheel platform, an Arduino combined with a CNC Shield is a very effective solution. This setup simplifies the wiring and control of multiple stepper motors.

### The CNC Shield
A CNC (Computer Numerical Control) shield is an Arduino add-on board that provides dedicated sockets for up to four stepper motor driver modules (e.g., A4988 or DRV8825). It breaks out the necessary control pins from the Arduino and provides terminals for connecting an external motor power supply.

### Arduino Pin Connections for the CNC Shield
The shield automatically maps the Arduino's digital pins to the control pins for each motor driver socket. The standard pinout is as follows:

* **X-Axis Motor:**
    * `X.STEP`: Arduino Pin 2
    * `X.DIR`: Arduino Pin 5
* **Y-Axis Motor:**
    * `Y.STEP`: Arduino Pin 3
    * `Y.DIR`: Arduino Pin 6
* **Z-Axis Motor:**
    * `Z.STEP`: Arduino Pin 4
    * `Z.DIR`: Arduino Pin 7
* **A-Axis Motor (4th Channel):**
    * `A.STEP`: Arduino Pin 12
    * `A.DIR`: Arduino Pin 13

### Activating the Fourth Channel (A-Axis)

The CNC shield provides an optional fourth channel, labeled "A". You can either control it independently using its dedicated pins (12 and 13) or "clone" the movement of another axis onto it.

**How to Clone an Axis:**
The shield has sets of jumper pins that allow you to slave the A-axis to the X, Y, or Z axis. If you want the A-axis motor to perform the exact same movements as the X-axis motor, you would use jumpers to connect the A-axis control pins to the X-axis control pins. This is useful in machines where two motors need to work in perfect unison.