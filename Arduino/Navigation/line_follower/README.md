# Line Follower Robots

This guide explains the principles, hardware, and programming required to build a line follower robot using an Arduino and an infrared (IR) sensor module.

## How Line Follower Sensors Work

Line follower robots navigate by detecting a line on a contrasting surface (e.g., a black line on a white background). This is achieved using an IR sensor module.

### The Sensor Pair
Each sensor unit on the module consists of two main components:
1.  **An Infrared (IR) LED:** This LED emits a beam of infrared light, which is invisible to the human eye.
2.  **A Photodiode:** This component acts as an IR light detector. It measures the amount of IR light that is reflected back to it.

### Detection Principle
The sensor works based on the principle of light reflectance:
* **White Surface:** A white or light-colored surface has high reflectance. It bounces most of the IR light from the LED back to the photodiode. This generates a **LOW** digital signal (logic "0").
* **Black Surface:** A black or dark-colored surface has low reflectance. It absorbs most of the IR light, so very little is reflected back to the photodiode. This generates a **HIGH** digital signal (logic "1").

**Key takeaway:** When the sensor is over the black line, its output is HIGH (1). When it's over the white surface, its output is LOW (0).

### Sensor Calibration
If you are not getting good detection, you can use the small blue potentiometer (knob) on the sensor module to adjust its sensitivity. Turning this knob changes the threshold for what the sensor considers a "HIGH" or "LOW" reading, allowing you to fine-tune its performance for different lighting conditions and surfaces.

## Line Following Logic

The robot stays on the line by constantly correcting its course based on feedback from the sensors. This is often implemented using an "error" value to determine the robot's position relative to the line.

* **Error 0 (On Track):** When the center sensor is on the black line, the robot is on course. The logic tells the robot to continue moving straight ahead.
* **Error -1 (Veered Left):** If the robot drifts to the right, the left sensor will eventually move over the black line. This indicates an error to the left, and the robot must turn left to correct its course.
* **Error 1 (Veered Right):** If the robot drifts to the left, the right sensor will eventually move over the black line. This indicates an error to the right, and the robot must turn right to correct its course.

By continuously checking for these error states and making small corrections, the robot appears to smoothly follow the line.

## Wiring and Code

The example code for the line follower can be found in the `line_follower` directory on the class GitHub.

### Wiring the Sensor Module
The line tracking module should be connected to the Arduino as follows. This wiring corresponds to the pins used in the example code.

* **VCC:** Connect to the `5V` pin on the Arduino.
* **GND:** Connect to the `GND` pin on the Arduino.
* **Left Sensor (L):** Connect to digital `pin 2`.
* **Middle Sensor (M):** Connect to digital `pin 4`.
* **Right Sensor (R):** Connect to digital `pin 10`.

### Example Arduino Code Snippet

This code provides the basic logic for following a line.

```cpp
// Line Tracking IO define
// These lines create easy-to-read names for the sensor states.
// The '!' inverts the reading, so a HIGH signal (on the line) becomes 'true'.
#define LT_R !digitalRead(10)
#define LT_M !digitalRead(4)
#define LT_L !digitalRead(2)


void loop() {
  // Check if the middle sensor is on the line
  if (LT_M) {
    forward(); // If so, move forward
  } 
  // If the middle is not on the line, check the right sensor
  else if (LT_R) {
    right(); // If the right sensor is on the line, turn right
  } 
  // If neither middle nor right are on the line, check the left
  else if (LT_L) {
    left(); // If the left sensor is on the line, turn left
  }
  // If no sensors detect the line, the robot will do nothing.
  // More advanced code could handle this case (e.g., stop or search).
}

// NOTE: The functions forward(), right(), and left() would need to be
// defined elsewhere in your code to control the robot's motors.