# Distance Sensors

This guide provides an overview of the distance sensor used in this course, focusing on its operation, specifications, and how to connect it to an Arduino.

## Time-of-Flight (ToF) Technology

While there are many types of distance sensors, this course will focus on a **Time-of-Flight (ToF)** sensor. These sensors work by emitting a pulse of light (typically an invisible laser) and measuring the time it takes for that light to reflect off an object and return to the sensor. By knowing the speed of light, the sensor can calculate the distance to the object with very high precision. This technology is generally more accurate and has a longer range than traditional infrared (IR) or ultrasonic sensors.

## Featured Sensor: VL53L1X

The specific sensor we will be using is the **VL53L1X**. This is a powerful, long-range ToF sensor that is ideal for robotics applications.

### Key Specifications
* **Maximum Range:** Up to 4 meters (400 cm)
* **Resolution:** 1 mm
* **Minimum Distance:** 4 cm
* **Sampling Frequency:** Up to 50 Hz (can take 50 readings per second)
* **Field of View (FoV):** 15° to 27°

### Required Library
To use the VL53L1X sensor with an Arduino, you must install the correct library.

1.  Open the Arduino IDE.
2.  Go to the Library Manager (`Sketch` > `Include Library` > `Manage Libraries...`).
3.  In the search bar, type `VL53L1X`.
4.  Find the library named **"VL53L1X by Pololu"** and click "Install".

### Wiring Instructions
The VL53L1X sensor communicates with the Arduino using the I2C protocol, which uses two main wires: SDA (Data) and SCL (Clock).

**Pin Connections:**
* `VCC`: Connect to the `5V` pin on the Arduino.
* `GND`: Connect to the `GND` pin on the Arduino.
* `SDA`: Connect to the Arduino's SDA pin.
* `SCL`: Connect to the Arduino's SCL pin.
* `SHUT` and `INT`: These pins are for more advanced functions and can typically be left unconnected for basic distance readings.

#### Arduino Uno Wiring
* `SDA` -> `A4`
* `SCL` -> `A5`

#### Arduino Mega Wiring (Important Note)
The I2C pins are in a different location on an Arduino Mega.
* `SDA` -> `Pin 20`
* `SCL` -> `Pin 21`