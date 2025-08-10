# LED Control with Arduino

This guide provides instructions on how to set up and control addressable LEDs using an Arduino for the BEE-3900 Robotic class.

## Required Library: FastLED

To control the addressable LED strips and rings, you must first install the `FastLED` library in your Arduino IDE. This library provides the necessary functions to easily manage individual LEDs, create animations, and set colors.

### Installation Steps:
1.  Open the Arduino IDE.
2.  Navigate to the Library Manager by going to `Sketch` > `Include Library` > `Manage Libraries...`.
3.  In the Library Manager window, type "FastLED" into the search bar.
4.  Find the "FastLED" library by Daniel Garcia in the search results.
5.  Click the "Install" button to add the library to your IDE.

## Code Example

Example sketches and code for controlling LEDs can be found in the `LED_example` directory on the class GitHub repository.

## Wiring Instructions

Proper wiring is essential for the LEDs to function correctly. A critical point for all addressable LEDs is to connect your Arduino to the **INPUT** side of the strip or ring. The **OUTPUT** side is used to chain multiple LED components together.

### LED Ring Wiring

For the circular LED ring, connect the three wires as follows:
* **Red Wire:** Connect to the `5V` pin on the Arduino.
* **Green Wire (Signal):** Connect to digital `Pin 9` on the Arduino.
* **Black Wire (Ground):** Connect to the `GND` pin on the Arduino.

### LED Strip Wiring (amomi.com Glow)

For the straight "Glow" LED strip, use the pins marked "INPUT" and connect them as follows:
* **5v Pin:** Connect to the `5V` pin on the Arduino.
* **S Pin (Signal):** Connect to digital `Pin 9` on the Arduino.
* **GND Pin:** Connect to the `GND` pin on the Arduino.