# BEE 3900 Robotic Class Guide

This document provides a comprehensive overview of the materials and procedures for the BEE 3900 Robotic class, as presented by Professor Sunny Jung of Cornell University.

## Introduction

This course will utilize the Arduino platform for various robotics projects. You can find the course materials and code examples on the class GitHub page.

### Software and Contact Information
* **Professor:** Sunny Jung
* **Email:** sj737@cornell.edu
* **Arduino Software:** Download from [https://www.arduino.cc/en/Main/Software](https://www.arduino.cc/en/Main/Software)
* **GitHub Repository:** [https://github.com/sunnyjsh/BEE-3900-Robotic-class](https://github.com/sunnyjsh/BEE-3900-Robotic-class)

### Boards and Components

This course will utilize various boards and electronic components. Here is a brief overview of some of the hardware:

* **Arduino Uno Rev3:** A popular microcontroller board.
* **Arduino Mega 2560 Rev3:** A larger board with more I/O pins and memory for more complex projects.
* **Raspberry Pi 4 Model B:** A single-board computer that can be used for more advanced robotics applications.

### Safety Precautions
* The maximum current for the Arduino is 150 mA. Ensure you use appropriate resistors in your circuits.
* Never place the Arduino board on a conductive surface.
* Always connect the battery last after you have double-checked all wiring connections.
* Be cautious of short circuits or physical damage to batteries, as this can cause a fire.
* Be careful with your fingertips when working with wire connectors.
* Do not force a stalled motor to run, as this can cause it to overheat and potentially burn out.

## Wiring and Connections

Proper wiring is crucial for all projects. Here are some of the common connection methods and components you will use:

### Breadboard

* The five horizontal holes in a row are electrically connected.
* The vertical columns on the sides are connected and are typically used for power and ground.

### Soldering

For permanent connections, you may need to solder components. A video tutorial on soldering Arduino header pins can be found here: [https://www.youtube.com/watch?v=37mW1i_oEpA](https://www.youtube.com/watch?v=37mW1i_oEpA).

### Wire Connectors

A variety of connectors will be used to create circuits:
* **Jumper Wires:** Used to connect components on a breadboard. They come in male and female varieties.
* **Lever Connectors:** A simple way to connect wires without soldering.
* **Solder Seal Wire Connectors:** These connectors have a ring of solder inside that melts when heated, creating a secure connection.
* **Screw Terminals:** Allow for secure wire connections using a screw to clamp the wire.
* **Power Plugs:** Various types of power plugs will be used, including XT60, Deans/T-plug, and EC3. The standard DC power plug for the Arduino is 5.5mm.

### Identifying Wires

* In DC circuits, the red wire is typically positive (+).
* The black wire is typically negative (-) or ground (GND).
* For wires of the same color, the ribbed wire is usually the negative wire.

## Working with the Arduino IDE

The Arduino Integrated Development Environment (IDE) is used to write and upload code to the Arduino board.

### Setting up the IDE
1.  Connect the Arduino to your computer via USB.
2.  In the Arduino IDE, select the correct board and port from the "Tools" menu or the dropdown menu.
3.  Write your code in the sketch editor.
4.  Verify and upload your code to the Arduino board.
5.  You can use the Serial Monitor to view data sent from the Arduino.

### Installing Libraries

You may need to install libraries to control certain components. For example, the "FastLED" library is used for controlling addressable LEDs.