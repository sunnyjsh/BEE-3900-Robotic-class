# BEE 3900 Robotic Class Guide

[cite_start]This document provides a comprehensive overview of the materials and procedures for the BEE 3900 Robotic class, as presented by Professor Sunny Jung of Cornell University[cite: 2].

## Introduction

[cite_start]This course will utilize the Arduino platform for various robotics projects[cite: 1]. [cite_start]You can find the course materials and code examples on the class GitHub page[cite: 6].

### Software and Contact Information
* [cite_start]**Professor:** Sunny Jung [cite: 2]
* [cite_start]**Email:** sj737@cornell.edu [cite: 4]
* [cite_start]**Arduino Software:** Download from [https://www.arduino.cc/en/Main/Software](https://www.arduino.cc/en/Main/Software) [cite: 5]
* [cite_start]**GitHub Repository:** [https://github.com/sunnyjsh/BEE-3900-Robotic-class](https://github.com/sunnyjsh/BEE-3900-Robotic-class) [cite: 6]

### Boards and Components

[cite_start]This course will utilize various boards and electronic components[cite: 13]. Here is a brief overview of some of the hardware:

* [cite_start]**Arduino Uno Rev3:** A popular microcontroller board[cite: 13].
* [cite_start]**Arduino Mega 2560 Rev3:** A larger board with more I/O pins and memory for more complex projects[cite: 13].
* [cite_start]**Raspberry Pi 4 Model B:** A single-board computer that can be used for more advanced robotics applications[cite: 13].

### Safety Precautions
* The maximum current for the Arduino is 150 mA. [cite_start]Ensure you use appropriate resistors in your circuits[cite: 9].
* [cite_start]Never place the Arduino board on a conductive surface[cite: 10].
* [cite_start]Always connect the battery last after you have double-checked all wiring connections[cite: 30].
* [cite_start]Be cautious of short circuits or physical damage to batteries, as this can cause a fire[cite: 36].
* [cite_start]Be careful with your fingertips when working with wire connectors[cite: 23].
* [cite_start]Do not force a stalled motor to run, as this can cause it to overheat and potentially burn out[cite: 175].

## Wiring and Connections

Proper wiring is crucial for all projects. Here are some of the common connection methods and components you will use:

### Breadboard

* [cite_start]The five horizontal holes in a row are electrically connected[cite: 14, 15].
* [cite_start]The vertical columns on the sides are connected and are typically used for power and ground[cite: 14, 16].

### Soldering

[cite_start]For permanent connections, you may need to solder components[cite: 17]. [cite_start]A video tutorial on soldering Arduino header pins can be found here: [https://www.youtube.com/watch?v=37mW1i_oEpA](https://www.youtube.com/watch?v=37mW1i_oEpA)[cite: 19].

### Wire Connectors

A variety of connectors will be used to create circuits:
* **Jumper Wires:** Used to connect components on a breadboard. [cite_start]They come in male and female varieties[cite: 20, 21].
* [cite_start]**Lever Connectors:** A simple way to connect wires without soldering[cite: 22].
* **Solder Seal Wire Connectors:** These connectors have a ring of solder inside that melts when heated, creating a secure connection.
* [cite_start]**Screw Terminals:** Allow for secure wire connections using a screw to clamp the wire[cite: 25, 26].
* [cite_start]**Power Plugs:** Various types of power plugs will be used, including XT60, Deans/T-plug, and EC3[cite: 33]. [cite_start]The standard DC power plug for the Arduino is 5.5mm[cite: 34].

### Identifying Wires

* [cite_start]In DC circuits, the red wire is typically positive (+)[cite: 28].
* [cite_start]The black wire is typically negative (-) or ground (GND)[cite: 29].
* [cite_start]For wires of the same color, the ribbed wire is usually the negative wire[cite: 31, 32].

## Working with the Arduino IDE

The Arduino Integrated Development Environment (IDE) is used to write and upload code to the Arduino board.

### Setting up the IDE
1.  [cite_start]Connect the Arduino to your computer via USB[cite: 45].
2.  [cite_start]In the Arduino IDE, select the correct board and port from the "Tools" menu or the dropdown menu[cite: 44, 46].
3.  [cite_start]Write your code in the sketch editor[cite: 46].
4.  [cite_start]Verify and upload your code to the Arduino board[cite: 46].
5.  [cite_start]You can use the Serial Monitor to view data sent from the Arduino[cite: 47, 48].

### Installing Libraries

You may need to install libraries to control certain components. [cite_start]For example, the "FastLED" library is used for controlling addressable LEDs[cite: 49].
