# 6. Buzzer

This guide explains how to use a buzzer with an Arduino to create sounds, from simple beeps to complex melodies.

## Understanding the Buzzer

A buzzer is an audio output device that produces sound. There are two main types: active buzzers, which produce a tone when power is applied, and passive buzzers, which require a changing signal (like a PWM wave) to produce different tones. For playing melodies, a passive buzzer is typically used.

[cite_start]You can identify the pins on a standard cylindrical buzzer by their leg length[cite: 1]:
* [cite_start]**Longer Leg:** This is the positive (+ve) terminal. [cite: 1]
* [cite_start]**Shorter Leg:** This is the negative (-ve) terminal. [cite: 1]

## Basic Wiring

[cite_start]To connect a buzzer to an Arduino, follow these steps[cite: 1]:

1.  Place the buzzer onto a breadboard.
2.  [cite_start]Connect the buzzer's **negative (-ve) leg** to a `GND` pin on the Arduino. [cite: 1]
3.  Connect the buzzer's **positive (+ve) leg** to a digital output pin on the Arduino. [cite_start]The example diagram uses `pin 8`. [cite: 1]

## Simple Beep Code

The simplest way to make a sound is to turn the digital pin on and off rapidly. This causes the buzzer's internal diaphragm to vibrate, creating a buzzing sound.

```cpp
// This sketch will produce a simple, continuous buzzing sound.

void setup() {
  // Set the buzzer pin (8) as an output
  pinMode(8, OUTPUT);
}

void loop() {
  // Turn the pin on (sends voltage to the buzzer)
  digitalWrite(8, HIGH);
  delay(500); // Wait for half a second
  
  // Turn the pin off (stops voltage to the buzzer)
  digitalWrite(8, LOW);
  delay(500); // Wait for half a second
}