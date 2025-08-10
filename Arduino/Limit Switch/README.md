# Limit Switches

A limit switch is an electromechanical device used to detect the presence or passage of an object. In robotics and automation, they are essential for detecting when a mechanical part has reached the end of its travel, preventing collisions, or simply acting as a physical sensor.

## Common Types of Limit Switches

Limit switches come in various forms, distinguished by the type of actuator used to trigger the switch. The choice of switch depends on the specific application.

* **Whisker Limit Switch:** Features a long, flexible wire or spring as its actuator. It's excellent for detecting objects from multiple directions without precise alignment.
* **Roller Limit Switch:** Has a small wheel on the actuator. This is useful for applications where an object slides past the switch, as the roller reduces friction and wear.
* **Roller-Lever Limit Switch:** Combines a lever arm with a roller at the tip. The lever provides mechanical advantage, allowing the switch to be triggered with less force or by objects further away.
* **Lever Limit Switch:** Uses a simple, flat lever arm as the actuator.
* **Plunger Limit Switch:** Activated by a small button or plunger that is pressed directly. This type is ideal for applications requiring precise position detection.

## Switch Terminals and Operation

Most micro limit switches have three terminals, allowing for flexible wiring configurations:

* **COMMON (C or COM):** This is the common terminal that the other two terminals connect to.
* **Normally Open (NO):** In its default, resting state, this terminal is **not connected** to the COMMON terminal. When the switch is pressed (activated), the NO terminal connects to the COMMON terminal, completing the circuit.
* **Normally Closed (NC):** In its default, resting state, this terminal **is connected** to the COMMON terminal. When the switch is pressed, the NC terminal disconnects from the COMMON terminal, breaking the circuit.

## Example Wiring to an Arduino

A limit switch can be easily read by an Arduino digital input pin. The most common setup uses the **Normally Open (NO)** configuration and the Arduino's internal pull-up resistor.

### Wiring Steps (NO Configuration):
1.  Connect the **COMMON (C)** terminal of the limit switch to a **GND** pin on the Arduino.
2.  Connect the **Normally Open (NO)** terminal of the switch to any digital input pin on the Arduino (e.g., pin 2).
3.  The **NC** terminal is left unconnected for this setup.

### Required Arduino Code

To make this wiring work correctly, you must enable the internal pull-up resistor on the input pin within your `setup()` function. This prevents the pin from "floating" and giving unreliable readings.

```cpp
const int limitSwitchPin = 2; // The pin the switch is connected to

void setup() {
  Serial.begin(9600); // Initialize serial communication to see the output
  
  // Set the pin as an input and enable the internal pull-up resistor
  pinMode(limitSwitchPin, INPUT_PULLUP); 
}

void loop() {
  // Read the state of the switch
  int switchState = digitalRead(limitSwitchPin);

  // The logic is inverted due to the pull-up resistor:
  // HIGH means not pressed, LOW means pressed.
  if (switchState == LOW) {
    Serial.println("Switch is pressed!");
  } else {
    Serial.println("Switch is not pressed.");
  }

  delay(100); // Small delay to prevent spamming the serial monitor
}