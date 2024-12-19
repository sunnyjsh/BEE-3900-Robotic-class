
void setup() {
  // Start the keyboard emulation
  Serial.begin(9600);
  delay(1000); // Wait for the setup to complete
}

void loop() {
  // Check if any key is pressed
  if (Serial.available()) {
    char key = Serial.read(); // Read the key from the serial input
    if (key == 't') {
      // If the pressed key is 't', print this to the serial monitor
      Serial.println("t");
    }
  }
  delay(100); // Small delay to prevent too many prints in a short time
}
