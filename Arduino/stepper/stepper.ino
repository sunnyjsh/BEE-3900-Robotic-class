const int stepPin = 2; 
const int dirPin = 5; 
const int enPin = 8;
const int stepDelay = 1000;

void setup() {
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
  digitalWrite(enPin,LOW);
  
}

void loop() {
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  for(int x = 0; x < 800; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(stepDelay); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(stepDelay); 
  }
  delay(1000); // One second delay
  digitalWrite(dirPin,LOW); //Changes the direction of rotation
  for(int x = 0; x < 800; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(stepDelay);
  }
  delay(1000); 
}
