#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize the PCA9685 PWM driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define servo channels on the PCA9685
const int baseChannel = 0;      
const int shoulderChannel = 1;  
const int elbowChannel = 2;     
const int wristChannel = 3;     
const int gripperChannel = 4; 

int counter = 0;

#define SERVOMIN  0  // Minimum pulse length for the servo
#define SERVOMAX  500  // Maximum pulse length for the servo
#define DELAYTIME 50   // Delay in milliseconds between steps (controls speed)

// Function to map angle to PWM pulse length
int angleToPulse(float angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Function to smoothly move the servo to a target angle
void moveServoSlowly(uint8_t channel, float startAngle, float endAngle, int stepDelay) {
  if (startAngle < endAngle) {
    for (float angle = startAngle; angle <= endAngle; angle += 1) {
      pwm.setPWM(channel, 0, angleToPulse(angle));
      delay(stepDelay);
    }
  } else {
    for (float angle = startAngle; angle >= endAngle; angle -= 1) {
      pwm.setPWM(channel, 0, angleToPulse(angle));
      delay(stepDelay);
    }
  }
}

int receivedNumber = 0; // Variable to store the received number

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud
  pwm.begin();
  pwm.setPWMFreq(50);  // 50Hz servo frequency

  // Initialize servos to their default positions
  //moveServoSlowly(baseChannel, 0, 90, DELAYTIME);
  moveServoSlowly(shoulderChannel, 0, 90, DELAYTIME);
  //moveServoSlowly(elbowChannel, 0, 90, DELAYTIME);
  //moveServoSlowly(wristChannel, 0, 90, DELAYTIME);
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    String receivedData = Serial.readStringUntil('\n'); // Read until newline
    
    receivedNumber = receivedData.toInt(); // Convert the received string to an integer
    
    
    if (counter == 0) {
      moveServoSlowly(baseChannel, 90, receivedNumber, DELAYTIME);
    } 
    if (counter == 1) {
      moveServoSlowly(shoulderChannel, 90, receivedNumber, DELAYTIME);
    } 
    if (counter == 2) {
      moveServoSlowly(elbowChannel, 90, receivedNumber, DELAYTIME);
    } 
    if (counter == 3) {
      moveServoSlowly(wristChannel, 90, receivedNumber, DELAYTIME);
    }
    
    // Print the received number back for confirmation
    Serial.print("Move motor ");
    Serial.print(counter);
    Serial.print(" to ");
    Serial.println(receivedNumber);
    
    counter = counter + 1;
    if (counter > 3) {
      counter = 0;
    }
  }
}
