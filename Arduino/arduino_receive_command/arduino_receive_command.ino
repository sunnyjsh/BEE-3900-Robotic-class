#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize the PCA9685 PWM driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define servo channels on the PCA9685
const int baseChannel = 0;      

#define SERVOMIN  0  // Minimum pulse length for the servo
#define SERVOMAX  500  // Maximum pulse length for the servo
#define DELAYTIME 50   // Delay in milliseconds between steps (controls speed)

int receivedNumber = 0; // Variable to store the received number

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud
  pwm.begin();
  pwm.setPWMFreq(50);  // 50Hz servo frequency
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    String receivedData = Serial.readStringUntil('\n'); // Read until newline
    
    receivedNumber = receivedData.toInt(); // Convert the received string to an integer
    
    moveServoSlowly(baseChannel, 90, receivedNumber, DELAYTIME);
 
    // Print DONE to confirm the completion of the action
    Serial.println("DONE");
    
  }
}

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
