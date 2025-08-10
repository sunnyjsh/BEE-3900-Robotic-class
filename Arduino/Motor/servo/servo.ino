#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize the PCA9685 PWM driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150  // Minimum pulse length for the servo
#define SERVOMAX  600  // Maximum pulse length for the servo
#define DELAYTIME 50   // Delay in milliseconds between steps (controls speed)

// Define servo channels on the PCA9685
const int baseChannel = 0;      
const int shoulderChannel = 1;  
const int elbowChannel = 2;     
const int wristChannel = 3;     
const int gripperChannel = 4;   

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

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);  // 50Hz servo frequency

  // Initialize servos to their default positions
  moveServoSlowly(baseChannel, 0, 90, DELAYTIME);
  moveServoSlowly(shoulderChannel, 0, 90, DELAYTIME);
  moveServoSlowly(elbowChannel, 0, 90, DELAYTIME);
//  moveServoSlowly(wristChannel, 0, 90, DELAYTIME);
//  moveServoSlowly(gripperChannel, 0, 0, DELAYTIME);
}

void loop() {  
  // Move base to 45 degrees
  moveServoSlowly(baseChannel, 90, 45, DELAYTIME);
  delay(1000);

  // Move shoulder to 120 degrees
  moveServoSlowly(shoulderChannel, 90, 120, DELAYTIME);
  delay(1000);

  // Move elbow to 60 degrees
  moveServoSlowly(elbowChannel, 90, 60, DELAYTIME);
  delay(1000);

  // Move wrist to 30 degrees
  moveServoSlowly(wristChannel, 90, 30, DELAYTIME);
  delay(1000);

  // Open gripper
  moveServoSlowly(gripperChannel, 0, 90, DELAYTIME);
  delay(1000);

  // Return all servos to initial positions
  moveServoSlowly(baseChannel, 45, 90, DELAYTIME);
  moveServoSlowly(shoulderChannel, 120, 90, DELAYTIME);
  moveServoSlowly(elbowChannel, 60, 90, DELAYTIME);
//  moveServoSlowly(wristChannel, 30, 90, DELAYTIME);
//  moveServoSlowly(gripperChannel, 90, 0, DELAYTIME);
  delay(2000);
}
