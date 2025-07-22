// This code controls chassis movement and navigation

// Include the stuff we need for the distance sensor
#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

VL53L0X distanceSensor;
Servo myServo; // Create a servo object

#define NUM_DISTANCES_TO_STORE 9
#define BTN_PIN 11

int previous_distances[NUM_DISTANCES_TO_STORE];
int currentMoveDirection;

// ===== Motion constants =====
const float STEPS_PER_MM = 9.67;
const float STEPS_PER_DEGREE = 10;
const float TURN_CORRECTION_K = 0.98930;

// Define all of our pin #'s
const byte enablePin = 8;
const int StepX = 2, DirX = 5;
const int StepY = 3, DirY = 6;
const int StepZ = 4, DirZ = 7;
const int StepA = 12, DirA = 13;
const int detPin = 16;

int Step = 0;

enum ChassisDirection {
  FORWARD,BACKWARD,LEFT,RIGHT,TLEFT,TRIGHT
};


//function, based operation... call function once in setup() to perform the task
void setup() {

  // Setup serial for debug
  Serial.begin(9600);

  // Set up pins
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);

  pinMode(detPin,INPUT);
  digitalWrite(detPin,HIGH);

  pinMode(StepX,OUTPUT);
  pinMode(DirX,OUTPUT);
  pinMode(StepY,OUTPUT);
  pinMode(DirY,OUTPUT);
  pinMode(StepZ,OUTPUT);
  pinMode( DirZ,OUTPUT);
  pinMode(StepA,OUTPUT);
  pinMode( DirA,OUTPUT);
  pinMode( BTN_PIN, INPUT_PULLUP);

  myServo.attach(10); // Attach the servo to pin 10

  delay(1000);

  Serial.println("Starting");
}

void loop() {
  // First number is about distance in mm;
  // Second number is the speed; smaller number is fast.
    moveChassisDir(FORWARD,  200, 300); delay(1000);
    moveChassisDir(BACKWARD,  200, 500); delay(1000);
    moveChassisDir(LEFT,  200, 300); delay(1000);
    moveChassisDir(RIGHT,  200, 500); delay(1000);
    moveChassisDir(TLEFT,  200, 300); delay(1000);
    moveChassisDir(TRIGHT,  200, 500); delay(1000);
    
}

//Navigation with acceleration
void moveChassisDir(int dir, float mm, int half_period_target) {
  // 1. Start time of recording
  unsigned long t0 = millis();

  // 2. Compute total number of steps
  int steps = round(mm * STEPS_PER_MM);
  Serial.print("Moving dir="); Serial.print(dir);
  Serial.print("  mm="); Serial.print(mm);
  Serial.print(" -> steps="); Serial.println(steps);

  // 3. Number of steps for acceleration/deceleration
  int accel_steps = max((int)(steps * 0.05), 10);
  int decel_start = steps - accel_steps;
  int start_period = half_period_target * 2;
  int current_period;

  // 4. Set direction pin
  setChassisMovementDirection(dir);

  // 5. Main loop: pulse output + bump detection
  for (int i = 0; i < steps; i++) {
    // —— Acceleration phase ——  
    if (i < accel_steps) {
      current_period = start_period - (start_period - half_period_target) * i / accel_steps;
    }
    // —— Deceleration phase ——  
    else if (i >= decel_start) {
      int j = i - decel_start;
      current_period = half_period_target + (start_period - half_period_target) * j / accel_steps;
    }
    // —— Constant speed phase ——  
    else {
      current_period = half_period_target;
    }
    // —— Send pulse ——  
    digitalWrite(StepX, HIGH); digitalWrite(StepY, HIGH);
    digitalWrite(StepZ, HIGH); digitalWrite(StepA, HIGH);
    delayMicroseconds(current_period);
    digitalWrite(StepX, LOW);  digitalWrite(StepY, LOW);
    digitalWrite(StepZ, LOW);  digitalWrite(StepA, LOW);
    delayMicroseconds(current_period);

  }

  // 6. After completing all steps normally, print elapsed time and completion flag
  unsigned long dt = millis() - t0;
  Serial.print("TIME_MS:"); Serial.println(dt);
  Serial.println("DONE");
}

void setChassisMovementDirection(int dir) {
  // Store current direction for odometry
  currentMoveDirection = dir;
  // left-front Z
  // left-rear X
  // right-front Y
  // right-rear A
  switch (dir) {
    case FORWARD:
      digitalWrite(DirX, HIGH); digitalWrite(DirY, LOW);
      digitalWrite(DirZ, HIGH); digitalWrite(DirA, LOW); break;
    case BACKWARD:
      digitalWrite(DirX, LOW); digitalWrite(DirY, HIGH);
      digitalWrite(DirZ, LOW); digitalWrite(DirA, HIGH); break;
    case LEFT:
      digitalWrite(DirX, LOW); digitalWrite(DirY, HIGH);
      digitalWrite(DirZ, HIGH); digitalWrite(DirA, LOW); break;
    case RIGHT:
      digitalWrite(DirX, HIGH); digitalWrite(DirY, LOW);
      digitalWrite(DirZ, LOW); digitalWrite(DirA, HIGH); break;
    case TLEFT:
      digitalWrite(DirX, HIGH); digitalWrite(DirY, HIGH);
      digitalWrite(DirZ, LOW); digitalWrite(DirA, LOW); break;
    case TRIGHT:
      digitalWrite(DirX, LOW); digitalWrite(DirY, LOW);
      digitalWrite(DirZ, HIGH); digitalWrite(DirA, HIGH); break;
  }
}

