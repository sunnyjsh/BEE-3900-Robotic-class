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

// Define all of our pin #'s
const byte enablePin = 8;

const int StepX = 2;
const int DirX = 5;
const int StepY = 3;
const int DirY = 6;
const int StepZ = 4;
const int DirZ = 7;
const int StepA = 12;
const int DirA = 13;


const int detPin = 16;

int Step = 0;

enum ChassisDirection {
  FORWARD,BACKWARD,LEFT,RIGHT
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

  //designate which function to perform
  // performComp2V2(); //blind (without distance sensor) navigation V2, calibrated for ASABE competition
  // performComp2(); //blind navigation V1, calibrated for Cornell Lab arena
  // performComp3(); //navigation with distance sensor

  //testWheels2(); //test steppers

  ////Read from distance sensor for debug
  //  while (true) {
  //    readDistanceSensor();
  //  }
}

void loop() {
  // move wheels one-by-one, in the order X, Y, A, Z, to help identify motor connections
  // testWheels(); 

  // move the chassis forward, then backward, repetitively
  testWheels2();

  // move the servo motor to angles 60, 90, 120 degrees
  testServo();
}

////Functions////

//Distance sensor debug
void checkForUserStartSignal() {
  while (true) {
    int average_distance = getDistanceSensorAverage();
    int distance = readDistanceSensor();
    if (distance - average_distance < -300) { break; }
  }
  delay(1000);

  // Make sure buffer has non-jank values before exit
  for(int i = 0; i<NUM_DISTANCES_TO_STORE; i++) {
     readDistanceSensor();
  }
}

//Navigation using distance sensor
void performComp3() {
  bool do_start = true;
  bool do_middle = false;
  bool do_end = true;
  
  if (do_start) {
    // Move close to first plant
    moveChassis(FORWARD,355,2000); 
    delay(500);
    turnChassis(RIGHT,12,2000);
    delay(500);
    moveChassis(FORWARD,155,2000); // 255

    // Do row code
    for(int x = 0; x<12; x++) { 
      // Move to plant using distance sensor
      moveToNextPlant();
      delay(500);

      //Move forward until the distance sensor does not see the block
      movePastBlock();

      //Plant-to-Plant adjustments in case of arena irregularities
      if (x == 0){
      } else if (x == 1){
      } else if (x == 2){
      } else if (x == 3){
      } else if (x == 4){
      } else if (x == 5){
      } else if (x == 6){
      } else if (x == 7){
      } else if (x == 8){
      } else if (x == 9){
        //moveChassis(FORWARD,40,2000);
      } else if (x == 10){
        //moveChassis(FORWARD,40,2000);
      } else if (x == 11){
        //moveChassis(FORWARD,40,2000);
      }
      

      // Take picture of plant
      takePicture(2400);
     
      // Move forward to align arm w plant
      moveChassis(FORWARD,40,2000);
      
      // Do not continue until a signal from the Robo-Arm Arduino is recieved
      waitForArm();
      
      // Turn into wall
      if (x  != 9 and x != 10) {
        moveChassis(RIGHT,50,2000); 
      } else {
        moveChassis(RIGHT,50,2000);
      }
      delay(500);

      // Move out from wall
      //moveChassis(LEFT,40,1000);

      // Move forward close to next plant
      moveChassis(FORWARD,150,2000);
      delay(500);      
    }
  }

  // Move to the other wall
  //  Move away from wall a little
  moveChassis(FORWARD,200,2000);
  moveChassis(LEFT,350,2000);
  delay(500);
  //  Turn around
  turnChassis(LEFT,575,2000);
  delay(500);
  //  Move to last wall
  moveChassis(RIGHT,1250,2000); // 950
  delay(500);

  if (do_middle) {
  }

  if (do_end) {
    // Align with backwall for precision's sake
    moveChassis(BACKWARD,350,2000); // 350
    delay(500);

    // Move close to first plant
    moveChassis(FORWARD,355,2000); 
    delay(500);
    turnChassis(RIGHT,20,2000);
    delay(500);
    moveChassis(FORWARD,155,2000); // 255

    // Do row code  
    for(int x = 0; x<12; x++) { 
      // Move to plant using distance sensor
      moveToNextPlant();
      moveChassis(FORWARD,100,2000);

      // Take picture of plant
      takePicture(2400);
      
      // Move forward to align w plant
      moveChassis(FORWARD,40,2000);
      waitForArm();
        
      // Turn into wall
      if (x  != 9 and x != 10) {
        moveChassis(RIGHT,50,2000); 
      } else {
        moveChassis(RIGHT,50,2000);
      }
      delay(500);
      
      // Move forward close to next plant
      moveChassis(FORWARD,150,2000);
      delay(500);     
    }
  }
}

//Blind navigation for ASABE arena
void performComp2V2() {
  bool do_start = true;
  bool do_end = true;


  int arm_distance = 30;
  int to_first_adjuster_right = 0;//arm_distance + 0;
  int to_first_adjuster_left = 0;//arm_distance + 0;
  int to_next_adjuster_right = arm_distance + 0;
  int to_next_adjuster_left = arm_distance + 0;
  int backup_adjuster = -100;
  bool do_arm = true;


  if (do_start) {
    // Move to the first plant
    moveChassis(FORWARD,270,2000); // 684 664 // 355 to 300 to 280
    delay(100);
    turnChassis(RIGHT,12,2000);
    delay(100);
    moveChassis(FORWARD,355 - to_first_adjuster_right,2000); // 684 664
  
    // Move through each plant, stopping a ways after the last one for clearance
    for(int x = 0; x<12; x++) { 
      delay(100);

      takePicture(2400); //send signal to pi to take picture
      if (do_arm) {
           moveChassis(FORWARD,arm_distance,2000); //Move forward to center arm on plant stem
           waitForArm(); //wait for pi signal from Robo-Arm Arduino
      }
  
      delay(100);

      ////Make adjustments for arena imperfections
      // if (x  != 9 and x != 10) {
      //   turnChassis(RIGHT,12,2000); 
      // } else {
      //   turnChassis(RIGHT,24,2000);
      // }
  
      // if (x  != 9) {
      //   moveChassis(FORWARD,322 - to_next_adjuster_right,2000); // 326 320 322*
      // } else {
      //   moveChassis(FORWARD,337 - to_next_adjuster_right,2000);
      // }

      // Move to next plant location
      moveChassis(FORWARD,292,2000);
      //moveChassis(LEFT,12,2000);

      if (x == 4) {
        moveChassis(LEFT,30,2000);
        delay(500);
        moveChassis(FORWARD,30,2000);
      }
    }
  
    // Move to the other wall
    delay(500);
    moveChassis(FORWARD,600,2000);
    //  Move away from wall a little
    delay(500);
    moveChassis(LEFT,350,2000);
    delay(100);
    //  Turn around
    turnChassis(LEFT,550,2000);
    delay(100);
    //  Move to last wall
    moveChassis(RIGHT,1250,2000); // 950
    delay(100);
  }
  
  if (do_end) {
    // Align with backwall for precision's sake
    moveChassis(BACKWARD,600 + backup_adjuster,2000); // 350
    moveChassis(RIGHT,100,2000); // 950
    delay(100);
    
    // Move to first plant
    moveChassis(FORWARD,322,2000); // 257
    delay(100);
    moveChassis(RIGHT,40,2000); // 257
    //turnChassis(RIGHT,24,2000);
    delay(100);
    moveChassis(FORWARD,302 - to_first_adjuster_left,2000); //
  
    // Move through each plant, stopping a ways after the last one for clearance
    for(int x = 0; x<12; x++) { 
      delay(100);

      if (x == 1 or x == 9) {
        moveChassis(LEFT,24,2000);
      } else {
         turnChassis(RIGHT,12,2000); 
      }

      delay(100);

      takePicture(2400);
      if (do_arm) {
           moveChassis(FORWARD,arm_distance,2000);
           waitForArm();
      }
      
      if (x == 0){
        moveChassis(FORWARD,312 - to_next_adjuster_left,2000); // 332
      } else if (x == 1) {
        moveChassis(FORWARD,322- to_next_adjuster_left,2000); //
        delay(100);
        moveChassis(RIGHT,24,2000);
      } else if (x == 10) {
         moveChassis(FORWARD,322- to_next_adjuster_left,2000); //
      } else {
        moveChassis(FORWARD,322- to_next_adjuster_left,2000); //
      }
      
    }

    delay(100);
     moveChassis(FORWARD,380,2000); //
  }  
}


//Blind navigation for arena at Cornell
void performComp2V1() {

  // Go down the 1st row, taking photos at each plant
  for(int i = 0; i<12; i++) {
    handleNextPlant();

    if (i == 8 or i == 4) {
      delay(100);
      moveChassis(RIGHT,100,2000);
    }

  }


  // Move to the other wall
  // Move out from the plant
  moveChassis(FORWARD,300,2000);
  //  Move away from wall a little
  moveChassis(LEFT,350,2000);
  //  Turn around
  turnChassis(LEFT,575,2000);
  //  Move to last wall
  moveChassis(RIGHT,900,2000);

  
  // Go down the 2nd row, taking photos at each plant
//  for(int i = 0; i<12; i++) {
//    handleNextPlant();
//  }
  
}

//Check pin for signal from robotic arm
void waitForArm() {
  while (analogRead(detPin) < 500) {
    delay(10);
  }
}

//move forward and take a picture at next plant
void handleNextPlant(){
  moveToNextPlant();
  takePicture(2400);
}

//Serial communications with the raspberry pi to tell it to take a picture
void takePicture(int to_wait) {
  Serial.println("9");
  delay(to_wait); // 5000
}

//Move to next plant using the distance sensor
void moveToNextPlant() {
  while (true) {
    // Move forward a step
    moveChassis(FORWARD, 10,2000);
    
    // Check if we've reached the dowel, stop if so
    int average_distance = getDistanceSensorAverage();
    int distance = readDistanceSensor();
    if (distance - average_distance > 300) { break; }
  }

  // Make sure buffer has non-jank values before exit
  for(int i = 0; i<NUM_DISTANCES_TO_STORE; i++) {
     readDistanceSensor();
  }
}

//Move chassis just beyond plant using the distance sensor, necessary for proper alignment
void movePastBlock() {
  const int wall_dis = 240;
  const int wall_dis_allowance = 10; // 15 435
  const int change_threshold = 20; // 100 300
  
  while (true) {
    // Move forward a step
    moveChassis(FORWARD,5,2000);
    
    // Check if we've reached the dowel, stop if so
    int average_distance = getDistanceSensorAverage();
    int distance = readDistanceSensor();
    //Serial.println(distance);
    if (distance - average_distance > change_threshold) {
      if (abs(distance - wall_dis) <= wall_dis_allowance) {
        break;
      }
    }
  }

  // Make sure buffer has non-jank values before exit
  for(int i = 0; i<NUM_DISTANCES_TO_STORE; i++) {
     readDistanceSensor();
  }
}


// Set up distance sensor
void prepDistanceSensor() {
  Wire.begin();
  distanceSensor.setTimeout(500);
  if (!distanceSensor.init())
  {
    //Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  for(int i = 0; i<NUM_DISTANCES_TO_STORE; i++) { 
    previous_distances[i] = 0;
  }
  distanceSensor.startContinuous();
  for(int i = 0; i<NUM_DISTANCES_TO_STORE; i++) { 
    previous_distances[i] = readDistanceSensor();
  }
}

//Read distance sensor values
int readDistanceSensor() {
  if (distanceSensor.timeoutOccurred()) { //Serial.print(" TIMEOUT");
    }
  int dis = distanceSensor.readRangeContinuousMillimeters();
  for (int i = NUM_DISTANCES_TO_STORE-1; i>0;i--) {
    previous_distances[i] = previous_distances[i-1];
  }
  previous_distances[0] = dis;
  int avg = getDistanceSensorAverage();
  //Serial.print(dis);
  //Serial.print(" ");
  //Serial.println(avg);
  //Serial.println(dis);
  return dis;
}

//Average distance sensor values
int getDistanceSensorAverage() {
  float average = 0;
  for (int i = NUM_DISTANCES_TO_STORE-1; i>=0;i--) {
    average = average + previous_distances[i];
  }
  average = average / ((float) NUM_DISTANCES_TO_STORE);
  return (int) average;
}

//////Stepper motor controls//////

void turnChassis(ChassisDirection dir, int steps, int half_period) {
  setChassisTurnDirection(dir);

  moveWheels(steps, half_period);
}

void moveChassis(ChassisDirection dir, int steps, int half_period) {
  setChassisMovementDirection(dir);
    
  moveWheels(steps, half_period);
}

void moveWheels(int steps, int half_period) {
  // 200 steps is a full WHEEL rotation
  // ~575 steps for CHASSIS 180 degrees
  for(int x = 0; x<steps; x++) { 
    // Set all of our wheels to go for a step
    digitalWrite(StepX,HIGH);
    digitalWrite(StepY,HIGH);
    digitalWrite(StepZ,HIGH);
    digitalWrite(StepA,HIGH);
    // Give the motors some time to do their thing
    delayMicroseconds(half_period);
    // Tell the motors to stop attempting the step forward
    digitalWrite(StepY,LOW); 
    digitalWrite(StepX,LOW); 
    digitalWrite(StepZ,LOW); 
    digitalWrite(StepA,LOW); 
    // Again give them some time to do so
    delayMicroseconds(half_period);
  }
}

void setChassisMovementDirection(ChassisDirection dir) {
  switch (dir) {
  case FORWARD:
    digitalWrite(DirX, 0); 
    digitalWrite(DirY, 0);
    digitalWrite(DirZ, 0);
    digitalWrite(DirA, 0);
    break;
  case BACKWARD:
    digitalWrite(DirX, 1); 
    digitalWrite(DirY, 1);
    digitalWrite(DirZ, 1);
    digitalWrite(DirA, 1);
    break;
  case LEFT:
    digitalWrite(DirX, 1); 
    digitalWrite(DirY, 1);
    digitalWrite(DirZ, 0);
    digitalWrite(DirA, 0);
    break;
  case RIGHT:
    digitalWrite(DirX, 0); 
    digitalWrite(DirY, 0);
    digitalWrite(DirZ, 1);
    digitalWrite(DirA, 1);
    break;
  default:
    break;
  }
}

void setChassisTurnDirection(ChassisDirection dir) {
  switch (dir) {
  case LEFT:
    digitalWrite(DirX, 0); 
    digitalWrite(DirY, 1);
    digitalWrite(DirZ, 1);
    digitalWrite(DirA, 0);
    break;
  case RIGHT:
    digitalWrite(DirX, 1); 
    digitalWrite(DirY, 0);
    digitalWrite(DirZ, 0);
    digitalWrite(DirA, 1);
    break;
  default:
    break;
  }
}

void testWheels() {
  // This shows me which wheel is which (X,Y,A,Z) without having to
  //  manually trace what wire from which motor goes to which part of the board

  moveWheel(200, DirX, 0, StepX);
  delay(500);
  moveWheel(200, DirY, 0, StepY);
  delay(500);
  moveWheel(200, DirA, 0, StepA);
  delay(500);
  moveWheel(200, DirZ, 0, StepZ);
  delay(2000);
}

void moveWheel(int Step, int DirPin, bool DirState, int StepPin) {
  // This moves a singular wheel; useful for testing. See moveForward for comments
  digitalWrite(DirPin, DirState);
  for(int x = 0; x<Step; x++) { 
    digitalWrite(StepPin,HIGH);
    delayMicroseconds(1000);
    digitalWrite(StepPin,LOW); 
    delayMicroseconds(1000);
 }
}

bool isMoving = false; // Flag to keep track if the stepper is moving

void testWheels2() {
  // This test illustrate the switch button. 

  // Read the button state
  bool buttonState = !digitalRead(BTN_PIN); // Invert the button state, as we're using a Normally Open button

  if (true) {
    // Button has been pressed, move the Z motor to the desired position
    //while (true) {
      // test moves
      moveChassis(FORWARD,3000,300);
      delay(500);
      moveChassis(BACKWARD,3000,300);
      delay(500);
      // moveChassis(LEFT,3000,300);
      // delay(500);
      // moveChassis(RIGHT,3000,300);
      // delay(500);
      // // //  Test turns
      // turnChassis(LEFT,3000,600);
      // delay(500);
      // turnChassis(RIGHT,3000,600);
      // delay(500);
    //}
  }

  // if (!buttonState) {
  //   // Button has been released, stop the Z motor
  //   isMoving = false; 
  //   // moveChassis(LEFT,0,0); // Stop the motor by stepping 0 steps
  // }

  // If the stepper is still moving, continue to step
  // if (isMoving) {
  //   testWheels2();
  // }
  Serial.println(buttonState);
}

void testServo() {
  // Move servo
  myServo.write(60);
  delay(1000); // Wait for 1 second

  // Move the servo to 90 degrees
  myServo.write(90);
  delay(1000); // Wait for 1 second

  // Move the servo to 180 degrees
  myServo.write(120);
  delay(1000); // Wait for 1 second
}