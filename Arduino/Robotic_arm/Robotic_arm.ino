/* Arduino with multiple servos to control a robotic arm */

#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

int pos1 = 90;  

void setup()
{
  servo1.attach(6);
  servo2.attach(9);
  servo3.attach(10);
  servo4.attach(11);
  }

void loop()
{
  
  for (pos1 = 90; pos1 <= 160; pos1 += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo1.write(pos1);              // tell servo to go to position in variable 'pos'
    servo2.write(pos1);              // tell servo to go to position in variable 'pos'
    servo3.write(pos1);              // tell servo to go to position in variable 'pos'
    servo4.write(pos1);              // tell servo to go to position in variable 'pos'
    delay(50);                       // waits 15ms for the servo to reach the position
  }
  for (pos1 = 160; pos1 >= 90; pos1 -= 1) { // goes from 180 degrees to 0 degrees
    servo1.write(pos1);              // tell servo to go to position in variable 'pos'
    servo2.write(pos1);              // tell servo to go to position in variable 'pos'
    servo3.write(pos1);              // tell servo to go to position in variable 'pos'
    servo4.write(pos1);              // tell servo to go to position in variable 'pos'
   delay(50);                       // waits 15ms for the servo to reach the position
  }

  delay(1000);
}
