/*
  Code written by Shaun Armstrong for the amomii Glow.

  This code is for a digital clock. An RTC (DS1307RTC) is used to keep the time, and
  the time is displayed on a panal made up of four 7-segment digits. Each segment
  is lit by an amomii Glow strip. Basically, the code works setting all of the LEDs a
  colour and then by changing leds in certain segments to black before turning the
  LEDs on. The segments that need to be on or off are stored in the digits.h file
  as bytes written in binary. The LMB of each byte is 0 as only 7 segments are needed
  to display a digit. A '1' means the segment in that position must be turned on, and a
  '0' means turned off.

  Coins are used as capacitive sensors as input devices for setting the color of the
  and for setting a countdown alarm, and sounds are achieved with a passive buzzer.

  For a more clear understanding of what the code does and how to set up the clock,
  visit https://amomii.com/pages/projects-hacks and look for the Glow Clock project
*/

//include libraries

#include <FastLED.h>
#include "digits.h"
#include <TimeLib.h>
#include <Wire.h>
#include <DS1307RTC.h>  // a basic DS1307 library that returns time as a time_t

#define NUM_LEDS 224 //8 pixels per segment, 7 segments per digit, 4 digits = 224
#define DATA_PIN 10 //glow attached to pin 10

uint8_t hue; // 0 - 255;

// Define the array of leds
CRGB leds[NUM_LEDS];

//coin wire connection pins
int topCoin = 8;
int bottomCoin = 9;

//used for setting the default color
int hue1 = 255;
int hue2 = 123;
int hue3 = 200;
int hue4 = 49;

//use to track if the countdown timer has been set or now
bool countDown = false;

//buzzer connected to pin 12
int buzz = 12;

void setup() {
  //initialize glowstrips ready to use
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  //set max brightness (0 - 255)
  FastLED.setBrightness(100);
  //initialize serial communication for debugging
  Serial.begin(115200);

  pinMode(buzz, OUTPUT);//buzzer pin is an output

  while (!Serial) ; // wait until Arduino Serial Monitor opens
  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");
  //wait one sec before proceeding
  delay(1000);

  showTime();// call the show time function

  //call the randomSeed function to make the random functions used in the code more random
  randomSeed(analogRead(A2));
}


/*
   This function is used to read the coins being used a cap sensors. This
   function returns 'false' if no press is detected or 'ture' if one is
*/
long checkCap(int c) {
  unsigned long D = 0;//used for counting how long the while loop runs
  /*
     Set the pin used as a function parameter as an INPUT_PULLUP effectively
     charging the capacitor created between the coin and the finger touching it
  */
  pinMode(c, INPUT_PULLUP);
  /*
     set the pinMode back to INPUT and write it low. Now any charge present at the
     pin can make its way to ground via the input pin
  */
  pinMode(c, INPUT);
  digitalWrite(c, LOW);

  /*
     trap the code in the while loop as long as the digitalRead function returns
     'false'. Each time the while loop iterates, add 1 to D. If D gets greater than 100,
     it is likely there is no charge to be recorded, so set D to 0 and 'break' from the loop.
     However, if a reading is detected within 100 iterations of the while loop, it is likely
     that this reading came from a charge collected in the make-shit capacitor created between
     the coin and the finger.
  */
  while (digitalRead(c) == 1) {
    D++;
    if (D > 100) {
      D = 0;
      break;
    }
  }

  /*
     If D = 0, this function will return 'false', but any positive value will make this
     function return 'true' (boolean functions can only return 'true' of 'false')
  */
  return (D);

}

/*
   This function set the number to display and the color of a given digit. It
   accepts 5 parameters: d = digit (0-3), n = the number to display (0-9),
   h = hue value (0-255), s = sturation value (0-255), and b = brightness(0-255).
*/
void NUMBER(int d, int n, uint8_t h, uint8_t s, uint8_t v) {
  int num = numbers[n];
  int index = 0;
  //this for loop set the color of all of the pixels in the given digit
  for (int i = (NUM_LEDS / 4) * d; i < (NUM_LEDS / 4) * (d + 1); i++) {
    leds[i] = CHSV (h, s, v);
  }
  // this for loop turns the particular segments of the digit that should not glo to black using the blacken() function
  for (int seg = 0; seg < 8 ; seg++) {
    if (!bitRead(num, seg)) {
      blacken(d, seg);
    }
  }

}

/*
   This function is used to blacken (turn off) the pixels in a given digit to a
   number can be displayed.

   It accepts two parameters: d = digit (0-3), and s = segment (0-6)
*/

void blacken(int d, int s) {
  int dig = (7 * 8) * d;
  int seg = s;
  for (int i = (8 * seg) + dig; i < ((8 * seg) + dig) + 8; i++) {
    if (i < NUM_LEDS) {
      leds[i] = CHSV (0, 0, 0);
    }

  }
}

/*
   This function splits a number into two digits. For example, the number 12 would be
   split into a 1 and a 2. It only returns one of the digits depending on the inputted
   parameters.

   It accepts two parameters: a = the number to be split, and b = the single digit to be
   returned
*/

int digitSplit(int a, int b) {
  int number = a;

  int dig;
  if (b == 1) {
    dig = number / 10;
  }

  if (b == 2) {
    dig = number % 10;
  }
  return (dig);
}

//This function displays the time on the glows strips based on readings from the RTC
void showTime() {

  /*
     The digitSplit function is used coupled with functions from the DS1307RTC.h library
     to get the 4 digitis to be displayed
  */
  int h1 = digitSplit(hour(), 1);
  int h2 = digitSplit(hour(), 2);
  int m1 = digitSplit(minute(), 1);
  int m2 = digitSplit(minute(), 2);

  /*
    The number function is called 4 times, one time for each digit
  */
  NUMBER(0, h1, hue1, 255, 255);
  NUMBER(1, h2, hue2, 255, 255);
  NUMBER(2, m1, hue3, 255, 255);
  NUMBER(3, m2, hue4, 255, 255);

  FastLED.show();

}

//this function is used to display the time of the countdown timer 
void counter(int a, int b) {

  int min1 = digitSplit(a, 1);
  int min2 = digitSplit(a, 2);
  int sec1 = digitSplit(b, 1);
  int sec2 = digitSplit(b, 2);
  NUMBER(0, min1, 255, 255, 255);
  NUMBER(1, min2, 255, 255, 255);
  NUMBER(2, sec1, 255, 255, 255);
  NUMBER(3, sec2, 255, 255, 255);
  FastLED.show();
}


void loop() {
  // Display the current time on the glow strips
  showTime();

  // Check if the top coin is pressed (capacitive sensor), if it is, make random colors
  if (checkCap(topCoin)) {
    // Play a tone on the buzzer
    for (int i = 500; i < 550; i++) {
      tone(buzz, i * 2);
      delay(1);
    }
    noTone(buzz);

    // Change the color of the glow strips to a rainbow fill
    fill_rainbow(leds, NUM_LEDS, CRGB::Blue);
    FastLED.show();

    // Generate random hues for the digits' color
    hue1 = random(0, 255);
    hue2 = random(0, 255);
    hue3 = random(0, 255);
    hue4 = random(0, 255);

    // Wait for the bottom coin to be pressed within 1 second, if it is pressed, make all digits the same color
    for (int i = 0; i < 1000; i++) {
      if (checkCap(bottomCoin)) {
        // Play a tone on the buzzer
        for (int i = 500; i < 550; i++) {
          tone(buzz, i * 2);
          delay(1);
        }
        noTone(buzz);

        // Set the hues of all digits to be the same
        hue1 = hue2 = hue3 = hue4;
      }
      delay(1);
    }
  }

  // Check if the bottom coin is pressed (capacitive sensor), if it is, enter countdown mode
  if (checkCap(bottomCoin)) {
    // Play a tone on the buzzer
    for (int i = 500; i < 550; i++) {
      tone(buzz, i * 2);
      delay(1);
    }
    noTone(buzz);

    // Clear the glow strips
    FastLED.clear();

    // Set the initial countdown time to 1 minute and 0 seconds
    int MIN = 1;
    int SEC = 0;

    // Display the countdown time on the glow strips
    counter(MIN, SEC);
    delay(400);

    // Start the countdown
    for (int i = 0; i < 500; i++) {
      if (checkCap(bottomCoin)) {
        // Play a tone on the buzzer
        for (int i = 500; i < 550; i++) {
          tone(buzz, i * 2);
          delay(1);
        }
        noTone(buzz);

        // Increment the minutes by 1 and reset the loop counter
        MIN++;
        i = 0;
        delay(500);
      }

      //if the top coin is pressed in countdown mode, set timer to 20 mins
      if (checkCap(topCoin)) {
        // Play a tone on the buzzer
        for (int i = 500; i < 550; i++) {
          tone(buzz, i * 2);
          delay(1);
        }
        noTone(buzz);

        // Set the countdown time to 20 minutes
        MIN = 20;
      }
      delay(1);
      // Update the countdown display
      counter(MIN, SEC);
    }

    // Update the seconds to 59 and decrement the minutes by 1
    SEC = 59;
    MIN -= 1;

    // Play a tone on the buzzer
    tone(buzz, 600, 400);

    // Continue the countdown
    while (MIN > 0) {
      if (checkCap(topCoin)) {
        // If the top coin is pressed, reset the countdown time
        MIN = 0;
        SEC = 0;
      }

      if (checkCap(bottomCoin)) {
        // If the bottom coin is pressed, increment the minutes by 1
        MIN++;
      }

      // Update the countdown display
      counter(MIN, SEC);

      // Decrement the seconds
      if (SEC > 0) {
        SEC--;
      } else {
        // If the seconds reach 0, reset them to 59 and decrement the minutes by 1
        SEC = 59;
        MIN--;
      }

      // Delay for 1 second
      delay(1000);
    }

    // Play a tone on the buzzer
    tone(buzz, 800, 400);

    // Clear the glow strips
    FastLED.clear();
  }
}
