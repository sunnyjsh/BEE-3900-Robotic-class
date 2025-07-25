#include <FastLED.h>
#define NUM_LEDS 8
#define DATA_PIN 9

// Define the array of leds
CRGB leds[NUM_LEDS];

void setup() {
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(50);
}

void loop() {
  // Turn on red LED
  leds[0] = CRGB::Red;
  // Turn on green LED
  leds[1] = CRGB::Green;
  // Turn on blue LED
  leds[2] = CRGB::Blue;
  leds[3] = CRGB::Pink;
  leds[4] = CRGB::Yellow;
  leds[5] = CRGB::Orange;
  leds[6] = CRGB::Aqua;
  leds[7] = CRGB::Beige;
  // Update LEDs
  FastLED.show();
  // Wait for half a second
  delay(500);
  // Turn off green LED
  leds[1] = CRGB::Black;
  // Update LEDs
  FastLED.show();
  // Wait for half a second
  delay(500);
}
