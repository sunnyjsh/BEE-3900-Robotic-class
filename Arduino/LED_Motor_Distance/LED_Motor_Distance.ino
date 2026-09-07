#include <Wire.h>
#include <VL53L1X.h>
#include <PWMServo.h>
#include <FastLED.h>

// --- PIN DEFINITIONS ---
#define SERVO_PIN 11  // Hardware PWM pin on Arduino Mega for PWMServo
#define LED_PIN 45    // Digital pin for WS2812B data
#define NUM_LEDS 8    // Adjust to match your LED ring

// --- OBJECT INSTANTIATION ---
VL53L1X distanceSensor;
PWMServo myServo;
CRGB leds[NUM_LEDS];

// --- SERVO SWEEP VARIABLES ---
int servoAngle = 180;
int servoStep = -1; // Controls the direction of the sweep

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // 400 kHz I2C for the distance sensor

  // 1. Initialize FastLED
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(50); 

  // 2. Initialize PWMServo
  myServo.attach(SERVO_PIN);
  myServo.write(servoAngle); // Start at 180 degrees

  // 3. Initialize VL53L1X Sensor
  distanceSensor.setTimeout(500);
  if (!distanceSensor.init()) {
    Serial.println("Failed to detect and initialize VL53L1X!");
    while (1);
  }
  distanceSensor.setDistanceMode(VL53L1X::Medium);
  distanceSensor.setMeasurementTimingBudget(50000);
  distanceSensor.startContinuous(50);
}

void loop() {
  // --- DISTANCE & LED LOGIC ---
  distanceSensor.read();
  int distance_mm = distanceSensor.ranging_data.range_mm;
  distance_mm = constrain(distance_mm, 50, 400);

  // Map distance to a color hue (0 = Red, 96 = Green, 160 = Blue)
  int ledHue = map(distance_mm, 50, 400, 0, 160);
  fill_solid(leds, NUM_LEDS, CHSV(ledHue, 255, 255));
  FastLED.show();

  // --- GRADUAL SERVO SWEEP LOGIC ---
  // Increment or decrement the angle
  servoAngle += servoStep;
  
  // Reverse direction if it hits the 90 or 180 boundary
  if (servoAngle <= 90) {
    servoAngle = 90;
    servoStep = 1; // Start moving back up to 180
  } else if (servoAngle >= 135) {
    servoAngle = 135;
    servoStep = -1; // Start moving down to 90
  }
  
  myServo.write(servoAngle);

  // --- SERIAL MONITOR TROUBLESHOOTING ---
  Serial.print("Distance: ");
  Serial.print(distance_mm);
  Serial.print(" mm | LED Hue: ");
  Serial.print(ledHue);
  Serial.print(" | Servo Angle: ");
  Serial.println(servoAngle);

  // 50ms delay creates a smooth, gradual sweep for the servo
  delay(50); 
}