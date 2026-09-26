// dht11_thingspeak.ino
// Reads humidity and temperature from a DHT11 and prints one line per reading
// over USB serial for pi_uploader_dht11.py to parse and upload to ThingSpeak.
//
// Output format on a good read (what the Python script expects):
//   DHT11, \tOK,\t54.0,\t23.5
//   -> second-to-last field = humidity (%), last field = temperature (C)
//
// Requires: "DHT sensor library" by Adafruit (plus "Adafruit Unified Sensor"),
// installed via Arduino IDE > Tools > Manage Libraries.
//
// Wiring (DHT11 module):
//   VCC  -> 5V
//   GND  -> GND
//   DATA -> digital pin 2
//   (Bare 4-pin DHT11 sensors also need a 10k pull-up resistor between DATA and VCC;
//    most 3-pin breakout modules already include one.)

#include <DHT.h>

#define DHTPIN   2       // Digital pin connected to the DHT11 data line
#define DHTTYPE  DHT11

// ThingSpeak free accounts accept at most one update every 15 seconds,
// so send a reading every 20 seconds to stay safely under the limit.
const unsigned long READ_INTERVAL_MS = 20000;

DHT dht(DHTPIN, DHTTYPE);
unsigned long lastReadTime = 0;

void setup() {
  Serial.begin(115200);   // Must match BAUD_RATE in the Python script
  dht.begin();

  // Header lines: the Python script can't parse these as numbers, so it skips them.
  Serial.println("DHT11 TEST PROGRAM");
  Serial.println("Type,\tStatus,\tHumidity (%),\tTemperature (C)");

  delay(2000);            // DHT11 needs ~1-2 s after power-up before first read
  lastReadTime = millis() - READ_INTERVAL_MS;  // take the first reading right away
}

void loop() {
  unsigned long now = millis();
  if (now - lastReadTime < READ_INTERVAL_MS) {
    return;
  }
  lastReadTime = now;

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();   // Celsius; use readTemperature(true) for Fahrenheit

  // On a failed read, print only two fields so the Python script skips the line.
  // (Printing "nan" values would slip through, since Python's float("nan") succeeds.)
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("DHT11, \tRead error");
    return;
  }

  Serial.print("DHT11, \tOK,\t");
  Serial.print(humidity, 1);
  Serial.print(",\t");
  Serial.println(temperature, 1);
}
