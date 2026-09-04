#include "BluetoothSerial.h"
#include "DHT.h"

// This checks if Bluetooth is properly configured in the IDE settings
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Create the Bluetooth Object
BluetoothSerial SerialBT;

// Define sensor settings
#define DHTPIN 4        // The GPIO pin connected to the sensor
#define DHTTYPE DHT11   // Change to DHT22 if you are using the white sensor

// Create the Sensor Object
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  // Start the standard USB serial monitor for debugging
  Serial.begin(115200);
  
  // Start the sensor
  dht.begin();
  
  // Start Bluetooth and broadcast this name
  SerialBT.begin("ESP32_Weather_Station"); 
  
  Serial.println("Bluetooth Started! Ready to pair.");
}

void loop() {
  // The DHT11 needs at least 2 seconds between readings
  delay(2000);

  // Read humidity and temperature (in Celsius)
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Check if the reading failed (wiring issue)
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from sensor! Check wiring.");
    SerialBT.println("Failed to read from sensor! Check wiring.");
    return;
  }

  // 1. Print to the Arduino IDE over the USB cable
  Serial.print("Temp: ");
  Serial.print(temperature);
  Serial.print(" °C  |  Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  // 2. Transmit over the airwaves via Bluetooth!
  SerialBT.print("Temp: ");
  SerialBT.print(temperature);
  SerialBT.print(" C  |  Humidity: ");
  SerialBT.print(humidity);
  SerialBT.println(" %");
}