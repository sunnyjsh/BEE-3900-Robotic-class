/*
 * ESP32-S3-WROOM (Freenove) — photo every 5s, sent to a computer over BLE.
 * The ESP32-S3 has NO Classic Bluetooth (no SerialBT). This uses BLE, so a
 * small Python script (ble_receiver.py) must run on your computer.
 *
 * Arduino IDE Tools settings (THESE MATTER):
 *   Board: "ESP32S3 Dev Module"
 *   PSRAM: "OPI PSRAM"        <-- camera won't init without this
 *   Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
 *   USB CDC On Boot: "Enabled"
 */
#include "esp_camera.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---- Camera pins: Freenove ESP32-S3-WROOM ----
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM   4
#define SIOC_GPIO_NUM   5
#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM     8
#define Y3_GPIO_NUM     9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM  6
#define HREF_GPIO_NUM   7
#define PCLK_GPIO_NUM  13

// ---- BLE IDs (must match ble_receiver.py) ----
#define SERVICE_UUID   "a1b2c3d4-0001-4a5b-8c6d-1234567890ab"
#define CTRL_CHAR_UUID "a1b2c3d4-0002-4a5b-8c6d-1234567890ab"
#define DATA_CHAR_UUID "a1b2c3d4-0003-4a5b-8c6d-1234567890ab"

const unsigned long INTERVAL_MS = 5000;
const size_t        CHUNK_SIZE  = 180;
const uint16_t      CHUNK_DELAY = 6;

BLECharacteristic *ctrlChar, *dataChar;
bool deviceConnected = false;
unsigned long lastCapture = 0;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) { deviceConnected = true; Serial.println("Computer connected."); }
  void onDisconnect(BLEServer *s) { deviceConnected = false; Serial.println("Disconnected, advertising..."); s->getAdvertising()->start(); }
};

bool startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA; config.jpeg_quality = 12; config.fb_count = 2;
  } else {
    Serial.println("WARNING: PSRAM not found.");
    config.frame_size = FRAMESIZE_UXGA; config.jpeg_quality = 15;
    config.fb_count = 1; config.fb_location = CAMERA_FB_IN_DRAM;
  }
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) { Serial.printf("Camera init failed 0x%x\n", err); return false; }
  Serial.println("Camera ready.");
  return true;
}

void sendPhoto() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { Serial.println("Capture failed."); return; }
  uint32_t len = fb->len;
  uint8_t header[4] = { (uint8_t)(len&0xFF), (uint8_t)((len>>8)&0xFF),
                        (uint8_t)((len>>16)&0xFF), (uint8_t)((len>>24)&0xFF) };
  ctrlChar->setValue(header, 4); ctrlChar->notify(); delay(25);
  size_t sent = 0;
  while (sent < fb->len) {
    size_t n = (fb->len - sent) < CHUNK_SIZE ? (fb->len - sent) : CHUNK_SIZE;
    dataChar->setValue(fb->buf + sent, n); dataChar->notify();
    sent += n; delay(CHUNK_DELAY);
  }
  Serial.printf("Sent image: %u bytes\n", len);
  esp_camera_fb_return(fb);
}

void setup() {
  Serial.begin(115200); delay(500);
  Serial.println("\nESP32-S3 Camera + BLE starting...");
  if (!startCamera()) { while (true) delay(1000); }
  BLEDevice::init("ESP32S3-CAM");
  BLEDevice::setMTU(185);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService *service = server->createService(SERVICE_UUID);
  ctrlChar = service->createCharacteristic(CTRL_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  ctrlChar->addDescriptor(new BLE2902());
  dataChar = service->createCharacteristic(DATA_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  dataChar->addDescriptor(new BLE2902());
  service->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID); adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("Advertising as 'ESP32S3-CAM'. Run ble_receiver.py on your computer.");
}

void loop() {
  if (deviceConnected && (millis() - lastCapture >= INTERVAL_MS)) {
    lastCapture = millis();
    sendPhoto();
  }
  delay(10);
}