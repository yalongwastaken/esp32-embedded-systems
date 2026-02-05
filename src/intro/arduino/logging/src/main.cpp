// Author: Anthony Yalong
// Description: Device logging for the esp32 using an Arduino framework.

#include <Arduino.h>

static unsigned long last_log = millis();

// Logging function
void log_esp32_device() {
  Serial.println("=== DEVICE LOG ===");
  Serial.printf("Uptime: %lu seconds\n", millis() / 1000);
  Serial.printf("Free Memory: %u bytes\n", ESP.getFreeHeap());
  Serial.printf("CPU Frequency: %u MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Temperature: %.1f°C\n", temperatureRead());
  Serial.printf("Flash Size: %u bytes\n", ESP.getFlashChipSize());
  Serial.println("==================");
}

// Serial setup
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Device Logger Started.");
}

// Logging loop
void loop() {
  if (millis() - last_log > 10000) {
    log_esp32_device();
    last_log = millis();
  }
}