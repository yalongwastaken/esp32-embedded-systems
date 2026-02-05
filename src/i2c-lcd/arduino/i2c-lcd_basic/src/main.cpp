// Author: Anthony Yalong
// Description: Non-blocking I2C scanner

#include "i2c-lcd_basic_arduino.h"

// logging
const char *TAG = "i2c-lcd_basic (arduino)";

// state machine for non-blocking operation
enum scan_state_t {
  STATE_IDLE,
  STATE_SCANNING,
  STATE_WAIT_NEXT_SCAN
};

// state variables
scan_state_t current_state = STATE_IDLE;
uint8_t current_addr = I2C_START_ADDR;
uint8_t device_count = 0;
unsigned long last_scan_time = 0;
unsigned long last_probe_time = 0;

void setup() {
  // initialize serial monitor
  Serial.begin(BAUDRATE);
  while (!Serial) delay(10);
  
  // initialize i2c
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(CLOCK_SPEED_DEFAULT);
  Serial.printf("%s: initialized I2C bus\n", TAG);
  
  // start first scan
  current_state = STATE_SCANNING;
  Serial.printf("%s: starting scan...\n", TAG);
}

void loop() {
  unsigned long current_time = millis();
  
  switch (current_state) {
    case STATE_IDLE:
      current_state = STATE_SCANNING;
      current_addr = I2C_START_ADDR;
      device_count = 0;
      Serial.printf("%s: starting scan...\n", TAG);
      break;
      
    case STATE_SCANNING:
      if (current_time - last_probe_time >= 10) {
        last_probe_time = current_time;
        
        // probe current address
        Wire.beginTransmission(current_addr);
        uint8_t response = Wire.endTransmission();
        
        switch (response) {
          case 0: 
            Serial.printf("%s: device found at 0x%02X\n", TAG, current_addr);
            device_count++;
            break;
          case 2:
            // NACK on address (expected when no device present)
            break;
          case 3:
            Serial.printf("%s: NACK on data at 0x%02X\n", TAG, current_addr);
            break;
          case 4:
            Serial.printf("%s: unknown error at 0x%02X\n", TAG, current_addr);
            break;
          default:
            Serial.printf("%s: unexpected error code %d at 0x%02X\n", TAG, response, current_addr);
        }
        
        // move to next address
        current_addr++;
        
        // check if scan is complete
        if (current_addr > I2C_END_ADDR) {
          Serial.printf("%s: scan complete\n", TAG);
          if (device_count == 0) {
            Serial.printf("%s: no devices found\n", TAG);
          }
          else {
            Serial.printf("%s: %d device(s) found\n", TAG, device_count);
          }
          Serial.printf("%s: scanning again in 5 seconds...\n", TAG);
          
          current_state = STATE_WAIT_NEXT_SCAN;
          last_scan_time = current_time;
        }
      }
      break;
      
    case STATE_WAIT_NEXT_SCAN:
      if (current_time - last_scan_time >= SCAN_DELAY) {
        // Reset for next scan
        current_addr = I2C_START_ADDR;
        device_count = 0;
        current_state = STATE_SCANNING;
        Serial.printf("%s: starting scan...\n", TAG);
      }
      break;
  }
}