// Author: Anthony Yalong
// Description: Device logging for the esp32 using an ESP-IDF framework.

#include <stdio.h>
#include <soc/rtc.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <driver/temperature_sensor.h>
#include <esp_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "Logger (ESP-IDF): ";

void log_esp32_device(void *pvParameters) {
    ESP_LOGI(TAG, "ESP32 Device Logger Started.");

    while (1) {
        ESP_LOGI(TAG, "=== DEVICE LOG ===");
    
        // Uptime
        ESP_LOGI(TAG, "Uptime: %llu seconds", esp_timer_get_time() / 1000000);
        
        // Free Memory
        ESP_LOGI(TAG, "Free Memory: %u bytes", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
        
        // CPU Frequency
        rtc_cpu_freq_config_t freq_config;
        rtc_clk_cpu_freq_get_config(&freq_config);
        ESP_LOGI(TAG, "CPU Frequency: %lu MHz", freq_config.freq_mhz);
        
        // Flash Size
        uint32_t flash_size;
        esp_flash_get_size(NULL, &flash_size);
        ESP_LOGI(TAG, "Flash Size: %lu bytes", flash_size);
        
        ESP_LOGI(TAG, "==================");

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
} 

void app_main() {
   xTaskCreate(log_esp32_device, "ESP32 Device Logger", 2048, NULL, 1, NULL);
}