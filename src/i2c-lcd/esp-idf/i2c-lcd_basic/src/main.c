/**
 * @file main.c
 * @author Anthony Yalong
 * @brief I2C bus scanner that continuously probes for connected devices
 */

#include "i2c-lcd_basic_esp.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// logging
static const char *TAG = "i2c-lcd_basic (esp)";

// task timing
static TickType_t i2c_task_last_probe;

void app_main() {
    // error management
    esp_err_t ret;

    // initialize i2c master
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize i2c master");
        halt_program();
    }

    // initialize task
    i2c_task_last_probe = xTaskGetTickCount();

    // create task
    xTaskCreate(
        i2c_scanner_task,
        "i2c_scanner_task",
        I2C_TASK_STACK_DEPTH,
        NULL,
        I2C_TASK_PRIO,
        NULL
    );
}

void halt_program(void) {
    while (true) {
        ESP_LOGW(TAG, "system error!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t i2c_master_init(void) {
    // error management
    esp_err_t ret;

    // i2c config
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    ret = i2c_param_config(I2C_NUM, &i2c_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initalize i2c param config");
        return ret;
    }

    ret = i2c_driver_install(
        I2C_NUM,
        i2c_config.mode,
        I2C_MASTER_RX_BUFF_SIZE,
        I2C_MASTER_TX_BUFF_SIZE,
        0
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to install i2c driver");
        return ret;
    }

    ESP_LOGI(TAG, "initialized i2c master");
    return ESP_OK;
}

esp_err_t i2c_probe_address(uint8_t addr) {
    // error management
    esp_err_t ret;

    // transaction descriptor
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // build i2c transaction
    i2c_master_start(cmd);

    // send address byte with WRITE bit
    i2c_master_write_byte(cmd, (addr << 1) | 0x00, I2C_ACK_CHECK_EN);

    // stop condition
    i2c_master_stop(cmd);

    // execute transaction
    ret = i2c_master_cmd_begin(
        I2C_NUM,
        cmd,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );

    // cleanup
    i2c_cmd_link_delete(cmd);

    return ret;
}

void i2c_scanner_task(void *pvParameters) {
    while (true) {
        uint8_t device_count = 0;

        // check address space
        for (uint8_t addr = I2C_START_ADDR; addr < I2C_END_ADDR; addr++) {
            esp_err_t ret = i2c_probe_address(addr);

            switch (ret) {
                // device found
                case ESP_OK:
                    device_count++;
                    ESP_LOGI(TAG, "device found at 0x%02X", addr);
                    break;
                // invalid argument
                case ESP_ERR_INVALID_ARG:
                    ESP_LOGE(TAG, "invalid argument");
                    break;
                // device not found
                case ESP_FAIL:
                    break;
                // driver not installed or in master mode
                case ESP_ERR_INVALID_STATE:
                    ESP_LOGE(TAG, "driver not properly installed");
                    break;
                case ESP_ERR_TIMEOUT:
                    ESP_LOGW(TAG, "timeout at 0x%02X", addr);
                    break;
            }
            // small delay
            vTaskDelayUntil(&i2c_task_last_probe, pdMS_TO_TICKS(I2C_TASK_ADDRESS_PROBE_DELAY_MS));
        }

        // results
        if (device_count == 0) {
            ESP_LOGI(TAG, "no devices found");
        }
        else {
            ESP_LOGI(TAG, "found %d devices", device_count);
        }

        ESP_LOGI(TAG, "scanning again in 5 seconds...");
        vTaskDelayUntil(&i2c_task_last_probe, pdMS_TO_TICKS(I2C_TASK_DELAY_MS));
    }

    vTaskDelete(NULL);
}