/**
 * @file main.c
 * @author Anthony Yalong
 * @brief I2C LCD with potentiometer-controlled PWM LED brightness
 */

#include <stdio.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c-lcd_advanced_esp.h"

// ============================================================================
// Global Variables
// ============================================================================

static const char *TAG = "i2c-lcd_advanced";

static lcd_handle_t lcd;
static adc_oneshot_unit_handle_t adc;
static pot_state_t pot_state;
static int8_t last_displayed_brightness;
static SemaphoreHandle_t pot_state_mutex;

// ============================================================================
// Main Application
// ============================================================================

void app_main(void) {
    esp_err_t ret;

    ESP_LOGI(TAG, "starting application");

    // task synchronization
    pot_state_mutex = xSemaphoreCreateMutex();

    // initialize i2c
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        halt_program();
    }

    // initialize lcd
    ret = lcd_init(&lcd, I2C_NUM, LCD_ADDR, LCD_COLUMNS, LCD_ROWS);
    if (ret != ESP_OK) {
        halt_program();
    }
    lcd_backlight(&lcd, true);
    lcd_clear(&lcd);

    // initialize adc
    ret = adc_init();
    if (ret != ESP_OK) {
        halt_program();
    }

    // initialize pwm
    ret = pwm_init();
    if (ret != ESP_OK) {
        halt_program();
    }

    // initialize potentiometer state
    pot_state.total = 0;
    pot_state.index = 0;
    pot_state.brightness_pct = 0;
    int new_read;
    for (uint8_t i = 0; i < POT_SAMPLING; i++) {
        ret = adc_oneshot_read(adc, ADC_CHANNEL, &new_read);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "failed to read initial adc samples");
            halt_program();
        }
        pot_state.samples[pot_state.index] = (uint32_t) new_read;
        pot_state.total += pot_state.samples[i];
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // initialize lcd task state
    last_displayed_brightness = -1;

    // create tasks
    xTaskCreate(pot_task, "pot_task", POT_TASK_STACK_DEPTH, NULL, POT_TASK_PRIO, NULL);
    xTaskCreate(lcd_task, "lcd_task", LCD_TASK_STACK_DEPTH, NULL, LCD_TASK_PRIO, NULL);

    ESP_LOGI(TAG, "application started successfully");
}

// ============================================================================
// Utility Functions
// ============================================================================

void halt_program(void) {
    while (1) {
        ESP_LOGE(TAG, "system error - halted");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// I2C Functions
// ============================================================================

esp_err_t i2c_master_init(void) {
    esp_err_t ret;

    // configure i2c parameters
    i2c_config_t master_config = {
        .mode = I2C_MODE,
        .sda_io_num = I2C_SDA_PIN_NUM,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_PIN_NUM,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_CLK_SPEED_HZ,
    };
    ret = i2c_param_config(I2C_NUM, &master_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to configure i2c parameters");
        return ret;
    }

    // install i2c driver
    ret = i2c_driver_install(
        I2C_NUM,
        master_config.mode,
        I2C_MASTER_RX_BUFF_SIZE,
        I2C_MASTER_TX_BUFF_SIZE,
        0
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to install i2c driver");
        return ret;
    }

    ESP_LOGI(TAG, "i2c initialized");
    return ESP_OK;
}

// ============================================================================
// LCD Functions
// ============================================================================

esp_err_t lcd_pulse_enable(lcd_handle_t *lcd, uint8_t data) {
    esp_err_t err;
    i2c_cmd_handle_t cmd;
    
    // pulse enable high
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lcd->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data | LCD_EN, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(lcd->i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // pulse enable low
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lcd->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data & ~LCD_EN, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(lcd->i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    vTaskDelay(pdMS_TO_TICKS(1));
    
    return err;
}

esp_err_t lcd_write_nibble(lcd_handle_t *lcd, uint8_t nibble, uint8_t mode) {
    uint8_t data = nibble | mode | lcd->backlight_state;
    return lcd_pulse_enable(lcd, data);
}

esp_err_t lcd_write_byte(lcd_handle_t *lcd, uint8_t data, uint8_t mode) {
    esp_err_t err;
    
    // write high nibble
    err = lcd_write_nibble(lcd, data & 0xF0, mode);
    if (err != ESP_OK) return err;
    
    // write low nibble
    err = lcd_write_nibble(lcd, (data << 4) & 0xF0, mode);
    return err;
}

esp_err_t lcd_send_command(lcd_handle_t *lcd, uint8_t cmd) {
    return lcd_write_byte(lcd, cmd, 0);
}

esp_err_t lcd_send_data(lcd_handle_t *lcd, uint8_t data) {
    return lcd_write_byte(lcd, data, LCD_RS);
}

esp_err_t lcd_init(lcd_handle_t *lcd, i2c_port_t i2c_port, uint8_t addr, 
                   uint8_t cols, uint8_t rows) {
    if (lcd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    lcd->i2c_port = i2c_port;
    lcd->addr = addr;
    lcd->cols = cols;
    lcd->rows = rows;
    lcd->backlight_state = LCD_BACKLIGHT;
    
    // wait for lcd power up
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // initialize in 4-bit mode (send 0x30 three times)
    lcd_write_nibble(lcd, 0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(lcd, 0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(lcd, 0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // set to 4-bit mode
    lcd_write_nibble(lcd, 0x20, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // function set: 4-bit mode, 2 lines, 5x8 dots
    lcd_send_command(lcd, LCD_CMD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2_LINE | LCD_5x8_DOTS);
    
    // display control: display on
    lcd_send_command(lcd, LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_ON);
    
    // clear display
    lcd_clear(lcd);
    
    // entry mode: left to right
    lcd_send_command(lcd, LCD_CMD_ENTRY_MODE | LCD_ENTRY_LEFT);
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "lcd initialized at address 0x%02X", addr);
    return ESP_OK;
}

esp_err_t lcd_clear(lcd_handle_t *lcd) {
    esp_err_t err = lcd_send_command(lcd, LCD_CMD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(2));
    return err;
}

esp_err_t lcd_set_cursor(lcd_handle_t *lcd, uint8_t col, uint8_t row) {
    if (col >= lcd->cols || row >= lcd->rows) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // row offsets for different lcd sizes
    static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    uint8_t addr = col + row_offsets[row];
    
    return lcd_send_command(lcd, LCD_CMD_DDRAM_ADDR | addr);
}

esp_err_t lcd_backlight(lcd_handle_t *lcd, bool state) {
    lcd->backlight_state = state ? LCD_BACKLIGHT : LCD_NO_BACKLIGHT;
    
    // send backlight state to i2c
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lcd->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, lcd->backlight_state, true);
    i2c_master_stop(cmd);
    
    esp_err_t err = i2c_master_cmd_begin(lcd->i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return err;
}

esp_err_t lcd_print(lcd_handle_t *lcd, const char *str) {
    if (str == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    while (*str) {
        esp_err_t err = lcd_send_data(lcd, *str++);
        if (err != ESP_OK) {
            return err;
        }
    }
    
    return ESP_OK;
}

esp_err_t lcd_printf(lcd_handle_t *lcd, const char *format, ...) {
    char buffer[64];
    va_list args;
    
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    return lcd_print(lcd, buffer);
}

// ============================================================================
// ADC Functions
// ============================================================================

esp_err_t adc_init(void) {
    esp_err_t ret;

    // configure adc unit
    adc_oneshot_unit_init_cfg_t adc_unit_config = {
        .unit_id = ADC_UNIT
    };
    ret = adc_oneshot_new_unit(&adc_unit_config, &adc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize adc unit");
        return ret;
    }

    // configure adc channel
    adc_oneshot_chan_cfg_t adc_channel_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH
    };
    ret = adc_oneshot_config_channel(adc, ADC_CHANNEL, &adc_channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to configure adc channel");
        return ret;
    }

    ESP_LOGI(TAG, "adc initialized");
    return ESP_OK;
}

// ============================================================================
// PWM Functions
// ============================================================================

esp_err_t pwm_init(void) {
    esp_err_t ret;

    // configure pwm timer
    ledc_timer_config_t timer_config = {
        .timer_num = PWM_TIMER,
        .clk_cfg = PWM_CLK_CFG,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ,
        .speed_mode = PWM_SPEED_MODE,
    };
    ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to configure pwm timer");
        return ret;
    }

    // configure pwm channel
    ledc_channel_config_t channel_config = {
        .timer_sel = PWM_TIMER,
        .channel = PWM_CHANNEL,
        .duty = 0,
        .gpio_num = POT_LED_PIN,
        .hpoint = 0,
        .speed_mode = PWM_SPEED_MODE,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ret = ledc_channel_config(&channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to configure pwm channel");
        return ret;
    }

    ESP_LOGI(TAG, "pwm initialized");
    return ESP_OK;
}

// ============================================================================
// Task Functions
// ============================================================================

void pot_task(void *pvParameters) {
    esp_err_t ret;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        // update moving average filter
        int new_read;
        pot_state.total -= pot_state.samples[pot_state.index];
        ret = adc_oneshot_read(adc, ADC_CHANNEL, &new_read);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "failed to read adc");
            halt_program();
        }
        pot_state.samples[pot_state.index] = (uint32_t) new_read;
        pot_state.total += pot_state.samples[pot_state.index];

        // update circular buffer index
        pot_state.index = (pot_state.index + 1) % POT_SAMPLING;

        // calculate average (integer roundoff intended)
        uint16_t pot_average = pot_state.total / POT_SAMPLING;

        // map 10-bit adc (0-1023) to 8-bit pwm (0-255)
        uint16_t duty = (pot_average * 255) / 1023;
        duty = (duty > 255) ? 255 : duty;
        
        // map duty to percentage (0-100)
        xSemaphoreTake(pot_state_mutex, portMAX_DELAY);
        pot_state.brightness_pct = (duty * 100) / 255;
        pot_state.brightness_pct = (pot_state.brightness_pct > 100) ? 100 : pot_state.brightness_pct;
        xSemaphoreGive(pot_state_mutex);

        // update pwm duty cycle
        ledc_set_duty_and_update(PWM_SPEED_MODE, PWM_CHANNEL, duty, 0);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POT_TASK_DELAY_MS));
    }
}

void lcd_task(void *pvParameters) {
    // setup
    esp_err_t ret;
    TickType_t last_wake = xTaskGetTickCount();
    
    while (1) {
        // get current brightness
        xSemaphoreTake(pot_state_mutex, portMAX_DELAY);
        uint8_t current_brightness = pot_state.brightness_pct;
        xSemaphoreGive(pot_state_mutex);

        // only update lcd when brightness changes
        if (current_brightness != last_displayed_brightness) {
            
            ret = lcd_set_cursor(&lcd, 0, 0);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "LCD cursor set failed, retrying...");
                vTaskDelay(pdMS_TO_TICKS(100));
                continue; // skip this iteration
            }

            ret = lcd_printf(&lcd, "LED Brightness: %d%% ", current_brightness);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "LCD print failed");
            }

            last_displayed_brightness = current_brightness;
        }
        
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LCD_TASK_DELAY_MS));
    }
}