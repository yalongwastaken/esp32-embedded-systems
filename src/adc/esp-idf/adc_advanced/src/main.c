// Author: Anthony Yalong
// Description: ESP32 ADC and PWM control system with dual sensor monitoring.
//              Potentiometer controls LED brightness via moving average filtering.
//              Light sensor controls LED with hysteresis and debouncing.

#include "adc_advanced_esp.h"

// logging
const char *TAG = "adc_advanced (esp)";

// handles
static adc_oneshot_unit_handle_t adc1_handle;
static TaskHandle_t task_potentiometer_handle;
static TaskHandle_t task_light_sensor_handle;

void app_main() {
    // error management
    esp_err_t ret;

    ret = init_adc();
    if (ret != ESP_OK) {
        halt_program();
    }

    ret = init_pwm();
    if (ret != ESP_OK) {
        halt_program();
    }

    // potentiometer task
    xTaskCreate(
        task_potentiometer,
        "task_potentiometer",
        TASK_STACK_DEPTH,
        NULL,
        TASK_PRIO,
        &task_potentiometer_handle
    );

    // light sensor task
    xTaskCreate(
        task_light_sensor,
        "task_light_sensor",
        TASK_STACK_DEPTH,
        NULL,
        TASK_PRIO,
        &task_light_sensor_handle
    );
}

void halt_program(void) {
    while (true) {
        ESP_LOGW(TAG, "system error!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t init_adc(void) {
    esp_err_t ret;

    // initialize adc unit
    adc_oneshot_unit_init_cfg_t adc_unit = {
        .unit_id = ADC_UNIT
    };
    ret = adc_oneshot_new_unit(&adc_unit, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to initialize ADC unit");
        return ret;
    }

    // initialize adc channel config
    adc_oneshot_chan_cfg_t adc_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH  
    };

    // configure potentiometer channel
    ret = adc_oneshot_config_channel(adc1_handle, PIN_POT, &adc_config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to configure potentiometer channel");
        return ret;
    }

    // configure light sensor channel
    ret = adc_oneshot_config_channel(adc1_handle, PIN_LIGHT, &adc_config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to configure light sensor channel");
        return ret;
    }
    
    ESP_LOGI(TAG, "adc setup complete");
    return ESP_OK;
}

esp_err_t init_pwm(void) {
    esp_err_t ret;

    // potentiometer timer
    ledc_timer_config_t timer_pot = {
        .speed_mode = LEDC_SPEED_MODE,
        .duty_resolution = LEDC_RESOLUTION_POT,
        .timer_num = LEDC_TIMER_POT,
        .clk_cfg = LEDC_CLK_CFG,
        .freq_hz = LEDC_FREQ_POT
    };
    ret = ledc_timer_config(&timer_pot);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to configure potentiometer ledc timer");
        return ret;
    }

    // configure potentiometer led
    ledc_channel_config_t pot_channel = {
        .gpio_num = LED_POT,
        .channel = LEDC_CHANNEL_POT,
        .duty = 0,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_POT,
        .speed_mode = LEDC_SPEED_MODE,
        .sleep_mode = LEDC_SLEEP_MODE_INVALID
    };
    ret = ledc_channel_config(&pot_channel);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to configure potentiometer led");
        return ret;
    }

    // light sensor timer
    ledc_timer_config_t timer_light = {
        .speed_mode = LEDC_SPEED_MODE,
        .duty_resolution = LEDC_RESOLUTION_LIGHT,
        .timer_num = LEDC_TIMER_LIGHT,
        .clk_cfg = LEDC_CLK_CFG,
        .freq_hz = LEDC_FREQ_LIGHT
    };
    ret = ledc_timer_config(&timer_light);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to configure light sensor ledc timer");
        return ret;
    }

    // configure light sensor led
    ledc_channel_config_t light_channel = {
        .gpio_num = LED_LIGHT,
        .channel = LEDC_CHANNEL_LIGHT,
        .duty = 0,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_LIGHT,
        .speed_mode = LEDC_SPEED_MODE,
        .sleep_mode = LEDC_SLEEP_MODE_INVALID
    };
    ret = ledc_channel_config(&light_channel);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to configure light sensor led");
        return ret;
    }

    ESP_LOGI(TAG, "ledc setup complete");
    return ESP_OK;
}

void task_potentiometer(void *pvParameters) {
    // error handling
    esp_err_t ret;

    // sampling
    uint32_t pot_readings[SAMPLES] = {0};
    uint8_t pot_read_index = 0;
    uint32_t pot_total = 0;
    uint32_t pot_average = 0;
    
    // timing
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(DELAY_POT);
        
    while (true) {
        // subtract old
        pot_total -= pot_readings[pot_read_index];
        
        // read new
        int new_reading;
        ret = adc_oneshot_read(adc1_handle, PIN_POT, &new_reading);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "failed to read ADC for potentiometer: %s", esp_err_to_name(ret));
            vTaskDelayUntil(&last_wake_time, task_period);
            continue;
        }
        
        // store new & update
        pot_readings[pot_read_index] = (uint32_t)new_reading;
        pot_total += pot_readings[pot_read_index];
        pot_read_index = (pot_read_index + 1) % SAMPLES;
        pot_average = pot_total / SAMPLES;
        
        // determine duty
        uint8_t duty = (uint8_t)(pot_average >> (ADC_BITWIDTH - LEDC_RESOLUTION_POT));
        
        // update LED PWM
        ret = ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_POT, duty);
        if (ret == ESP_OK) {
            ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_POT);
        } 
        else {
            ESP_LOGW(TAG, "failed to set PWM duty: %s", esp_err_to_name(ret));
        }
        
        // delay
        vTaskDelayUntil(&last_wake_time, task_period);
    }

    // cleanup
    vTaskDelete(NULL);
}

void task_light_sensor(void *pvParameters) {
    // error handling
    esp_err_t ret;

    // setup
    bool led_state = false;
    TickType_t last_toggle = xTaskGetTickCount();
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(DELAY_LIGHT);
    const TickType_t debounce_period = pdMS_TO_TICKS(LIGHT_DEBOUNCE_TIME);

    while (true) {
        // update timing
        TickType_t cur_time = xTaskGetTickCount();

        // read value
        int light_level;
        ret = adc_oneshot_read(adc1_handle, PIN_LIGHT, &light_level);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "failed to read ADC for light sensor: %s", esp_err_to_name(ret));
            vTaskDelayUntil(&last_wake_time, task_period);
            continue;
        }

        // turn led on if low light level
        if (
            !led_state &&
            light_level < LIGHT_THRESHOLD_LOW &&
            (cur_time - last_toggle >= debounce_period)
        ) {
            led_state = true;
            last_toggle = cur_time;
        }
        // turn led off if high light level
        else if (
            led_state &&
            light_level > LIGHT_THRESHOLD_HIGH &&
            (cur_time - last_toggle >= debounce_period)
        ) {
            led_state = false;
            last_toggle = cur_time;
        }

        // determine duty
        uint16_t duty = 0;
        if (led_state) {
            uint16_t max_duty = (1U << LEDC_RESOLUTION_LIGHT) - 1;
            duty = max_duty - (uint16_t)(light_level >> (ADC_BITWIDTH - LEDC_RESOLUTION_LIGHT));
        }

        // update LED PWM
        ret = ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_LIGHT, duty);
        if (ret == ESP_OK) {
            ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_LIGHT);
        } 
        else {
            ESP_LOGW(TAG, "failed to set PWM duty: %s", esp_err_to_name(ret));
        }
        vTaskDelayUntil(&last_wake_time, task_period);
    }

    // cleanup
    vTaskDelete(NULL);
}