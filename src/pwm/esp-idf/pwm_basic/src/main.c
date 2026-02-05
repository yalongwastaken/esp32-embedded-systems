// Author: Anthony Yalong
// Description: Reads analog potentiometer value and controls LED brightness via PWM.
//              Updates every 50ms with serial monitoring for real-time feedback.

#include "pwm_control.h"

// Logging
const static char *TAG = "PWM_BASIC (ESP-IDF)";
static TaskHandle_t pwm_task_handle;

// Error Queue
static QueueHandle_t error_queue;
static TaskHandle_t error_queue_handle;


void app_main() {
    // Initialize queue
    error_queue = xQueueCreate(QUEUE_SIZE, sizeof(QUEUE_ITEM_TYPE));
    if (error_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create error_queue! Halting system.");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(10000));
            ESP_LOGE(TAG, "ERROR! PLEASE FIX & REFLASH.");
        }
    }

    // Call setup
    init_adc();
    init_pwm();

    BaseType_t task_ret;

    // Main Task Handler
    task_ret = xTaskCreate(
        main_task, "main_task", STACK_DEPTH, NULL, TASK_PRIO, &pwm_task_handle
    );
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create main_task! Halting system.");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(10000));
            ESP_LOGE(TAG, "ERROR! PLEASE FIX & REFLASH.");
        }
    }
    ESP_LOGI(TAG, "app_main: successfully initialized `main_task`");

    // Error Queue Handler
    task_ret = xTaskCreate(
        error_monitor, "error_monitor", STACK_DEPTH, NULL, QUEUE_PRIO, &error_queue_handle
    );
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create error_monitor! Halting system.");
        // Clean main task
        if (pwm_task_handle != NULL) {
            vTaskDelete(pwm_task_handle);
            pwm_task_handle = NULL;
            ESP_LOGW(TAG, "Main task deleted due to error");
        }

        while (1) {
            vTaskDelay(pdMS_TO_TICKS(10000));
            ESP_LOGE(TAG, "ERROR! PLEASE FIX & REFLASH.");
        }
    }
    ESP_LOGI(TAG, "app_main: successfully initialized `error_monitor`");
}

void queue_error(char *source, esp_err_t err) {
    if (err == ESP_OK) return;

    ESP_LOGE(TAG, "ERROR in %s: %s", source, esp_err_to_name(err));

    BaseType_t queue_send_status = pdFALSE;
    int32_t retry_counter = 9;

    while (queue_send_status != pdTRUE && retry_counter >= 0) {
        queue_send_status = xQueueSend(error_queue, &err, pdMS_TO_TICKS(10));
        if (queue_send_status != pdTRUE) {
            ESP_LOGW(TAG, "Error queue full, attempting %ld more times", retry_counter);
            retry_counter--;
        }
    }

    if (queue_send_status != pdTRUE) {
        ESP_LOGE(TAG, "Failed to queue error after all retries!");
    }
}

void init_adc(void) {
    esp_err_t ret;

    ret = adc1_config_width(ADC_WIDTH);
    queue_error("adc1_config_width", ret);

    ret = adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
    queue_error("adc1_config_channel_atten", ret);

    ESP_LOGI(TAG, "init_adc: successfully initialized ADC");
}

void init_pwm(void) {
    esp_err_t ret;

    // Timer
    ledc_timer_config_t timer_config = {
        .speed_mode = PWM_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQ,
        .clk_cfg = PWM_TIMER_CONFIG,
        .deconfigure = false,
    };
    ret = ledc_timer_config(&timer_config);
    queue_error("ledc_timer_config", ret);

    // Channel
    ledc_channel_config_t channel_config = {
        .speed_mode = PWM_SPEED_MODE,
        .gpio_num = LED_NUM,
        .timer_sel = PWM_TIMER,
        .channel = PWM_CHANNEL,
        .duty = 0,
        .hpoint = 0,
    };
    ret = ledc_channel_config(&channel_config);
    queue_error("ledc_channel_config", ret);

    ESP_LOGI(TAG, "init_pwm: successfully initialized LEDC");
}

void set_duty(uint32_t duty) {
    // Clamp duty
    if (duty > ((1 << PWM_RESOLUTION) - 1)) {
        duty = (1 << PWM_RESOLUTION) - 1;
    }

    esp_err_t ret;

    ret = ledc_set_duty(PWM_SPEED_MODE, PWM_CHANNEL, duty);
    queue_error("ledc_set_duty", ret);

    ret = ledc_update_duty(PWM_SPEED_MODE, PWM_CHANNEL);
    queue_error("ledc_update_duty", ret);

    ESP_LOGI(TAG, "set_duty: set & updated duty w/ value: %lu", duty);
}

uint32_t read_raw_adc(void) {
    uint32_t sum = 0;
    for (int i = 0; i < 5; i++) {
        sum += adc1_get_raw(ADC_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return sum / 5;
}

uint32_t map_value(uint32_t value, uint32_t in_min, uint32_t in_max, 
    uint32_t out_min, uint32_t out_max) {
    if (value < in_min) value = in_min;
    if (value > in_max) value = in_max;

    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; 
}

void main_task(void *pvParameters) {
    while (1) {
        uint32_t pot_value = read_raw_adc();

        uint32_t mapped_value = map_value(pot_value, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);

        set_duty(mapped_value);

        vTaskDelay(pdMS_TO_TICKS(PWM_TASK_DELAY));
    }
}

void error_monitor(void *pvParameters) {
    esp_err_t err;
    while(1) {
        if (xQueueReceive(error_queue, &err, portMAX_DELAY) == pdTRUE) {
            // Clean main task
            if (pwm_task_handle != NULL) {
                vTaskDelete(pwm_task_handle);
                pwm_task_handle = NULL;
                ESP_LOGW(TAG, "Main task deleted due to error");
            }

            // Halt
            ESP_LOGW(TAG, "Error monitor halting system");
            while (1) {
                vTaskDelay(pdMS_TO_TICKS(10000));
                ESP_LOGE(TAG, "ERROR! PLEASE FIX & REFLASH.");
            }
        }
    }
}