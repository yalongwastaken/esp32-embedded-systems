// Author: Anthony Yalong
// Description: RGB LED controller that reads potentiometer values 
// from 3 ADC channels and controls LED brightness via PWM

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/ledc.h>

// LED configuration
#define RED_LED GPIO_NUM_25                 // GPIO25
#define RED_PWM_CHANNEL LEDC_CHANNEL_0
#define GREEN_LED GPIO_NUM_26               // GPIO26
#define GREEN_PWM_CHANNEL LEDC_CHANNEL_1
#define BLUE_LED GPIO_NUM_27                // GPIO27
#define BLUE_PWM_CHANNEL LEDC_CHANNEL_2

// ADC configuration
static adc_oneshot_unit_handle_t adc1_handle;
#define RED_ADC_CHANNEL ADC_CHANNEL_0       // GPIO36
#define GREEN_ADC_CHANNEL ADC_CHANNEL_3     // GPIO39
#define BLUE_ADC_CHANNEL ADC_CHANNEL_6      // GPIO34
#define ADC_ATTEN ADC_ATTEN_DB_12           // 0.0V - 3.3V range
#define ADC_BITWIDTH ADC_BITWIDTH_11        // SAME AS PWM SO NO MAPPING
#define ADC_SAMPLE_COUNT 5 

// PWM/Timer configuration
#define PWM_SPEED_MODE LEDC_LOW_SPEED_MODE
#define PWM_TIMER LEDC_TIMER_0
#define PWM_FREQ 1000
#define PWM_RES LEDC_TIMER_11_BIT           // 0 - 2047
#define PWM_CLK_CFG LEDC_AUTO_CLK

// RGB LED task configuration
static TaskHandle_t rgb_led_task_handle;
#define RGB_LED_TASK_STACK_DEPTH 2048
#define RGB_LED_TASK_PRIO 1
#define RGB_LED_TASK_DELAY 50               // 50 ms

// Error queue task configuration 
static TaskHandle_t err_task_handle;
#define ERR_TASK_STACK_DEPTH 2048
#define ERR_TASK_PRIO RGB_LED_TASK_PRIO + 1

// Logging configuration
static const char *TAG = "PWM_MULTI-CHANNEL (ESP-IDF)";

// Error queue configuration
static QueueHandle_t err_queue;
#define ERR_QUEUE_SIZE 10
#define ERR_QUEUE_ITEM esp_err_t

// Function prototypes
static void halt_program(void);
static void queue_error(const char *source, esp_err_t err);
static void init_pwm(void);
static void init_pwm_channel(ledc_channel_t channel, gpio_num_t gpio_num);
static void init_adc(void);
static void rgb_led_task(void *pvParameters);
static void err_queue_task(void *pvParameters);

void app_main() {
    // Initialize error queue
    err_queue = xQueueCreate(ERR_QUEUE_SIZE, sizeof(ERR_QUEUE_ITEM));
    if (err_queue == NULL) halt_program();

    init_pwm();
    init_adc();

    BaseType_t ret;

    ret = xTaskCreate(
        rgb_led_task,
        "rgb_led_task",
        RGB_LED_TASK_STACK_DEPTH,
        NULL,
        RGB_LED_TASK_PRIO,
        &rgb_led_task_handle
    );
    if (ret != pdTRUE) halt_program();

    ret = xTaskCreate(
        err_queue_task,
        "err_queue_task",
        ERR_TASK_STACK_DEPTH,
        NULL,
        ERR_TASK_PRIO,
        &err_task_handle
    );
    if (ret != pdTRUE) halt_program();
}

static void halt_program(void) {
    ESP_LOGW(TAG, "Halting program.");

    while (1) {
        ESP_LOGW(TAG, "System Error! Please update source code & reflash.");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

static void queue_error(const char *source, esp_err_t err) {
    if (err == ESP_OK) return;

    BaseType_t queue_send_status = pdFALSE;
    int32_t queue_send_retry_counter = 10;

    while (queue_send_status != pdTRUE && queue_send_retry_counter >= 0) {
        ESP_LOGE(TAG, "Error in %s: %s", source, esp_err_to_name(err));
        queue_send_status = xQueueSend(err_queue, &err, 0);
        if (queue_send_status != pdTRUE) {
            ESP_LOGW(TAG, "Queue send failed. Re-attempting %ld times.", queue_send_retry_counter);
            queue_send_retry_counter--;
        }
    }

    if (queue_send_status != pdTRUE) {
        ESP_LOGW(TAG, "Queue send failed");
    }
}

static void init_pwm(void) {
    // Timer config
    ledc_timer_config_t timer_config = {
        .speed_mode = PWM_SPEED_MODE,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQ,
        .duty_resolution = PWM_RES,
        .clk_cfg = PWM_CLK_CFG,
    };
    queue_error("ledc_timer_config", ledc_timer_config(&timer_config));

    // RGB channel config
    init_pwm_channel(RED_PWM_CHANNEL, RED_LED);
    init_pwm_channel(GREEN_PWM_CHANNEL, GREEN_LED);
    init_pwm_channel(BLUE_PWM_CHANNEL, BLUE_LED);
}

static void init_pwm_channel(ledc_channel_t channel, gpio_num_t gpio_num) {
    ledc_channel_config_t pwm_channel_config = {
        .channel = channel,
        .gpio_num = gpio_num,
        .timer_sel = PWM_TIMER,
        .speed_mode = PWM_SPEED_MODE,
        .duty = 0,
        .hpoint = 0,
    };
    queue_error("ledc_channel_config", ledc_channel_config(&pwm_channel_config));
}

static void init_adc(void) {
    adc_oneshot_unit_init_cfg_t adc_init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    queue_error("adc_oneshot_new_unit", adc_oneshot_new_unit(&adc_init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t adc_oneshot_chan_cfg = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };

    // Red ADC channel
    queue_error("adc_oneshot_config_channel", adc_oneshot_config_channel(
        adc1_handle,
        RED_ADC_CHANNEL,
        &adc_oneshot_chan_cfg
    ));

    // Green ADC channel
    queue_error("adc_oneshot_config_channel", adc_oneshot_config_channel(
        adc1_handle,
        GREEN_ADC_CHANNEL,
        &adc_oneshot_chan_cfg
    ));

    // Blue ADC channel
    queue_error("adc_oneshot_config_channel", adc_oneshot_config_channel(
        adc1_handle,
        BLUE_ADC_CHANNEL,
        &adc_oneshot_chan_cfg
    ));
}

static void rgb_led_task(void *pvParameters) {
    while (1) {
        int red_value = 0, green_value = 0, blue_value = 0;
        int sampled_red_value = 0, sampled_green_value = 0, sampled_blue_value = 0;
        int valid_samples = 0;

        // Read RGB
        for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
            queue_error("adc_oneshot_read", adc_oneshot_read(adc1_handle, RED_ADC_CHANNEL, &sampled_red_value));
            queue_error("adc_oneshot_read", adc_oneshot_read(adc1_handle, GREEN_ADC_CHANNEL, &sampled_green_value));
            queue_error("adc_oneshot_read", adc_oneshot_read(adc1_handle, BLUE_ADC_CHANNEL, &sampled_blue_value));

            if (sampled_red_value < 0 || sampled_red_value > 2047 ||
                sampled_green_value < 0 || sampled_green_value > 2047 ||
                sampled_blue_value < 0 || sampled_blue_value > 2047
            ) {
                ESP_LOGW(TAG, "Unexpected ADC reading: %d", sampled_red_value);
                continue;  // Skip this sample
            }

            red_value += sampled_red_value;
            green_value += sampled_green_value;
            blue_value += sampled_blue_value;
            valid_samples++;

            // Small delay
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        if (valid_samples > 0) {
            red_value /= valid_samples, green_value /= valid_samples, blue_value /= valid_samples;
        }

        // Clamp 
        red_value = (red_value > 2047) ? 2047 : red_value;
        green_value = (green_value > 2047) ? 2047 : green_value;
        blue_value = (blue_value > 2047) ? 2047 : blue_value;

        // Set RGB
        queue_error("ledc_set_duty", ledc_set_duty(PWM_SPEED_MODE, RED_PWM_CHANNEL, red_value));
        queue_error("ledc_set_duty", ledc_set_duty(PWM_SPEED_MODE, GREEN_PWM_CHANNEL, green_value));
        queue_error("ledc_set_duty", ledc_set_duty(PWM_SPEED_MODE, BLUE_PWM_CHANNEL, blue_value));

        // Update RGB
        queue_error("ledc_update_duty", ledc_update_duty(PWM_SPEED_MODE, RED_PWM_CHANNEL));
        queue_error("ledc_update_duty", ledc_update_duty(PWM_SPEED_MODE, GREEN_PWM_CHANNEL));
        queue_error("ledc_update_duty", ledc_update_duty(PWM_SPEED_MODE, BLUE_PWM_CHANNEL));
        
        vTaskDelay(pdMS_TO_TICKS(RGB_LED_TASK_DELAY));
    }
}

static void err_queue_task(void *pvParameters) {
    while (1) {
        esp_err_t err;

        if (xQueueReceive(err_queue, &err, portMAX_DELAY) == pdTRUE) {
            // Clean main task
            if (rgb_led_task_handle != NULL) {
                vTaskDelete(rgb_led_task_handle);
                rgb_led_task_handle = NULL;
                ESP_LOGW(TAG, "Main task deleted due to error");
            }

            // Clean ADC
            if (adc1_handle != NULL) {
                adc_oneshot_del_unit(adc1_handle);
                adc1_handle = NULL;
                ESP_LOGW(TAG, "ADC unit reset due to error");
            }

            // Stop PWM
            ledc_stop(PWM_SPEED_MODE, RED_PWM_CHANNEL, 0);
            ledc_stop(PWM_SPEED_MODE, GREEN_PWM_CHANNEL, 0);
            ledc_stop(PWM_SPEED_MODE, BLUE_PWM_CHANNEL, 0);

            // Halt program
            halt_program();
        }
    }
}