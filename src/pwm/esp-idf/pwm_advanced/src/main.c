// Author: Anthony Yalong
// Author: Anthony Yalong
// Description: PWM-based LED fade controller using ESP-IDF's LEDC peripheral.
//              Implements continuous fade up/down cycles with error monitoring.

#include <driver/gpio.h>
#include <esp_log.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Queue configuration
#define QUEUE_SIZE 10
#define QUEUE_ITEM_TYPE esp_err_t

// Led configuration
#define LED_NUM GPIO_NUM_2  // esp32-wroom-32e built-in LED

// PWM configuration
#define PWM_SPEED_MODE LEDC_LOW_SPEED_MODE
#define PWM_TIMER LEDC_TIMER_0
#define PWM_RES LEDC_TIMER_12_BIT
#define PWM_FREQ 1000
#define PWM_CLK_CFG LEDC_AUTO_CLK
#define PWM_CHANNEL LEDC_CHANNEL_0
#define MAX_DUTY (1 << PWM_RES) - 1

// Task configuration
#define STACK_DEPTH 2048
#define TASK_PRIO 1
#define MONITOR_PRIO TASK_PRIO + 1

// Fade configuration
#define FADE_TIME 2000 // 2 sec
static SemaphoreHandle_t fade_semaphore;

// Error queue
static QueueHandle_t error_queue;

// Loggng
static TaskHandle_t pwm_task_handle;
static const char *TAG = "PWM_ADVANCED (ESP-IDF)";

// Function prototypes
static void halt_program(void);
static void queue_error(const char *source, esp_err_t err);
static void init_pwm(void);
static bool IRAM_ATTR fade_cb(const ledc_cb_param_t *param, void *arg);
static void main_task(void *pvParameters);
static void error_monitor(void *pvParameters);

void app_main() {
    // Initialize semaphore
    fade_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(fade_semaphore);

    // Initialize queue
    error_queue = xQueueCreate(QUEUE_SIZE, sizeof(QUEUE_ITEM_TYPE));
    if (!error_queue) {
        halt_program();
    }

    BaseType_t ret;

    // Main task creation
    ret = xTaskCreate(main_task, "main_task", STACK_DEPTH, NULL, TASK_PRIO, &pwm_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create main_task! Halting system.");
        halt_program();
    }

    // Error monitor creation
    ret = xTaskCreate(error_monitor, "error_monitor", STACK_DEPTH, NULL, MONITOR_PRIO, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create error_monitor! Halting system.");
        halt_program();
    }
}

static void halt_program(void) {
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGE(TAG, "FATAL ERROR: please update source code & reflash");
    }
}

static void queue_error(const char *source, esp_err_t err) {
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

static void init_pwm(void) {
    // PWM timer
    ledc_timer_config_t timer_config = {
        .speed_mode = PWM_SPEED_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RES,
        .freq_hz = 1000,
        .clk_cfg = PWM_CLK_CFG,
    };
    queue_error("ledc_timer_config_t", ledc_timer_config(&timer_config));

    // PWM channel
    ledc_channel_config_t channel_config = {
        .speed_mode = PWM_SPEED_MODE,
        .gpio_num = LED_NUM,
        .timer_sel = PWM_TIMER,
        .channel = PWM_CHANNEL,
        .duty = 0,
        .hpoint = 0,
    };
    queue_error("ledc_channel_config_t", ledc_channel_config(&channel_config));

    // Fade
    queue_error("ledc_fade_func_install", ledc_fade_func_install(0));

    ledc_cbs_t callbacks = {
        .fade_cb = fade_cb
    };
    queue_error("ledc_cb_register", ledc_cb_register(PWM_SPEED_MODE, PWM_CHANNEL, &callbacks, NULL));

    ESP_LOGI(TAG, "Successfully initialized PWM");
}

static bool fade_cb(const ledc_cb_param_t *param, void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (param->event == LEDC_FADE_END_EVT) {
        xSemaphoreGiveFromISR(fade_semaphore, &xHigherPriorityTaskWoken);
    }
    
    return xHigherPriorityTaskWoken == pdTRUE;
}

static void main_task(void *pvParameters) {
    init_pwm();

    bool fade_status = false;   // up-down tracker

    while (1) {
        if (xSemaphoreTake(fade_semaphore, portMAX_DELAY) == pdTRUE) {
            uint32_t target_duty;

            if (!fade_status) {
                ESP_LOGI(TAG, "Fading UP");
                target_duty = MAX_DUTY;
            }
            else {
                ESP_LOGI(TAG, "Fading DOWN");
                target_duty = 0;
            }
            queue_error("ledc_set_fade_with_time", ledc_set_fade_with_time(PWM_SPEED_MODE, PWM_CHANNEL, target_duty, FADE_TIME));
            queue_error("ledc_fade_start", ledc_fade_start(PWM_SPEED_MODE, PWM_CHANNEL, LEDC_FADE_NO_WAIT));
            fade_status = !fade_status;
        }
    }
}

static void error_monitor(void *pvParameters) {
    esp_err_t err;
    while (1) {
        if (xQueueReceive(error_queue, &err, portMAX_DELAY) == pdTRUE) {
            // Clean main task
            if (pwm_task_handle != NULL) {
                vTaskDelete(pwm_task_handle);
                pwm_task_handle = NULL;
                ESP_LOGW(TAG, "Main task deleted due to error");
            }

            // Halt program
            halt_program();
        } 
    }
}
