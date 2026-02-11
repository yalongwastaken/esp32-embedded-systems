/**
 * @file main.c
 * @author Anthony Yalong
 * @brief FreeRTOS queue demo using a producer-consumer pattern to sample ADC readings and print converted voltages. ESP-IDF framework.
 */

// includes
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

// configuration
#define ADC_TASK_DELAY_MS 500

// adc configuration
#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL ADC_CHANNEL_0
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_BITWIDTH ADC_BITWIDTH_12

// queue configure
#define QUEUE_DEPTH 10
#define QUEUE_TIMEOUT_MS 100

static const char *TAG = "queues_basic (esp-idf)";
static QueueHandle_t event_queue = NULL;
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;
static bool do_calibration = false;

// function prototypes
static esp_err_t adc_init(void);
static void adc_task(void *pvParameters);
static void print_task(void *pvParameters);

void app_main() {
    // error management
    esp_err_t ret;
    BaseType_t xRet;

    // initialize adc
    ret = adc_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize adc");
        while (true) { vTaskDelay(portMAX_DELAY); }
    }

    // create queue
    event_queue = xQueueCreate(QUEUE_DEPTH, sizeof(int));
    if (event_queue == NULL) {
        ESP_LOGE(TAG, "failed to initialize queue");
        while (true) { vTaskDelay(portMAX_DELAY); }
    }

    // create adc task
    xRet = xTaskCreatePinnedToCore(
        adc_task,
        "adc_task",
        2048,
        NULL,
        1,
        NULL,
        1
    );
    if (xRet != pdTRUE) {
        ESP_LOGE(TAG, "failed to create adc task");
        while (true) { vTaskDelay(portMAX_DELAY); }
    }

    // create print task
    xRet = xTaskCreatePinnedToCore(
        print_task,
        "print_task",
        2048,
        NULL,
        1,
        NULL,
        1
    );
    if (xRet != pdTRUE) {
        ESP_LOGE(TAG, "failed to create print task");
        while (true) { vTaskDelay(portMAX_DELAY); }
    }
}

static esp_err_t adc_init(void) {
    // error management
    esp_err_t ret;
    
    // adc unit config 
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT,
    };
    ret = adc_oneshot_new_unit(&unit_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize adc unit");
        return ret;
    }

    // adc channel config
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH
    };
    ret = adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize adc channel");
        return ret;
    }
    
    // adc calibration scheme
    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = ADC_UNIT,
            .chan = ADC_CHANNEL,
            .atten = ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali_handle);

    #elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id = ADC_UNIT,
            .atten = ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_cfg, &adc_cali_handle);

    #else
        ESP_LOGW(TAG, "no calibration scheme supported");
        ret = ESP_ERR_NOT_SUPPORTED;
    #endif

    if (ret == ESP_OK) {
        do_calibration = true;
        ESP_LOGI(TAG, "calibration scheme enabled");
    } 
    else if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "calibration not supported, using raw values");
    } 
    else {
        ESP_LOGE(TAG, "failed to create calibration scheme");
        return ret;
    }

    ESP_LOGI(TAG, "successfully initialized adc");
    return ESP_OK;
}

static void adc_task(void *pvParameters) {
    // error management
    esp_err_t ret;

    int raw_value;
    TickType_t adc_last_wake = xTaskGetTickCount();

    while (true) {
        ret = adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw_value);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "failed to read adc... skipping...");
            vTaskDelayUntil(&adc_last_wake, pdMS_TO_TICKS(ADC_TASK_DELAY_MS));
            continue;
        }

        // enqueue
        if (xQueueSend(event_queue, &raw_value, pdMS_TO_TICKS(QUEUE_TIMEOUT_MS)) != pdPASS) {
            ESP_LOGW(TAG, "queue full! sample dropped!");
        }

        vTaskDelayUntil(&adc_last_wake, pdMS_TO_TICKS(ADC_TASK_DELAY_MS));
    }
}

static void print_task(void *pvParameters) {
    // error management
    esp_err_t ret;

    // initialization
    int received;
    int voltage_mv;
    float voltage;

    while (true) {
        if (xQueueReceive(event_queue, &received, portMAX_DELAY) == pdPASS) {
            if (do_calibration) {
                ret = adc_cali_raw_to_voltage(adc_cali_handle, received, &voltage_mv);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "failed to convert to voltage... skipping...");
                    continue;
                }
                voltage = voltage_mv / 1000.0f;
            } 
            else {
                voltage = (received / 4095.0f) * 3.3f;
            }
            
            // print
            ESP_LOGI(TAG, "raw: %d, voltage: %.1f V", received, voltage);
        }
    }
}