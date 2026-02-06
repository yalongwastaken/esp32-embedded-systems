// Author: Anthony Yalong
// Description: Reads potentiometer and light sensor via ADC with voltage calibration

#include "adc_basic_esp.h"

// general logging
const char *TAG = "adc_basic (esp)";

// task handles
TaskHandle_t light_sensor_handle;
TaskHandle_t pot_handle;

// sensor task config
typedef struct {
    char *name;
    adc_channel_t adc_channel;
    TickType_t task_delay;
} sensor_task_config_t;

// ADC handles
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle;
static bool do_calibration = false;

void app_main() {
    esp_err_t ret;
    ret = init_adc();
    if (ret != ESP_OK) halt_program();
    
    // light sensor task
    static sensor_task_config_t light_sensor_task_config = {
        .adc_channel = ADC_LIGHT_CHANNEL,
        .name = "light_sensor",
        .task_delay = pdMS_TO_TICKS(TASK_DELAY),
    };
    
    xTaskCreate(
        log_sensor,
        "log_light_sensor",
        TASK_STACK_DEPTH,
        &light_sensor_task_config,
        TASK_PRIO,
        &light_sensor_handle
    );
    
    // potentiometer task
    static sensor_task_config_t potentiometer_task_config = {
        .adc_channel = ADC_POT_CHANNEL,
        .name = "potentiometer",
        .task_delay = pdMS_TO_TICKS(TASK_DELAY),
    };
    
    xTaskCreate(
        log_sensor,
        "log_potentiometer",
        TASK_STACK_DEPTH,
        &potentiometer_task_config,
        TASK_PRIO,
        &pot_handle
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
    
    // initialize ADC unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT,
    };
    ret = adc_oneshot_new_unit(&init_config, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize ADC unit");
        return ret;
    }
    
    // configure channel settings
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH,
        .atten = ADC_ATTEN,
    };
    
    // configure potentiometer channel
    ret = adc_oneshot_config_channel(adc1_handle, ADC_POT_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to configure potentiometer channel");
        return ret;
    }
    
    // configure light sensor channel
    ret = adc_oneshot_config_channel(adc1_handle, ADC_LIGHT_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to configure light sensor channel");
        return ret;
    }
    
    // setup calibration
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle);
    if (ret == ESP_OK) {
        do_calibration = true;
        ESP_LOGI(TAG, "calibration: line fitting scheme enabled");
    } 
    else if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "calibration: not supported on this chip, using raw values");
        do_calibration = false;
    } 
    else {
        ESP_LOGE(TAG, "calibration: failed to create calibration scheme");
        return ret;
    }
    
    return ESP_OK;
}

void log_sensor(void *pvParameters) {
    sensor_task_config_t *sensor_task_config = (sensor_task_config_t *) pvParameters;
    
    while (true) {
        int raw = 0;
        int voltage = 0;
        
        // read raw value
        esp_err_t ret = adc_oneshot_read(adc1_handle, sensor_task_config->adc_channel, &raw);
        if (ret != ESP_OK) {
            ESP_LOGE(sensor_task_config->name, "failed to read ADC");
            vTaskDelay(sensor_task_config->task_delay);
            continue;
        }
        
        // convert to voltage
        if (do_calibration) {
            ret = adc_cali_raw_to_voltage(adc1_cali_handle, raw, &voltage);
            if (ret == ESP_OK) {
                ESP_LOGI(sensor_task_config->name, "raw: %d, voltage: %d mV", raw, voltage);
            } 
            else {
                ESP_LOGE(sensor_task_config->name, "failed to convert to voltage");
            }
        } 
        else {
            ESP_LOGI(sensor_task_config->name, "raw: %d (no calibration)", raw);
        }
        
        // delay
        vTaskDelay(sensor_task_config->task_delay);
    }
    
    vTaskDelete(NULL);
}