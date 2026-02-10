/**
 * @file main.cpp
 * @author Anthony Yalong
 * @brief
 * Demonstrates FreeRTOS priority-based scheduling on the ESP32 using the
 * Arduino framework, with three tasks and an ISR-triggered high-priority task.
 *
 * Shows task preemption, ISR-to-task notification, and starvation of
 * lower-priority tasks under sustained high-priority activity.
 */

#include <Arduino.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// project configuration
#define BAUDRATE            115200
#define LED_TASK_HIGH_PIN   GPIO_NUM_12
#define LED_TASK_MED_PIN    GPIO_NUM_13
#define LED_TASK_LOW_PIN    GPIO_NUM_14
#define BUTTON_PIN          GPIO_NUM_15

// task handles 
static TaskHandle_t high_prio_task_handle;
static TaskHandle_t med_prio_task_handle;
static TaskHandle_t low_prio_task_handle;

// volatile variables
volatile bool high_prio_active;

// function prototypes
void IRAM_ATTR button_isr(void);
void high_prio_task(void *pvParameters);
void med_prio_task(void *pvParameters);
void low_prio_task(void *pvParameters);

void setup() {
  // setup serial
  Serial.begin(BAUDRATE);
  if (!Serial) delay(10);

  // initialize i/o
  pinMode(LED_TASK_HIGH_PIN, OUTPUT);
  digitalWrite(LED_TASK_HIGH_PIN, LOW);
  pinMode(LED_TASK_MED_PIN, OUTPUT);
  digitalWrite(LED_TASK_MED_PIN, LOW);
  pinMode(LED_TASK_LOW_PIN, OUTPUT);
  digitalWrite(LED_TASK_LOW_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // interrupt setup
  high_prio_active = false;
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_isr, FALLING);

  // create tasks
  xTaskCreate(
    high_prio_task,
    "high_prio_task",
    2048,
    NULL,
    3,
    &high_prio_task_handle
  );

  xTaskCreate(
    med_prio_task,
    "med_prio_task",
    2048,
    NULL,
    2,
    &med_prio_task_handle
  );

  xTaskCreate(
    low_prio_task,
    "low_prio_task",
    2048,
    NULL,
    1,
    &low_prio_task_handle
  );
}

void loop() {
  // monitor stack usage every 10 seconds
  vTaskDelay(10000 / portTICK_PERIOD_MS);
  
  Serial.println("\n--- stack usage ---");
  Serial.print("high priority: ");
  Serial.print(uxTaskGetStackHighWaterMark(high_prio_task_handle) * 4);
  Serial.println(" bytes free");
  
  Serial.print("medium priority: ");
  Serial.print(uxTaskGetStackHighWaterMark(med_prio_task_handle) * 4);
  Serial.println(" bytes free");
  
  Serial.print("low priority: ");
  Serial.print(uxTaskGetStackHighWaterMark(low_prio_task_handle) * 4);
  Serial.println(" bytes free\n");
}

void IRAM_ATTR button_isr() {
  high_prio_active = !high_prio_active;
  
  // notify high-priority task from isr
  BaseType_t higher_prio_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(high_prio_task_handle, &higher_prio_task_woken);
  
  // request context switch
  if (higher_prio_task_woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void high_prio_task(void *pvParameters) {
  while (1) {
    // wait for notification from button isr
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    if (high_prio_active) {
        Serial.println("HIGH PRIORITY TASK ACTIVE");
        
        // blink
        for (int i = 0; i < 50; i++) {
            digitalWrite(LED_TASK_HIGH_PIN, HIGH);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            digitalWrite(LED_TASK_HIGH_PIN, LOW);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        
        Serial.println("HIGH PRIORITY TASK COMPLETE");
    }
  }
}

void med_prio_task(void *pvParameters) {
  uint32_t loop_count = 0;
    
  while (1) {
    digitalWrite(LED_TASK_MED_PIN, HIGH);
    vTaskDelay(300 / portTICK_PERIOD_MS);
    
    digitalWrite(LED_TASK_MED_PIN, LOW);
    vTaskDelay(300 / portTICK_PERIOD_MS);
    
    // log
    if (++loop_count % 10 == 0) {
      Serial.printf("medium priority task running (count: %d)\n", loop_count);
    }
  }
}

void low_prio_task(void *pvParameters) {
  uint32_t loop_count = 0;
    
  while (1) {
    digitalWrite(LED_TASK_LOW_PIN, HIGH);
    vTaskDelay(700 / portTICK_PERIOD_MS);
    
    digitalWrite(LED_TASK_LOW_PIN, LOW);
    vTaskDelay(700 / portTICK_PERIOD_MS);
    
    // log
    if (++loop_count % 10 == 0) {
      Serial.printf("low priority task running (count: %d)\n", loop_count);
    }
  }
}