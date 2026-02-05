// Author: Anthony Yalong
// Description: Creates a breathing LED effect using ESP-IDF LEDC PWM control.
//              LED fades up and down continuously with 50ms timing intervals.

#include <Arduino.h>
#include <driver/ledc.h>

// Hardware configuration
#define BAUDRATE 115200
#define PWM_GPIO_NUM GPIO_NUM_2 // Built-in LED pin
#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_SPEED_MODE LEDC_LOW_SPEED_MODE
#define PWM_DUTY_RES LEDC_TIMER_10_BIT // 10-bit resolution (0-1023)
#define PWM_FREQ_HZ 1000
#define PWM_TIMER LEDC_TIMER_0
#define PWM_CLK_CONFIG LEDC_AUTO_CLK
#define PWM_MAX 1023 // Max duty for 10-bit resolution
#define PWM_DELAY_MS 50

// PWM LED control states
typedef enum
{
    LED_UP = 0,
    LED_DOWN,
} led_state_t;

// LED control object
typedef struct
{
    unsigned long prev_time; // Timing control
    int duty;                // Current PWM duty cycle
    int delay_ms;            // Update interval
    led_state_t led_state;   // Current fade direction
} pwm_control_t;

// Global control structure
pwm_control_t pwm_control;

// Error management
static bool system_error;

void setup()
{
    // Serial initialization
    Serial.begin(BAUDRATE);

    // Error management init
    system_error = false;
    esp_err_t ret;

    // PWM timer configuration
    ledc_timer_config_t timer_config = {
        .speed_mode = PWM_SPEED_MODE,
        .duty_resolution = PWM_DUTY_RES,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = PWM_CLK_CONFIG,
    };
    ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK)
    {
        Serial.printf("setup: LEDC timer config failed: %s\n", esp_err_to_name(ret));
        system_error = true;
        return;
    }

    // PWM channel configuration
    ledc_channel_config_t channel_config = {
        .gpio_num = PWM_GPIO_NUM,
        .speed_mode = PWM_SPEED_MODE,
        .channel = PWM_CHANNEL,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ret = ledc_channel_config(&channel_config);
    if (ret != ESP_OK)
    {
        Serial.printf("setup: LEDC timer config failed: %s\n", esp_err_to_name(ret));
        system_error = true;
        return;
    }

    // PWM control initialization
    pwm_control.led_state = LED_UP;
    pwm_control.delay_ms = PWM_DELAY_MS;
    pwm_control.prev_time = millis();
    pwm_control.duty = 0;
}

void loop()
{
    if (system_error)
    {
        return;
    }

    unsigned long cur_time = millis();

    if (cur_time - pwm_control.prev_time > pwm_control.delay_ms)
    {
        pwm_control.prev_time = cur_time;

        switch (pwm_control.led_state)
        {
        case LED_UP:
            if (pwm_control.duty < PWM_MAX)
            {
                pwm_control.duty = (pwm_control.duty + 5) > PWM_MAX ? PWM_MAX : pwm_control.duty + 5;
            }
            else
            {
                pwm_control.led_state = LED_DOWN;
            }
            break;

        case LED_DOWN:
            if (pwm_control.duty > 0)
            {
                pwm_control.duty = (pwm_control.duty - 5) < 0 ? 0 : pwm_control.duty - 5;
            }
            else
            {
                pwm_control.led_state = LED_UP;
            }
            break;
        }

        // Update PWM duty cycle
        ledc_set_duty(PWM_SPEED_MODE, PWM_CHANNEL, pwm_control.duty);
        ledc_update_duty(PWM_SPEED_MODE, PWM_CHANNEL);

        static int step_counter = 0;
        if (++step_counter % 20 == 0)
        {
            Serial.printf("loop: Duty: %d, State: %s\n", pwm_control.duty,
                        (pwm_control.led_state == LED_UP) ? "UP" : "DOWN");
        }
    }
}