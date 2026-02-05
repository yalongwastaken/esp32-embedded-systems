// Author: Anthony Yalong
// Description:
//    Reads a potentiometer, controls LED brightness via PWM, and displays the
//    brightness percentage on an I2C LCD using non-blocking timing.


#include "i2c-lcd_advanced.h"

// logging
const char *TAG = "i2c-lcd_advanced (arduino)";

// lcd
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLUMNS, LCD_ROWS);
unsigned long lcd_last_wake;

// potentiometer task
int sampling[SAMPLING];
int pot_total;
int pot_index;
unsigned long pot_last_wake;

// lcd task
int led_brightness_percentage; 
int last_displayed_brightness = -1;

void setup() {
  // serial setup
  Serial.begin(BAUDRATE);
  while (!Serial) delay(10);

  // initialize i2c
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_DEFAULT_CLOCK);
  Serial.printf("%s: i2c initialized\n", TAG);

  // initialize lcd
  lcd.init();
  lcd.backlight();
  lcd_last_wake = millis();
  led_brightness_percentage = 0;
  Serial.printf("%s: lcd initialized\n", TAG);

  // initialize adc
  analogReadResolution(ADC_RESOLUTION);
  analogSetAttenuation(ADC_ATTENUATION);
  Serial.printf("%s: adc initialized\n", TAG);
  
  // intialize potentiometer task
  pot_total = 0;
  for (int sample = 0; sample < SAMPLING; sample++) {
    sampling[sample] = analogRead(PIN_POT);
    pot_total += sampling[sample];
    delay(10);
  }
  pot_index = 0;
  pot_last_wake = millis();
  Serial.printf("%s: potentiometer task initialized\n", TAG);

  // initialize pwm
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 0);
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_BIT_RESOLUTION);
  ledcAttachPin(PIN_LED, PWM_CHANNEL);
  Serial.printf("%s: ledc initialized\n", TAG);

  lcd.clear();
  Serial.printf("%s: setup complete\n", TAG);
}

void loop() {
  unsigned long cur_time = millis();

  // potentiometer task
  if (cur_time - pot_last_wake >= POT_TASK_DELAY) {
    // read potentiomter
    pot_total -= sampling[pot_index];
    sampling[pot_index] = analogRead(PIN_POT);
    pot_total += sampling[pot_index];

    // update index
    pot_index = (pot_index + 1) % SAMPLING;

    // determine pot average
    int pot_average = pot_total / SAMPLING;     // note: integer round-off intended

    // map pot average to pwm
    int duty = map(pot_average, 0, 1023, 0, 255);            // note: see resolutions
    duty = constrain(duty, 0, 255);

    // map pwm to percentage
    led_brightness_percentage = map(duty, 0, 255, 0, 100);
    led_brightness_percentage = constrain(led_brightness_percentage, 0, 100);

    // write duty
    ledcWrite(PWM_CHANNEL, duty);

    // update timing
    pot_last_wake = cur_time;
  }

  // lcd update task
  if (cur_time - lcd_last_wake >= LCD_DELAY) {
    if (led_brightness_percentage != last_displayed_brightness) {
      lcd.setCursor(0, 0);
      lcd.printf("LED Brightness: %d%%", led_brightness_percentage);
      last_displayed_brightness = led_brightness_percentage;
    }

    lcd_last_wake = cur_time;
  } 
}