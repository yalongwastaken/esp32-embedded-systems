// Author: Anthony Yalong
// Description: UART command parser for ESP32-WROOM-32E that provides serial interface
//              for LED control, system monitoring, and diagnostics. Supports multi-parameter
//              commands with real-time statistics tracking and system information display.
// Improvements:
//     - Add error management & backup LED value initialization
//     - Use char[] instead of String

#include <Arduino.h>

// Hardware configuration
#define BAUDRATE 115200
#define UART_TIMEOUT 10000
#define INTERNAL_LED_PIN 2    // esp32-wroom-32e onboard LED
#define EXTERNAL_LED_PIN 4

// Command configuration
#define MAX_COMMANDS 5
#define MAX_TOKENS MAX_COMMANDS + 1

// Command structure
typedef struct {
    String name;
    int param_count;
    String params[MAX_COMMANDS];       // Maximum 5 parameters/command
} command_t;

// Statistics structure
typedef struct {
    unsigned long total_messages;
    unsigned long total_char;
    unsigned long total_bytes;
    unsigned long start_time;
} statistics_t;
statistics_t statistics_manager;

// Function prototypes
void init_gpio(void);
void init_stats(void);
command_t parse_command(String input);
void execute_command(command_t cmd);
void command_led(command_t cmd);
void command_stats(void);
void command_reset(void);
void command_info(void);
void command_help(void);
void update_stats(String input);

void setup() {
    // Initialize UART
    Serial.begin(BAUDRATE);
    while (!Serial) delay(10);
    Serial.setTimeout(UART_TIMEOUT);

    // Initialize gpio
    init_gpio();
    init_stats();

    // Initial message
    Serial.println("===ESP32 Command Parser===");
    command_help();
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() > 0) {
            command_t cmd = parse_command(input);
            execute_command(cmd);
        }
    }
}

void init_gpio(void) {
    // Internal LED
    if (!GPIO_IS_VALID_OUTPUT_GPIO(INTERNAL_LED_PIN)) {
        // TODO: Add error management & backup value
        return;
    }

    pinMode(INTERNAL_LED_PIN, OUTPUT);
    digitalWrite(INTERNAL_LED_PIN, LOW);

    // External LED
    if (!GPIO_IS_VALID_OUTPUT_GPIO(EXTERNAL_LED_PIN)) {
        // TODO: Add error management & backup value
        return;
    }

    pinMode(EXTERNAL_LED_PIN, OUTPUT);
    digitalWrite(EXTERNAL_LED_PIN, LOW);
}

void init_stats(void) {
    statistics_manager.start_time = millis();
    statistics_manager.total_bytes = 0;
    statistics_manager.total_char = 0;
    statistics_manager.total_messages = 0;
}

command_t parse_command(String input) {
    command_t cmd;
    cmd.param_count = 0;

    update_stats(input);
    input.toUpperCase();

    int token_count = 0;
    String tokens[MAX_TOKENS];

    int start = 0;
    int space = input.indexOf(' ');

    // First token
    if (space == -1) {
        tokens[token_count] = input;
        token_count++;
    }
    else {
        tokens[token_count] = input.substring(start, space);
        token_count++;
    }

    // Remaining tokens
    while (space != -1 && token_count < MAX_TOKENS) {
        start = space + 1;
        space = input.indexOf(' ', start);

        if (space == -1) {
            tokens[token_count] = input.substring(start);
            token_count++;
        }
        else {
            tokens[token_count] = input.substring(start, space);
            token_count++;
        }
    }

    // Populate cmd object
    if (token_count > 0) {
        cmd.name = tokens[0];
        cmd.param_count = token_count - 1;

        for (int i = 0; i < cmd.param_count && i < 5; i++) {
            cmd.params[i] = tokens[i+1];
        }
    }

    return cmd;
}

void execute_command(command_t cmd) {
    Serial.printf("Executing: %s\n", cmd.name);
    if (cmd.param_count > 0) {
        Serial.print("  Parameters: ");

        for (int i = 0; i< cmd.param_count; i++) {
            Serial.print(cmd.params[i]);
            if (i < cmd.param_count - 1) Serial.print(", ");
        }
    }
    Serial.println();

    // Process command
    if (cmd.name == "LED") {
        command_led(cmd);
    }
    else if (cmd.name == "STATS") {
        command_stats();
    }
    else if (cmd.name == "RESET") {
        command_reset();
    }
    else if (cmd.name == "INFO") {
        command_info();
    }
    else if (cmd.name == "HELP") {
        command_help();
    }
    else {
        Serial.println("ERROR: unknown command! Type `HELP` for available commands.");
    }

    Serial.println();
}

void command_led(command_t cmd) {
    if (cmd.param_count < 1) {
        Serial.println("ERROR: LED command requires a parameter");
        Serial.println("Type `HELP` for LED command usage");
        return;
    }

    String operation = cmd.params[0];

    // Process target parameter
    String target = (cmd.param_count > 1) ? cmd.params[1] : "INTERNAL";
    if (target != "INTERNAL" && target != "EXTERNAL") {
        Serial.printf("ERROR: invalid parameter: `%s`\n", target);
        Serial.println("Type `HELP` for LED command usage");
        return;
    }
    int pin = (target == "INTERNAL") ? INTERNAL_LED_PIN : EXTERNAL_LED_PIN;

    if (operation == "ON") {
        // Process brightness parameter
        int brightness = (cmd.param_count > 2) ? cmd.params[2].toInt() : 255;
        if (brightness < 0 || brightness > 255) {
            Serial.printf("ERROR: invalid parameter: `%d`\n", brightness);
            Serial.println("Type `HELP` for LED command usage");
            return;
        }
        else {
            analogWrite(pin, brightness);

            Serial.printf("SUCCESS: %s LED turned on (Brightness: %d)\n", target, brightness);
        }
    }
    else if (operation == "OFF") {
        analogWrite(pin, LOW);

        Serial.printf("SUCCESS: %s LED turned off\n", target);
    }
    else {
        Serial.println("ERROR: invalid parameter");
        Serial.println("Type `HELP` for LED command usage");
        return;
    } 
}

void command_stats(void) {
    unsigned long runtime = millis() - statistics_manager.start_time;
    Serial.println("===Statistics===");
    Serial.printf("Total Messages: %lu\n", statistics_manager.total_messages);
    Serial.printf("Runtime: %0.2f\n", runtime / 1000.0);
    Serial.printf("Total Bytes: %lu\n", statistics_manager.total_bytes);
    Serial.printf("Total Chars: %lu\n", statistics_manager.total_char);
    Serial.println("================\n");
}

void command_reset(void) {
    statistics_manager.start_time = millis();
    statistics_manager.total_bytes = 0;
    statistics_manager.total_char = 0;
    statistics_manager.total_messages = 0;

    Serial.println("===Reset===");
    Serial.println("Reset all statistics\n");
    Serial.println("===========\n");
}

void command_info(void) {
    Serial.println("\n===SYSTEM INFORMATION===");
    Serial.printf("Chip Model: %s\n", ESP.getChipModel());
    Serial.printf("Chip Revision: %d\n", ESP.getChipRevision());
    Serial.printf("CPU Frequency: %lu MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Flash Size: %0.2f MB\n", ESP.getFlashChipSize() / (1024 * 1024));
    Serial.printf("Flash Speed: %0.2f MHz\n", ESP.getFlashChipSpeed() / (1000 * 1000));
    Serial.printf("SDK Version: %s\n", ESP.getSdkVersion());
    Serial.println("==========================\n");
} 

void command_help(void) {
    Serial.println("\n===AVAILABLE COMMANDS===");
    Serial.println("LED ON|OFF [INTERNAL|EXTERNAL] [brightness]");
    Serial.println("    - control LED state & brightness (0-255)");
    Serial.println("STATS");
    Serial.println("    - Display current runtime stats");
    Serial.println("RESET");
    Serial.println("    - Reset current runtime stats");
    Serial.println("INFO");
    Serial.println("-   Display detailed system information");
    Serial.println("HELP");
    Serial.println("    - Display this help message");
    Serial.println("==========================\n");
}

void update_stats(String input) {
    int len = input.length();

    statistics_manager.total_messages++;
    statistics_manager.total_char += len;
    statistics_manager.total_bytes += len + 1;  // +1 for newline
}
