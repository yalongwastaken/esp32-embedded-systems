// Author: Anthony Yalong
// Description: An ESP32 serial tool that echoes user input and provides
//              commands (STATS, RESET, HELP) to track usage statistics.

#include <Arduino.h>
#include <string.h>

#define BAUDRATE 115200
#define INPUT_BUFFER_SIZE 64

const char ECHO_PREFIX[] = "ECHO: "; 

// Serial commands
typedef enum {
    ECHO_CMD_STATS,
    ECHO_CMD_RESET,
    ECHO_CMD_HELP,
    ECHO_CMD_NONE,
} echo_cmd_t;

// Statistics manager
typedef struct {
    unsigned long total_messages;
    unsigned long total_char;
    unsigned long total_bytes;
    unsigned long start_time;
} echo_stats_t;
echo_stats_t echo_stats_manager;

// Input buffer
static char serial_buffer[INPUT_BUFFER_SIZE];

// Function prototypes
void init_echo_stats(echo_stats_t *stats);
echo_cmd_t process_input(const char *in);
void echo_basic(const char *in);
void echo_stats(echo_stats_t *stats);
void echo_reset(echo_stats_t *stats);
void echo_help(void);
void update_stats(echo_stats_t *stats, const char *in);

void setup() {
    init_echo_stats(&echo_stats_manager);

    Serial.begin(BAUDRATE);
    while (!Serial) delay(10);

    Serial.println("===Echo Tool===");
    echo_help();
}

void loop() {
    // Check if any serial data is available
    if (Serial.available() > 0) {
        // Read until newline or buffer full
        size_t len = Serial.readBytesUntil('\n', serial_buffer, INPUT_BUFFER_SIZE - 1);
        serial_buffer[len] = '\0';

        // Trim
        while (len > 0 && (serial_buffer[len-1] == '\r' || serial_buffer[len-1] == ' ')) {
            serial_buffer[--len] = '\0';
        }

        if (len > 0) {
            // Determine command
            echo_cmd_t serial_cmd = process_input(serial_buffer);

            switch (serial_cmd) {
                case ECHO_CMD_NONE:
                    echo_basic(serial_buffer);
                    update_stats(&echo_stats_manager, serial_buffer);
                    break;
                case ECHO_CMD_STATS:
                    echo_stats(&echo_stats_manager);
                    break;
                case ECHO_CMD_RESET:
                    echo_reset(&echo_stats_manager);
                    break;
                case ECHO_CMD_HELP:
                    echo_help();
                    break;
            }
        }
    }
}

void init_echo_stats(echo_stats_t *stats) {
    stats->total_messages = 0;
    stats->total_char = 0;
    stats->total_bytes = 0;
    stats->start_time = millis();
}

echo_cmd_t process_input(const char *in) {
    if (strcasecmp(in, "STATS") == 0) return ECHO_CMD_STATS;
    else if (strcasecmp(in, "RESET") == 0) return ECHO_CMD_RESET;
    else if (strcasecmp(in, "HELP") == 0) return ECHO_CMD_HELP;
    else return ECHO_CMD_NONE;
}

void echo_basic(const char *in) {
    int letters = 0, digits = 0, spaces = 0, special = 0;
    size_t len = strlen(in);

    for (size_t i = 0; i < len; i++) {
        char cur = in[i];
        if (isAlpha(cur)) letters++;
        else if (isDigit(cur)) digits++;
        else if (isSpace(cur)) spaces++;
        else special++;
    }

    Serial.printf(
        "%s'%s' (Letters: %d, Digits: %d, Spaces: %d, Special: %d)\n",
        ECHO_PREFIX, in, letters, digits, spaces, special
    );
}

void echo_stats(echo_stats_t *stats) {
    unsigned long runtime = millis() - stats->start_time;
    Serial.println("===Statistics===");
    Serial.printf("Total Messages: %lu\n", stats->total_messages);
    Serial.printf("Runtime: %0.2f\n", runtime / 1000.0);
    Serial.printf("Total Bytes: %lu\n", stats->total_bytes);
    Serial.printf("Total Chars: %lu\n\n", stats->total_char);
}

void echo_reset(echo_stats_t *stats) {
    stats->start_time = millis();
    stats->total_bytes = 0;
    stats->total_char = 0;
    stats->total_messages = 0;

    Serial.println("===Reset===");
    Serial.println("Reset all statistics\n");
}

void echo_help(void) {
    Serial.println("===Commands===");
    Serial.println("STATS - show statistics");
    Serial.println("RESET - reset statistics");
    Serial.println("HELP - show this help");
    Serial.println("Any other text will be echoed\n");
}

void update_stats(echo_stats_t *stats, const char *in) {
    size_t len = strlen(in);

    stats->total_messages++;
    stats->total_char += len;
    stats->total_bytes += len + 1;  // +1 for newline
}
