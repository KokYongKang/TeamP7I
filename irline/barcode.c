#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>

#define IR_BARCODE_DIGITAL_PIN 12 // Detect edge (D0 -> GPI12)
#define IR_BARCODE_ANALOG_PIN 26  // Read ADC (A0 -> GPIO26)
#define MAX_BUFFER_SIZE 1024      // large buffer size
#define DELAY_MS 10               // Debounce delay in milliseconds
#define INACTIVE_TIMEOUT_MS 3000  // Timeout duration for inactivity reset
#define WHITE_STRIP_THRESHOLD_US 1000
#define LINE_FOLLOWING_PIN 27 // ADC for line-following IR sensor
#define LINE_THRESHOLD 1000   // Threshold ADC value to detect line
#define DEBOUNCE_DELAY_FAST_MS 5
#define DEBOUNCE_DELAY_SLOW_MS 15

// Motor control pins
#define R_MOTOR_IN3 8
#define R_MOTOR_IN4 9
#define R_MOTOR_ENA 1
#define L_MOTOR_IN1 2
#define L_MOTOR_IN2 3
#define L_MOTOR_ENB 0

typedef enum
{
    MEASURING_BLACK,
    MEASURING_WHITE
} MeasureState;
MeasureState measure_state = MEASURING_BLACK;

// Global variables
volatile absolute_time_t start_time;
volatile int pulse_width_us = 0;
int pulse_widths[MAX_BUFFER_SIZE];
int pulse_index = 0;
int binary_index = 0;
char binary_pattern[MAX_BUFFER_SIZE];
uint16_t adc_value;

volatile uint32_t last_interrupt_time = 0;

// Function declarations
void initialize_motors();
void control_motors(int left_speed, int right_speed);
void reset_scanning();
void gpio_callback(uint gpio, uint32_t events);
void analyze_and_compile_pattern();
int decode_binary_pattern(const char *pattern);
void reverse_binary_pattern(char *original, char *reversed, int length);
void shift_binary_pattern_right(char *pattern, int length);

// Code 39 table: Binary patterns mapped to characters
typedef struct
{
    char character;
    const char *pattern;
} Code39Char;

Code39Char code39_table[] = {
    {'0', "1110101011101010"},
    {'1', "1010111010101110"},
    {'2', "1110101110101010"},
    {'3', "1010111110101010"},
    {'4', "1010101011101110"},
    {'5', "1110101011101010"},
    {'6', "1010111011101010"},
    {'7', "1010101110101110"},
    {'8', "1110101010101110"},
    {'9', "1010111010101110"},
    {'A', "1011101000101110"},
    {'B', "1110101000101110"},
    {'C', "1110111010001010"},
    {'D', "1010111000101110"},
    {'E', "1110101110001010"},
    {'F', "1011101110001010"},
    {'G', "1010100011101110"},
    {'H', "1110101000111010"},
    {'I', "1011101000111010"},
    {'J', "1010111000111010"},
    {'K', "1110101010001110"},
    {'L', "1011101010001110"},
    {'M', "1110111010100010"},
    {'N', "1010111010001110"},
    {'O', "1110101110100010"},
    {'P', "1011101110100010"},
    {'Q', "1010101110001110"},
    {'R', "1110101011100010"},
    {'S', "1011101011100010"},
    {'T', "1010111011100010"},
    {'U', "1110001010101110"},
    {'V', "1000111010101110"},
    {'W', "1110001110101010"},
    {'X', "1000101110101110"},
    {'Y', "1110001011101010"},
    {'Z', "1000111011101010"},
    {'*', "1000101110111010"}};

    // Function to reverse the binary pattern
    void
    reverse_binary_pattern(char *original, char *reversed, int length)
{
    for (int i = 0; i < length; i++)
    {
        reversed[i] = original[length - i - 1];
    }
    reversed[length] = '\0'; // Null-terminate the reversed string
}

// Decodes the binary pattern and prints the final decoded value as a string
// Returns 1 if successful, 0 if decoding fails (i.e., if any unknown segments are found)
int decode_binary_pattern(const char *pattern)
{
    char decoded_string[MAX_BUFFER_SIZE / 16]; // Store decoded result
    int decoded_index = 0;
    int successful_decode = 1;

    // Loop through the binary pattern in chunks of 16 bits
    for (int i = 0; i < strlen(pattern); i += 16)
    {
        char segment[17]; // Temporary buffer for 16-bit segment
        strncpy(segment, &pattern[i], 16);
        segment[16] = '\0'; // Null-terminate

        // Search the Code 39 table for a matching pattern
        int found = 0;
        for (int j = 0; j < sizeof(code39_table) / sizeof(Code39Char); j++)
        {
            if (strcmp(segment, code39_table[j].pattern) == 0)
            {
                decoded_string[decoded_index++] = code39_table[j].character;
                found = 1;
                break;
            }
        }

        // If no match was found, it may indicate an error in the decoding process
        if (!found)
        {
            decoded_string[decoded_index++] = '?'; // Mark as unknown
            successful_decode = 0;
        }
    }

    decoded_string[decoded_index] = '\0'; // Null-terminate the final decoded string
    printf("Decoded String: %s\n", decoded_string);

    return successful_decode;
}

// Function to shift the binary pattern by one position to the right for reversing
void shift_binary_pattern_right(char *pattern, int length)
{
    if (length <= 1)
        return; // No need to shift if length is 1 or less

    char last = pattern[length - 1]; // Save the last element

    // Shift all elements to the right by one
    for (int i = length - 1; i > 0; i--)
    {
        pattern[i] = pattern[i - 1];
    }

    // Move the last element to the first position
    pattern[0] = last;
}

void analyze_and_compile_pattern()
{
    int total_pulse_width = 0;
    int count = pulse_index;

    // Calculate total pulse width and find maximum pulse width
    int max_pulse_width = 0;
    for (int i = 0; i < count; i++)
    {
        total_pulse_width += pulse_widths[i];
        if (pulse_widths[i] > max_pulse_width)
        {
            max_pulse_width = pulse_widths[i];
        }
    }

    // Avoid division by zero
    if (count > 0)
    {
        int average_pulse_width = total_pulse_width / count;
        binary_index = 0; // Reset binary index for new pattern

        // Compile the binary pattern
        for (int i = 0; i < count; i++)
        {
            // Check for valid pulse width before adding to binary pattern
            if (pulse_widths[i] > 0)
            {
                if (pulse_widths[i] < average_pulse_width / 2)
                {
                    // Short pulses indicate black bars
                    binary_pattern[binary_index++] = (i % 2 == 0) ? '1' : '0';
                }
                else
                {
                    // Long pulses indicate white spaces
                    strncpy(&binary_pattern[binary_index], (i % 2 == 0) ? "111" : "000", 3);
                    binary_index += 3;
                }
            }
        }

        // Null-terminate the binary pattern
        binary_pattern[binary_index] = '\0';

        // Print and decode when binary index reaches a multiple of 48
        if (binary_index >= 48)
        {
            printf("Binary Pattern: %s\n", binary_pattern);

            // Attempt to decode the binary pattern
            int decoded_successfully = decode_binary_pattern(binary_pattern);

            // If decoding fails, try reversing the binary pattern and decode again
            if (!decoded_successfully)
            {
                printf("Decoding failed. Attempting with reversed binary pattern...\n");
                char reversed_pattern[MAX_BUFFER_SIZE];
                reverse_binary_pattern(binary_pattern, reversed_pattern, binary_index);

                // Try decoding the reversed pattern
                if (!decode_binary_pattern(reversed_pattern))
                {
                    printf("Decoding failed even after reversing the binary pattern.\n");
                }
            }

            // Reset for next pattern
            binary_index = 0;
        }
    }

    // Reset pulse index after processing
    pulse_index = 0;
}

// Reset scanning state
void reset_scanning()
{
    pulse_index = 0;
    binary_index = 0;
    memset(pulse_widths, 0, sizeof(pulse_widths));
    memset(binary_pattern, 0, sizeof(binary_pattern));
    printf("Resetting scanning due to inactivity.\n");
    start_time = get_absolute_time();
    measure_state = MEASURING_BLACK;
}

// GPIO interrupt handler
void gpio_callback(uint gpio, uint32_t events)
{
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Debounce check
    if (current_time - last_interrupt_time < DELAY_MS)
    {
        return;
    }
    last_interrupt_time = current_time;

    // Handle edge events
    if (events & GPIO_IRQ_EDGE_RISE)
    {
        if (measure_state == MEASURING_WHITE)
        {
            absolute_time_t end_time = get_absolute_time();
            pulse_width_us = absolute_time_diff_us(start_time, end_time);
            pulse_widths[pulse_index++] = pulse_width_us;
            measure_state = MEASURING_BLACK;
            start_time = get_absolute_time();
        }

        // 1st contact with black strip, start timer
        if (measure_state == MEASURING_BLACK && pulse_width_us == 0)
        {
            start_time = get_absolute_time();
        }
    }
    else if (events & GPIO_IRQ_EDGE_FALL)
    {
        if (measure_state == MEASURING_BLACK)
        {
            absolute_time_t end_time = get_absolute_time();
            pulse_width_us = absolute_time_diff_us(start_time, end_time);
            pulse_widths[pulse_index++] = pulse_width_us;
            measure_state = MEASURING_WHITE;
            start_time = get_absolute_time();
        }

        // 1st contact with black strip, start timer
        if (measure_state == MEASURING_WHITE && pulse_width_us == 0)
        {
            start_time = get_absolute_time();
        }
    }

    // for debugging
    printf("%d) Pulse width: %d [%s], Event: %d \n", pulse_index, pulse_width_us, (measure_state) ? "BLACK" : "WHITE");
    pulse_width_us = 0;

    // Analyze and compile the pattern when buffer reaches its limit
    if (pulse_index >= 30)
    {
        analyze_and_compile_pattern();
    }
}

void follow_line()
{
    adc_select_input(1); // Select the line-following ADC pin (GPIO 27)
    uint16_t line_sensor_value = adc_read();

    int error = LINE_THRESHOLD - line_sensor_value;
    int base_speed = 70; // Base speed for both motors

    // Adjust speeds based on the error
    int left_speed = base_speed - error * 0.1; // Adjust sensitivity as needed
    int right_speed = base_speed + error * 0.1;

    // Keep speeds within allowable range (e.g., 0 to 100)
    if (left_speed < 0)
        left_speed = 0;
    if (left_speed > 100)
        left_speed = 100;
    if (right_speed < 0)
        right_speed = 0;
    if (right_speed > 100)
        right_speed = 100;

    control_motors(left_speed, right_speed);
}

void initialize_motors()
{
    gpio_init(R_MOTOR_IN3);
    gpio_init(R_MOTOR_IN4);
    gpio_init(R_MOTOR_ENA);
    gpio_init(L_MOTOR_IN1);
    gpio_init(L_MOTOR_IN2);
    gpio_init(L_MOTOR_ENB);

    gpio_set_dir(R_MOTOR_IN3, GPIO_OUT);
    gpio_set_dir(R_MOTOR_IN4, GPIO_OUT);
    gpio_put(L_MOTOR_IN2, 1);
    gpio_put(R_MOTOR_IN3, 0);
    gpio_put(R_MOTOR_IN4, 1);
}

int main()
{
    stdio_init_all();
    adc_init();
    initialize_motors();

    // Set up digital input for barcode IR sensor
    gpio_init(IR_BARCODE_DIGITAL_PIN);
    gpio_set_dir(IR_BARCODE_DIGITAL_PIN, GPIO_IN);
    gpio_pull_down(IR_BARCODE_DIGITAL_PIN);
    gpio_set_irq_enabled_with_callback(IR_BARCODE_DIGITAL_PIN,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &gpio_callback);

    // Set up analog input for barcode IR sensor
    adc_gpio_init(IR_BARCODE_ANALOG_PIN);
    adc_select_input(0);

    // Set up analog input for line-following IR sensor
    adc_gpio_init(LINE_FOLLOWING_PIN);
    gpio_set_dir(R_MOTOR_ENA, GPIO_OUT);
    gpio_set_dir(L_MOTOR_IN1, GPIO_OUT);
    gpio_set_dir(L_MOTOR_IN2, GPIO_OUT);
    gpio_set_dir(L_MOTOR_ENB, GPIO_OUT);

    while (true)
    {
        follow_line(); // Line-following function

        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        adc_value = adc_read();
        printf("Analog Value: %d\n", adc_value);

        // Check for inactivity and reset if timed out
        if (current_time - last_interrupt_time >= INACTIVE_TIMEOUT_MS)
        {
            reset_scanning();
            last_interrupt_time = current_time; // Update to prevent repeated resets
        }

        // Check if this last white pulse width exceeds the threshold
        if (measure_state == MEASURING_WHITE &&
            (current_time - last_interrupt_time >= 2500) &&
            pulse_index == 29)
        {
            pulse_widths[pulse_index++] = WHITE_STRIP_THRESHOLD_US;
            printf("30) Excess white strip detected; inserting '0' into binary pattern.\n");
            analyze_and_compile_pattern();
        }

        sleep_ms(100); // Adjust as necessary
    }

    return 0;
}

void control_motors(int left_speed, int right_speed)
{
    gpio_put(0, 1); // Enable right motor
    gpio_put(1, 1); // Enable left motor
    if (left_speed > right_speed)
    {
        gpio_put(L_MOTOR_IN1, 1);
        gpio_put(L_MOTOR_IN2, 0);
        gpio_put(R_MOTOR_IN3, 1);
        gpio_put(R_MOTOR_IN4, 0);
    }
    else if (right_speed > left_speed)
    {
        gpio_put(L_MOTOR_IN1, 0);
        gpio_put(L_MOTOR_IN2, 1);
        gpio_put(R_MOTOR_IN3, 1);
        gpio_put(R_MOTOR_IN4, 0);
    }
    else
    {
        // Both motors at same speed
        gpio_put(L_MOTOR_IN1, 1);
        gpio_put(L_MOTOR_IN2, 0);
        gpio_put(R_MOTOR_IN3, 1);
        gpio_put(R_MOTOR_IN4, 0);
    }
}