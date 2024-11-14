#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "encoder.h"
#include "../motor/motor.h"

// Define GPIO pins for encoder control
#define RIGHT_ENCODER_PIN 7     // encoder for right motor
#define LEFT_ENCODER_PIN 6      // encoder for left motor

// Encoder parameters
#define ENCODER_DISC_SLOTS 20   // 20 slots in an optical encoder disc
#define WHEEL_DIAMETER_CM 6.3   // car wheel diameter (cm)

// encoder timer parameter
struct repeating_timer encoder_timer;

// Calculate wheel circumference (cm)
const float WHEEL_CIRCUMFERENCE_CM = 3.14159 * WHEEL_DIAMETER_CM;  // 19.792017 cm

// Distance per encoder disc slot
const float CM_PER_SLOT = WHEEL_CIRCUMFERENCE_CM / ENCODER_DISC_SLOTS; // 0.989601 cm

// Encoder pulse counters
volatile uint32_t right_pulse_count = 0;  // Store pulse counts from the encoder
volatile uint32_t right_last_pulse_count = 0;  // Store pulse count from the last interval
volatile uint32_t left_pulse_count = 0;  // Store pulse counts from the encoder
volatile uint32_t left_last_pulse_count = 0;  // Store pulse count from the last interval

// Distance variables
float right_total_distance = 0.0f; // Distance (cm)
float left_total_distance = 0.0f; // Distance (cm)

// Function to setup encoders
void encoder_init() {
    // Set up GPIO pins for encoder
    gpio_init(RIGHT_ENCODER_PIN);
    gpio_init(LEFT_ENCODER_PIN);
    gpio_set_dir(RIGHT_ENCODER_PIN, GPIO_IN);
    gpio_set_dir(LEFT_ENCODER_PIN, GPIO_IN);

    // Enable interrupt on rising edges
    gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &encoder_isr_callback);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &encoder_isr_callback);

    // Set up encoder timer (every 250ms)
    add_repeating_timer_ms(-250.0f, encoder_timer_callback, NULL, &encoder_timer);
}

// Function for encoder ISR
void encoder_isr_callback(uint gpio, uint32_t events) {
    if (gpio == RIGHT_ENCODER_PIN) {
        right_pulse_count++;  // Increment pulse count on each rising edge
    }
    if (gpio == LEFT_ENCODER_PIN) {
        left_pulse_count++;  // Increment pulse count on each rising edge
    }
}

// Function for timer callback for calculating speed and distance
bool encoder_timer_callback(struct repeating_timer *t) {
    if (car_moving){
        // Calculate pulse difference
        uint32_t right_pulse_diff = right_pulse_count - right_last_pulse_count;
        uint32_t left_pulse_diff = left_pulse_count - left_last_pulse_count;

        // Calculate speed and total distance
        float right_distance_covered = right_pulse_diff * CM_PER_SLOT;  
        right_pid.current_speed = right_distance_covered * 1000 / 250.0f;  // Speed (cm/s), timer interval is 250ms
        right_total_distance = right_pulse_count * CM_PER_SLOT; // Total distance (cm)
        float left_distance_covered = left_pulse_diff * CM_PER_SLOT;  
        left_pid.current_speed = left_distance_covered * 1000 / 250.0f;  // Speed (cm/s), timer interval is 250ms
        left_total_distance = left_pulse_count * CM_PER_SLOT; // Total distance (cm)

        // Update last pulse count
        right_last_pulse_count = right_pulse_count;
        left_last_pulse_count = left_pulse_count;
    }

    return true; 
}
