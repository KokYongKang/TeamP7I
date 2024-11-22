#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "ultrasonic.h"
#include "FreeRTOS.h"
#include "task.h"

// Define GPIO pins for ultrasonic sensor
#define TRIG_PIN 4              // Ultrasonic Trigger pin
#define ECHO_PIN 5              // Ultrasonic Echo pin

// Ultrasonic sensor parameters
#define MAX_ECHO_TIME 40000         // Timeout in microseconds (~5 meters range)
#define SPEED_OF_SOUND_CM_US 0.0343 // Speed of sound in cm/us
#define NUM_DISTANCE_SAMPLES 5

// Ultrasonic sensor, for filtering distance readings (moving average)
int distanceSamples[NUM_DISTANCE_SAMPLES] = {0};
int distanceSampleIndex = 0;

// Function to initialize the ultrasonic sensor
void ultrasonic_init(){
    gpio_init(TRIG_PIN);
    gpio_init(ECHO_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
}

// Function to trigger the ultrasonic sensor
void ultrasonic_trigger(){
    gpio_put(TRIG_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1)); // Trigger pulse duration
    gpio_put(TRIG_PIN, 0);
}

// Function to measure distance with timeout handling
int ultrasonic_distance(){
    ultrasonic_trigger();
    uint64_t start = time_us_64();
    while (gpio_get(ECHO_PIN) == 0) {
        if (time_us_64() - start > MAX_ECHO_TIME) {
            return -1;  // Timeout: Object out of range
        }
    }

    uint64_t pulse_start = time_us_64();
    while (gpio_get(ECHO_PIN) == 1) {
        if (time_us_64() - pulse_start > MAX_ECHO_TIME) {
            return -1;  // Timeout: Object out of range
        }
    }
    uint64_t pulse_end = time_us_64();
    uint64_t pulse_duration = pulse_end - pulse_start;

    return (int)((pulse_duration * SPEED_OF_SOUND_CM_US) / 2);
}

// Function to get filtered distance with averaging
int ultrasonic_filtered_distance(){
    int distance = ultrasonic_distance();

    if (distance == -1) {
        return -1;  // Return -1 if the object is out of range
    }

    distanceSamples[distanceSampleIndex] = distance;
    distanceSampleIndex = (distanceSampleIndex + 1) % NUM_DISTANCE_SAMPLES;

    int sum = 0;
    for (int i = 0; i < NUM_DISTANCE_SAMPLES; i++) {
        sum += distanceSamples[i];
    }
    return sum / NUM_DISTANCE_SAMPLES;
}