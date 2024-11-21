#ifndef ULTRASONIC_H
#define ULTRASONIC_H

// Initialize ultrasonic sensor
void ultrasonic_init();

// Trigger ultrasonic sensor
void ultrasonic_trigger();

// Get distance from ultrasonic sensor with timeout
int ultrasonic_distance();

// Get filtered distance from ultrasonic sensor with averaging
int ultrasonic_filtered_distance();

#endif