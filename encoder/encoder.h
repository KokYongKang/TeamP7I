#ifndef ENCODER_H
#define ENCODER_H

// Initialize encoders
void encoder_init();

// Encoder ISR to increment pulse count
void encoder_isr_callback(uint gpio, uint32_t events);

// Timer callback for calculating speed and distance
bool encoder_timer_callback(struct repeating_timer *t);

// To allow access from other files
extern float right_total_distance;
extern float left_total_distance;

#endif