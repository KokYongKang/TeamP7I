#include <stdio.h>
#include <stdint.h>
#include "detector.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#define LINE_FOLLOWING_PIN 27 // ADC for line-following IR sensor 
#define LINE_THRESHOLD 1000 // Threshold ADC value to detect line 

// Motor control pins
#define R_MOTOR_IN3 8
#define R_MOTOR_IN4 9
#define R_MOTOR_ENA 1
#define L_MOTOR_IN1 2
#define L_MOTOR_IN2 3
#define L_MOTOR_ENB 0


void follow_line() { 
   adc_select_input(1); // Select line-following ADC pin (GPIO27)
   uint16_t line_sensor_value = adc_read(); 

   int error = LINE_THRESHOLD - line_sensor_value; 
   int base_speed = 70; 

   int left_speed= base_speed - error * .1; 
   int right_speed= base_speed + error * .1;

   if(left_speed < base_speed && left_speed > base_speed){left_speed=base_speed;}
   
}

void initialize_motors() { 
   gpio_init(R_MOTOR_IN3); 
   gpio_init(R_MOTOR_IN4); 
   gpio_init(R_MOTOR_ENA); 

   gpio_init(L_MOTOR_IN1); 
   gpio_init(L_MOTOR_IN2); 
   gpio_init(L_MOTOR_ENB); 

}
