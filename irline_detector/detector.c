#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "motor.h"
#include "encoder.h"

#define LINE_FOLLOWING_PIN 27
#define LINE_THRESHOLD 1000
#define ERROR_SCALE 0.03f
#define BASE_SPEED 20.0f
#define LEFT_SPEED_OFFSET 5.0f // Adjust this value to balance motors

void follow_line()
{
   adc_select_input(1);
   uint16_t line_sensor_value = adc_read();
   float error = line_sensor_value - LINE_THRESHOLD;
   float correction = error * ERROR_SCALE;

   right_pid.target_speed = clamper(BASE_SPEED - correction, 5.0f, 30.0f);
   left_pid.target_speed = clamper(BASE_SPEED + LEFT_SPEED_OFFSET + correction, 5.0f, 30.0f);

   car_direction(true);
   car_move();
}
int main()
{
   stdio_init_all();

   // Initialize hardware
   motor_init();
   adc_init();
   adc_gpio_init(LINE_FOLLOWING_PIN);

   // Main control loop
   while (1)
   {
      follow_line();
      sleep_ms(50); // Small delay to prevent overwhelming the system
   }

   return 0;
}