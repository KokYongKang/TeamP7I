#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "motor.h"
#include "../encoder/encoder.h"

// Define GPIO pins for motor control
#define RIGHT_MOTOR_PIN_IN3 8   // IN1: input pin for right motor direction control 
#define RIGHT_MOTOR_PIN_IN4 9   // IN2: input pin for right motor direction control
#define RIGHT_MOTOR_PIN_ENA 1   // ENA: enable pin for right motor speed control (PWM)
#define LEFT_MOTOR_PIN_IN1 2    // IN1: input pin for left motor direction control 
#define LEFT_MOTOR_PIN_IN2 3    // IN2: input pin for left motor direction control
#define LEFT_MOTOR_PIN_ENB 0    // ENB: enable pin for left motor speed control (PWM)

// PWM parameters
#define PWM_CYCLE 65536
#define MS_PER_DEGREE 9         // for car steering

// Motor parameters
const bool forward_direction = true;
const bool turn_right = true;
bool car_moving = false;
const float target_speed = 36.0f; // to be integrated with the accelerometer

PID right_pid = {
    .Kp = 1.0f, 
    .Ki = 0.04f, 
    .Kd = 0.0f, 
    .target_speed = 0.0f,
    .integral = 0.0f, 
    .prev_error = 0.0f,
    .current_speed = 0.0f, 
    .duty_cycle = 0.0f,
};

PID left_pid = {
    .Kp = 1.0f, 
    .Ki = 0.04f, 
    .Kd = 0.0f, 
    .target_speed = 0.0f,
    .integral = 0.0f, 
    .prev_error = 0.0f,
    .current_speed = 0.0f, 
    .duty_cycle = 0.0f,
};

// PID timer parameter
struct repeating_timer pid_timer;

// Function to control the car movements
void car_control(int car_states){
    switch(car_states){
        case FORWARD:
            car_speed(target_speed);
            car_direction(forward_direction);
            car_move();
            break;
        case REVERSE:
            car_speed(target_speed);
            car_direction(!forward_direction);
            car_move();
            break;
        case TURN_RIGHT:
            car_speed(target_speed);
            car_turn(turn_right, 90); // Turn right for 90 degrees
            car_stop();
            break;
        case TURN_LEFT:
            car_speed(target_speed);
            car_turn(!turn_right, 90); // Turn right for 90 degrees
            car_stop();
            break;
        case STOP:
            car_stop();
            break;
        default:
            break;
    }
}

// Function to setup motors
void motor_init() {
    // Setup GPIO pins for motor control
    gpio_init(RIGHT_MOTOR_PIN_IN3);
    gpio_init(RIGHT_MOTOR_PIN_IN4);
    gpio_init(LEFT_MOTOR_PIN_IN1);
    gpio_init(LEFT_MOTOR_PIN_IN2);
    gpio_set_dir(RIGHT_MOTOR_PIN_IN4, GPIO_OUT);
    gpio_set_dir(RIGHT_MOTOR_PIN_IN3, GPIO_OUT);
    gpio_set_dir(LEFT_MOTOR_PIN_IN1, GPIO_OUT);
    gpio_set_dir(LEFT_MOTOR_PIN_IN2, GPIO_OUT);

    // Setup PWM for motor speed control
    setup_pwm(RIGHT_MOTOR_PIN_ENA, right_pid.duty_cycle);
    setup_pwm(LEFT_MOTOR_PIN_ENB, left_pid.duty_cycle);

    // Set car direction
    car_direction(forward_direction);

    // Setup PID timer (every 250ms)
    add_repeating_timer_ms(-250.0f, pid_timer_callback, NULL, &pid_timer);
}

// Function to set up the PWM for motor control
void setup_pwm(uint gpio, float duty_cycle) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);    // Set the GPIO function to PWM

    float clk_freq = 125000000.0f;  // 125 MHz clock frequency
    uint freq = 100;
    uint slice_num = pwm_gpio_to_slice_num(gpio);  // Find PWM slice connected to the GPIO
    uint32_t divider = clk_freq / (freq * PWM_CYCLE);  // Set divider for desired frequency

    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, (uint16_t)(PWM_CYCLE - 1));  // Set the PWM wrap value (16-bit; 65535)
    set_duty_cycle(gpio, duty_cycle);
    pwm_set_enabled(slice_num, true);  // Enable PWM
}

// Function to set duty cycle to change motor speed
void set_duty_cycle(uint motor_pwm, float duty_cycle) {
    duty_cycle = clamper(duty_cycle, 0.0f, 95.0f); // Clamp duty cycle (0 to 95%)
    uint16_t level = duty_cycle / 100.0f * PWM_CYCLE; // Convert duty cycle to 16-bit value
    pwm_set_gpio_level(motor_pwm, level);
}

// Function to set motor direction
void set_direction(uint motor_dir1, uint motor_dir2, bool direction) {
    gpio_put(motor_dir1, direction);
    gpio_put(motor_dir2, !direction);
}

// Function to set car's target speed
void car_speed(float target_speed) {
    target_speed = clamper(target_speed, 0.0f, 40.0f); // Clamp target speed (0 to 40 cm/s)
    right_pid.target_speed = left_pid.target_speed = target_speed;
}

// Function to set car direction (true = forward, false = reverse)
void car_direction(bool direction) {
    set_direction(RIGHT_MOTOR_PIN_IN3, RIGHT_MOTOR_PIN_IN4, direction);
    set_direction(LEFT_MOTOR_PIN_IN1, LEFT_MOTOR_PIN_IN2, direction);
}

// Function for straight car movement at input duty cycle
void car_move() {
    set_duty_cycle(RIGHT_MOTOR_PIN_ENA, right_pid.duty_cycle);
    set_duty_cycle(LEFT_MOTOR_PIN_ENB, left_pid.duty_cycle);
    car_moving = true;
}

// Function to stop car movement
void car_stop() {
    right_pid.duty_cycle = left_pid.duty_cycle = 0.0f;
    set_duty_cycle(RIGHT_MOTOR_PIN_ENA, right_pid.duty_cycle);
    set_duty_cycle(LEFT_MOTOR_PIN_ENB, left_pid.duty_cycle);
    car_moving = false;
}

// Function to turn car right or left at input angle
void car_turn(bool turn, int angle) {
    set_direction(RIGHT_MOTOR_PIN_IN3, RIGHT_MOTOR_PIN_IN4, !turn);
    set_direction(LEFT_MOTOR_PIN_IN1, LEFT_MOTOR_PIN_IN2, turn);
    sleep_ms(angle * MS_PER_DEGREE); // Turning delay
    car_direction(forward_direction); // Reset car direction to forward
}

// Function to clamp value between min and max
float clamper(float value, float min, float max)
{
    if (value > max) // return max if value > max
    {
        return max;
    }
    if (value < min) // return min if value > min
    {
        return min;
    }
    return value; // no change
}

// Function to compute the control signal
float compute_pid(PID *pid) {
    // Compute error
    float error = pid->target_speed - pid->current_speed;
    // Update integral term
    pid->integral += error;
    // Clamp integral term to avoid integral windup (-100 to 100)
    pid->integral = clamper(pid->integral, -100.0f, 100.0f);
    // Compute derivative term
    float derivative = error - pid->prev_error;
    // Compute control signal
    float control_signal = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
    // Update previous error
    pid->prev_error = error;
    return control_signal;
}

// Function for timer callback for PID control
bool pid_timer_callback(struct repeating_timer *t) {
    if (car_moving){
        // Apply PID adjustments to duty cycles (0 to 95%)
        right_pid.duty_cycle = clamper(50.0f + compute_pid(&right_pid), 0.0f, 95.0f);
        left_pid.duty_cycle = clamper(50.0f + compute_pid(&left_pid), 0.0f, 95.0f);

        // Update duty cycles
        set_duty_cycle(RIGHT_MOTOR_PIN_ENA, right_pid.duty_cycle);
        set_duty_cycle(LEFT_MOTOR_PIN_ENB, left_pid.duty_cycle);
    }

    return true;
}