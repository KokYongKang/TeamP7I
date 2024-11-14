#ifndef MOTOR_H
#define MOTOR_H

typedef struct PID {
    float Kp;                   // Proportional gain, present error
    float Ki;                   // Integral gain, past error
    float Kd;                   // Derivative gain, prediction of future error
    float target_speed;         // Setpoint (cm/s)
    float integral;             // Accumulated integral error
    float prev_error;           // Previous error value for derivative calculation
    float current_speed;        // Current value (cm/s)
    volatile float duty_cycle;  // Current duty cycle
} PID;

enum car_states {
    FORWARD = 0,
    REVERSE = 1,
    TURN_RIGHT = 2,
    TURN_LEFT = 3,
    STOP = 5
};

// To allow access from other files (i.e. encoder.c)
extern bool car_moving;
extern PID right_pid;
extern PID left_pid;

// Control the car
void car_control(int car_states);

// Initialize motors
void motor_init();

// Set up PWM for a specific motor
void setup_pwm(uint gpio, float duty_cycle);

// Set duty cycle for a specific motor
void set_duty_cycle(uint motor_pwm, float duty_cycle);

// Set direction for a specific motor
void set_direction(uint motor_dir1, uint motor_dir2, bool direction);

// Set the car's target speed
void car_speed(float target_speed);

// Change the car direction (true = forward, forward = reverse)
void car_direction(bool direction);

// Move the car at input duty cycle
void car_move();

// Stop the car
void car_stop();

// Turn the car right or left at input angle, then stop, and reset car direction
void car_turn(bool turn, int angle);

// Clamp value between min and max
float clamper(float value, float min, float max);

// Compute PID control adjustments
float compute_pid(PID *pid);

// Timer callback for PID control
bool pid_timer_callback(struct repeating_timer *t);

#endif