#ifndef DETECTOR_H
#define DETECTOR_H

#include <stdint.h>

void follow_line();
void initialize_motors();
void control_motors(int left_speed, int right_speed);

#endif // DETECTOR_H