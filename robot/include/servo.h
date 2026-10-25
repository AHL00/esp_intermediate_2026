#ifndef SERVO_H
#define SERVO_H

#include "esp_err.h"

// TODO: set this to the actual wiring
#define SERVO_GPIO            13

#define SERVO_FREQ_HZ         50
#define SERVO_MIN_PULSE_US    500
#define SERVO_MAX_PULSE_US    2400
#define SERVO_MAX_ANGLE_DEG   180

esp_err_t servo_init(void);

// Moves to angle in [0, SERVO_MAX_ANGLE_DEG] degrees
void servo_set_angle(float angle);

#endif
