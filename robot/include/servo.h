#ifndef SERVO_H
#define SERVO_H

#include "esp_err.h"

#define SERVO1_PIN            22
#define SERVO2_PIN            23

#define SERVO1_OFFSET_DEG     0.0f
#define SERVO2_OFFSET_DEG     0.0f

#define SERVO1_INVERTED       0
#define SERVO2_INVERTED       1

#define SERVO_FREQ_HZ         50
#define SERVO_MIN_PULSE_US    500
#define SERVO_MAX_PULSE_US    2400
#define SERVO_MAX_ANGLE_DEG   180

esp_err_t servo_init(void);

// Moves to angle in [0, SERVO_MAX_ANGLE_DEG] degrees
void servo_set_angle(float angle);

#endif
