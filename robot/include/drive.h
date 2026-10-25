#ifndef DRIVE_H
#define DRIVE_H

#include "esp_err.h"

// Motors are stopped if no packet arrives from the controller for this long
#define DRIVE_COMMAND_TIMEOUT_MS 500
#define DRIVE_LOG_INTERVAL_MS 1000

// Scales the x input before applying it to the motors, to make the turning less sensitive
#define DRIVE_TURN_GAIN       0.6f

// Smallest motor speed (from 0 to 1) that will actually start moving the motors.
// Brushed dc motors don't move under a certain speed
#define DRIVE_MOTOR_DEADBAND  0.15f

esp_err_t drive_start(void);

#endif
