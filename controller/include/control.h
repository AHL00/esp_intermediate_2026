#ifndef CONTROL_H
#define CONTROL_H

#include "esp_err.h"

#define CONTROL_TASK_INTERVAL_MS 20

// NOTE: Must be ADC1 pins (GPIO32 to 39 on ESP32 (not s3)), ADC2 conflicts with WiFi
#define JOYSTICK_X_PIN 34
#define JOYSTICK_Y_PIN 35

#define JOYSTICK_MIN_MV   0
#define JOYSTICK_MAX_MV   3300
#define JOYSTICK_DEADZONE 0.05f

// how much of the cubic curve to use
#define JOYSTICK_CURVE_CONSTANT  0.5f

esp_err_t control_start(void);

#endif
