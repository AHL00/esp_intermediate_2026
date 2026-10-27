#ifndef CONTROL_H
#define CONTROL_H

#include "esp_err.h"

#define CONTROL_TASK_INTERVAL_MS 20
#define CONTROL_LOG_EVERY_N 10

// NOTE: Must be ADC1 pins (GPIO32 to 39 on ESP32 (not s3)), ADC2 conflicts with WiFi
#define JOYSTICK_X_PIN 34
#define JOYSTICK_Y_PIN 35

#define JOYSTICK_SW_PIN 27

#define JOYSTICK_MIN_MV   142
#define JOYSTICK_MAX_MV   3160
#define JOYSTICK_DEADZONE 0.05f

// how much of the cubic curve to use
#define JOYSTICK_CURVE_CONSTANT  0.5f

// While the switch is held, joystick y moves the servo at up to this speed
// and driving is disabled
#define SERVO_MAX_RATE_DEG_PER_S 900.0f

esp_err_t control_start(void);

#endif
