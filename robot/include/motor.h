#ifndef MOTOR_H
#define MOTOR_H

#include "driver/gpio.h"
#include "esp_err.h"

// TODO: set these to the actual wiring
#define MOTOR_LEFT_PWM_GPIO   GPIO_NUM_26
#define MOTOR_LEFT_IN1_GPIO   GPIO_NUM_17
#define MOTOR_LEFT_IN2_GPIO   GPIO_NUM_16
#define MOTOR_RIGHT_PWM_GPIO  GPIO_NUM_19
#define MOTOR_RIGHT_IN1_GPIO  GPIO_NUM_18
#define MOTOR_RIGHT_IN2_GPIO  GPIO_NUM_21

#define MOTOR_PWM_FREQ_HZ     5000
#define MOTOR_COUNT           2

typedef enum {
    MOTOR_LEFT,
    MOTOR_RIGHT,
} motor_id_t;

esp_err_t motor_init(void);

// speed is in [-1, 1], negative is reverse
void motor_set(motor_id_t motor, float speed);

void motor_stop(void);

#endif
