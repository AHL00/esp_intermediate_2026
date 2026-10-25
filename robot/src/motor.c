#include "motor.h"

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

#include <math.h>
#include <stdbool.h>

#define MOTOR_MCPWM_GROUP          1 // servo uses group 0
#define MOTOR_TIMER_RESOLUTION_HZ  10000000
#define MOTOR_PERIOD_TICKS         (MOTOR_TIMER_RESOLUTION_HZ / MOTOR_PWM_FREQ_HZ)

typedef struct {
    gpio_num_t pwm_gpio;
    gpio_num_t in1_gpio;
    gpio_num_t in2_gpio;
} motor_hw_t;

static const motor_hw_t motors[MOTOR_COUNT] = {
    [MOTOR_LEFT]  = {.pwm_gpio = MOTOR_LEFT_PWM_GPIO,  .in1_gpio = MOTOR_LEFT_IN1_GPIO,  .in2_gpio = MOTOR_LEFT_IN2_GPIO},
    [MOTOR_RIGHT] = {.pwm_gpio = MOTOR_RIGHT_PWM_GPIO, .in1_gpio = MOTOR_RIGHT_IN1_GPIO, .in2_gpio = MOTOR_RIGHT_IN2_GPIO},
};

static mcpwm_cmpr_handle_t comparators[MOTOR_COUNT];
static mcpwm_gen_handle_t generators[MOTOR_COUNT];
static bool forced_low[MOTOR_COUNT];

esp_err_t motor_init(void)
{
    mcpwm_timer_handle_t timer;
    mcpwm_timer_config_t timer_cfg = {
        .group_id = MOTOR_MCPWM_GROUP,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MOTOR_TIMER_RESOLUTION_HZ,
        .period_ticks = MOTOR_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));

    // an operator has 2 comparators and 2 generators, one of each per motor
    mcpwm_oper_handle_t oper;
    mcpwm_operator_config_t oper_cfg = {.group_id = MOTOR_MCPWM_GROUP};
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    for (int i = 0; i < MOTOR_COUNT; i++) {
        ESP_ERROR_CHECK(gpio_set_direction(motors[i].in1_gpio, GPIO_MODE_OUTPUT));
        ESP_ERROR_CHECK(gpio_set_direction(motors[i].in2_gpio, GPIO_MODE_OUTPUT));

        // New duty takes effect at the start of the next period, so the output never does weird stuff mid pulse
        mcpwm_comparator_config_t cmpr_cfg = {.flags.update_cmp_on_tez = true};
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmpr_cfg, &comparators[i]));

        mcpwm_generator_config_t gen_cfg = {.gen_gpio_num = motors[i].pwm_gpio};
        ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, &generators[i]));

        // pin goes high at the start of the period, low when the timer hits the compare value
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            generators[i], MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            generators[i], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i], MCPWM_GEN_ACTION_LOW)));
    }

    motor_stop();

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    return ESP_OK;
}

void motor_set(motor_id_t motor, float speed)
{
    if (speed > 1.0f) {
        speed = 1.0f;
    } else if (speed < -1.0f) {
        speed = -1.0f;
    }

    uint32_t duty = (uint32_t)(fabsf(speed) * MOTOR_PERIOD_TICKS);

    // compare value 0 sets the pin high and low on the same tick, so force it low instead
    bool want_low = duty == 0;
    if (want_low != forced_low[motor]) {
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generators[motor], want_low ? 0 : -1, true));
        forced_low[motor] = want_low;
    }

    bool reverse = speed < 0.0f;
    gpio_set_level(motors[motor].in1_gpio, !reverse);
    gpio_set_level(motors[motor].in2_gpio, reverse);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[motor], duty));
}

void motor_stop(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_set(i, 0.0f);
    }
}
