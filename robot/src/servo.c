#include "servo.h"

#include "driver/mcpwm_prelude.h"

#define SERVO_TIMER_RESOLUTION_HZ  1000000
#define SERVO_PERIOD_US            (1000000 / SERVO_FREQ_HZ)

static mcpwm_cmpr_handle_t comparator;

esp_err_t servo_init(void)
{
    mcpwm_timer_handle_t timer;
    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMER_RESOLUTION_HZ,
        .period_ticks = SERVO_PERIOD_US,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));

    mcpwm_oper_handle_t oper;
    mcpwm_operator_config_t oper_cfg = {.group_id = 0};
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    // new pulse width takes effect at the start of the next period, so it never changes mid pulse
    mcpwm_comparator_config_t cmpr_cfg = {.flags.update_cmp_on_tez = true};
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmpr_cfg, &comparator));

    mcpwm_gen_handle_t generator;
    mcpwm_generator_config_t gen_cfg = {.gen_gpio_num = SERVO_GPIO};
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, &generator));

    // pin goes high at the start of the period, low when the timer hits the compare value
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    servo_set_angle(SERVO_MAX_ANGLE_DEG / 2.0f);

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    return ESP_OK;
}

void servo_set_angle(float angle)
{
    if (angle < 0.0f) {
        angle = 0.0f;
    } else if (angle > SERVO_MAX_ANGLE_DEG) {
        angle = SERVO_MAX_ANGLE_DEG;
    }

    float pulse_us = SERVO_MIN_PULSE_US + (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * angle / SERVO_MAX_ANGLE_DEG;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, (uint32_t)pulse_us));
}
