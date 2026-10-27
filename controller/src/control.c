#include "control.h"
#include "comm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#include <stdbool.h>
#include <math.h>

static const char *TAG = "CONTROL";

typedef struct {
    int16_t x;
    int16_t y;
    int16_t raw_x; // for logging only
    int16_t raw_y;
} joystick_mv_t;

typedef struct {
    float x;
    float y;
} joystick_out_t;

static adc_oneshot_unit_handle_t adc;
static adc_cali_handle_t adc_cali;
static adc_channel_t x_channel;
static adc_channel_t y_channel;

static void adc_init(void)
{
    adc_unit_t unit;
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(JOYSTICK_X_PIN, &unit, &x_channel));
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(JOYSTICK_Y_PIN, &unit, &y_channel));

    adc_oneshot_unit_init_cfg_t unit_cfg = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc));

    adc_oneshot_chan_cfg_t chan_cfg = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, x_channel, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, y_channel, &chan_cfg));

    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_cfg, &adc_cali));
}

static void switch_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1 << JOYSTICK_SW_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
}

static bool switch_held(void)
{
    return gpio_get_level(JOYSTICK_SW_PIN) == 0;
}

static joystick_mv_t joystick_read(void)
{
    int raw, mv;
    joystick_mv_t js;

    ESP_ERROR_CHECK(adc_oneshot_read(adc, x_channel, &raw));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali, raw, &mv));
    js.x = mv;
    js.raw_x = raw;

    ESP_ERROR_CHECK(adc_oneshot_read(adc, y_channel, &raw));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali, raw, &mv));
    js.y = mv;
    js.raw_y = raw;

    return js;
}

static joystick_out_t joystick_map(joystick_mv_t js)
{
    joystick_out_t out;
    float magnitude;

    // rescale to [-1, 1]
    out.x = (float)(js.x - JOYSTICK_MIN_MV) / (JOYSTICK_MAX_MV - JOYSTICK_MIN_MV) * 2.0f - 1.0f;
    out.y = (float)(js.y - JOYSTICK_MIN_MV) / (JOYSTICK_MAX_MV - JOYSTICK_MIN_MV) * 2.0f - 1.0f;

    // apply a deadzone based on the magnitude of the vector
    magnitude = sqrtf(out.x * out.x + out.y * out.y);
    if (magnitude < JOYSTICK_DEADZONE) {
        out.x = 0.0f;
        out.y = 0.0f;
    } else {
        float k = (magnitude - JOYSTICK_DEADZONE) / (1.0f - JOYSTICK_DEADZONE) / magnitude;
        out.x *= k;
        out.y *= k;
    }

    // normalise to a unit vector 
    magnitude = sqrtf(out.x * out.x + out.y * out.y);
    if (magnitude > 1.0f) {
        out.x /= magnitude;
        out.y /= magnitude;
    }

    // acceleration curve, less sensitive near the centre and more sensitive at the edges
    // something like this:
    // https://www.chiefdelphi.com/uploads/default/original/3X/9/0/90389dfd56d1c9925975522accc98e99fc03e5e1.gif
    magnitude = sqrtf(out.x * out.x + out.y * out.y);
    float k = JOYSTICK_CURVE_CONSTANT * magnitude * magnitude * magnitude + (1.0f - JOYSTICK_CURVE_CONSTANT);
    out.x *= k;
    out.y *= k;

    return out;
}

#define SERVO_MAX_ANGLE_DEG 180

static void control_task(void *arg)
{
    espnow_packet_t packet = {0};
    TickType_t last_wake = xTaskGetTickCount();
    float servo = SERVO_MAX_ANGLE_DEG / 2.0f;

    while (true) {
        packet.seq++;
        joystick_mv_t js = joystick_read();
        joystick_out_t out;
        bool sw = switch_held();

        if (sw) {
            // servo mode: only joystick y is used, x is centred
            joystick_mv_t centred = js;
            centred.x = (JOYSTICK_MIN_MV + JOYSTICK_MAX_MV) / 2;
            out = joystick_map(centred);

            servo += out.y * SERVO_MAX_RATE_DEG_PER_S * CONTROL_TASK_INTERVAL_MS / 1000.0f;
            if (servo < 0.0f) {
                servo = 0.0f;
            } else if (servo > SERVO_MAX_ANGLE_DEG) {
                servo = SERVO_MAX_ANGLE_DEG;
            }

            // movement is disabled while controlling the servo
            out.x = 0.0f;
            out.y = 0.0f;
        } else {
            out = joystick_map(js);
        }

        packet.payload = (control_payload_t){.x = out.x, .y = out.y, .servo = servo};

        if (packet.seq % CONTROL_LOG_EVERY_N == 0) {
            ESP_LOGI(TAG, "raw x=%d y=%d, mv x=%d y=%d, sw=%d, out x=%.2f y=%.2f servo=%.1f",
                     js.raw_x, js.raw_y, js.x, js.y, sw, out.x, out.y, servo);
        }

        comm_send(&packet);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_TASK_INTERVAL_MS));
    }
}

esp_err_t control_start(void)
{
    adc_init();
    switch_init();
    return xTaskCreate(control_task, "control", 4096, NULL, 4, NULL) == pdPASS ? ESP_OK : ESP_FAIL;
}
