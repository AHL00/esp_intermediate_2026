#include "drive.h"
#include "comm.h"
#include "motor.h"
#include "servo.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include <math.h>
#include <stdbool.h>

static const char *TAG = "DRIVE";

// brushed dc motors don't move below a certain speed, so remap [0, 1] to [DEADBAND, 1] and [-1, 0] to [-1, -DEADBAND]
static float drive_deadband(float speed)
{
    if (speed == 0.0f) {
        return 0.0f;
    }
    return copysignf(DRIVE_MOTOR_DEADBAND + (1.0f - DRIVE_MOTOR_DEADBAND) * fabsf(speed), speed);
}

static void drive_arcade(float x, float y)
{
    x *= DRIVE_TURN_GAIN;

    float left = y + x;
    float right = y - x;

    float peak = fmaxf(fabsf(left), fabsf(right));
    if (peak > 1.0f) {
        left /= peak;
        right /= peak;
    }

    motor_set(MOTOR_LEFT, drive_deadband(left));
    motor_set(MOTOR_RIGHT, drive_deadband(right));
}

static void drive_task(void *arg)
{
    espnow_packet_t packet;
    espnow_packet_t latest = {0};
    bool have_latest = false;
    bool link_lost = false;
    TickType_t last_log = xTaskGetTickCount();
    uint32_t last_count = comm_rx_count();

    while (true) {
        if (comm_receive(&packet, pdMS_TO_TICKS(DRIVE_COMMAND_TIMEOUT_MS)) == ESP_OK) {
            latest = packet;
            have_latest = true;

            if (link_lost) {
                ESP_LOGI(TAG, "Controller link restored");
                link_lost = false;
            }

            control_payload_t ctrl = packet.payload;
            drive_arcade(-ctrl.x, ctrl.y);
            servo_set_angle(ctrl.servo);
        } else {
            if (!link_lost) {
                ESP_LOGW(TAG, "No packet for %d ms, stopping", DRIVE_COMMAND_TIMEOUT_MS);
                link_lost = true;
            }
            motor_stop();
        }

        TickType_t now = xTaskGetTickCount();
        TickType_t elapsed_ms = (now - last_log) * portTICK_PERIOD_MS;
        if (elapsed_ms >= DRIVE_LOG_INTERVAL_MS) {
            uint32_t count = comm_rx_count();
            uint32_t rate = ((count - last_count) * 1000 + elapsed_ms / 2) / elapsed_ms;

            if (have_latest) {
                ESP_LOGI(TAG, "Rx %u pkt/s, latest seq %u: x=%.2f y=%.2f servo=%.2f", rate, latest.seq,
                         latest.payload.x, latest.payload.y, latest.payload.servo);
            } else {
                ESP_LOGI(TAG, "Rx %u pkt/s, no command received yet", rate);
            }

            last_count = count;
            last_log = now;
        }
    }
}

esp_err_t drive_start(void)
{
    return xTaskCreate(drive_task, "drive", 4096, NULL, 4, NULL) == pdPASS ? ESP_OK : ESP_FAIL;
}
