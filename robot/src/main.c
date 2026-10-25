
#include "main.h"
#include "comm.h"
#include "motor.h"
#include "servo.h"
#include "drive.h"

#include "esp_err.h"

void app_main(void)
{
    ESP_ERROR_CHECK(motor_init());
    ESP_ERROR_CHECK(servo_init());
    ESP_ERROR_CHECK(comm_init());
    ESP_ERROR_CHECK(drive_start());
}
