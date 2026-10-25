
#include "comm.h"
#include "control.h"

#include "esp_err.h"

void app_main(void)
{
    ESP_ERROR_CHECK(comm_init());
    ESP_ERROR_CHECK(control_start());
}
