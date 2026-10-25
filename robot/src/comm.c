#include "comm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"

#include <stdint.h>
#include <string.h>

static const char *TAG = "COMM";

static QueueHandle_t espnow_queue;

static volatile uint32_t rx_count;

static esp_err_t nvs_init(void)
{
    esp_err_t res = nvs_flash_init();
    if (res == ESP_ERR_NVS_NO_FREE_PAGES || res == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        res = nvs_flash_init();
    }
    return res;
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (data == NULL || len != sizeof(espnow_packet_t)) {
        return;
    }

    espnow_packet_t packet;
    memcpy(&packet, data, sizeof(packet));
    xQueueOverwrite(espnow_queue, &packet);
    rx_count++;
}

static esp_err_t espnow_init(void)
{
    espnow_queue = xQueueCreate(1, sizeof(espnow_packet_t));
    if (espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create queue fail");
        return ESP_FAIL;
    }

    // rx only, so no peer needs to be added
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_set_pmk((const uint8_t *)ESPNOW_PMK));

    return ESP_OK;
}

esp_err_t comm_init(void)
{
    ESP_ERROR_CHECK(nvs_init());

    wifi_init();

    uint8_t own_mac[ESP_NOW_ETH_ALEN];
    ESP_ERROR_CHECK(esp_read_mac(own_mac, ESP_MAC_WIFI_STA));
    ESP_LOGI(TAG, "My MAC: " MACSTR, MAC2STR(own_mac));

    return espnow_init();
}

esp_err_t comm_receive(espnow_packet_t *packet, TickType_t timeout)
{
    return xQueueReceive(espnow_queue, packet, timeout) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

uint32_t comm_rx_count(void)
{
    return rx_count;
}
