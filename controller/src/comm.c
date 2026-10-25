#include "comm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static const char *TAG = "COMM";

static QueueHandle_t tx_queue;     // packets waiting to be sent, drained by tx_task
static QueueHandle_t status_queue; // send result from espnow_send_cb
static const uint8_t robot_mac[ESP_NOW_ETH_ALEN] = ROBOT_MAC;

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

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    xQueueOverwrite(status_queue, &status);
}

static void tx_task(void *arg)
{
    espnow_packet_t packet;
    esp_now_send_status_t status;
    uint32_t sent = 0, not_acked = 0, timed_out = 0, send_errors = 0;
    esp_err_t last_err = ESP_OK;
    TickType_t window_start = xTaskGetTickCount();

    while (true) {
        xQueueReceive(tx_queue, &packet, portMAX_DELAY);

        xQueueReset(status_queue); // clear the status queue before sending, so we don't get a stale status from a previous packet
        esp_err_t err = esp_now_send(robot_mac, (const uint8_t *)&packet, sizeof(packet));
        if (err != ESP_OK) {
            send_errors++;
            last_err = err;
        } else if (xQueueReceive(status_queue, &status, pdMS_TO_TICKS(ESPNOW_SEND_DELAY_MS)) != pdTRUE) {
            timed_out++;
        } else if (status != ESP_NOW_SEND_SUCCESS) {
            not_acked++;
        } else {
            sent++;
        }

        if (xTaskGetTickCount() - window_start >= pdMS_TO_TICKS(1000)) {
            ESP_LOGI(TAG, "%lu sent, %lu not acked, %lu timed out, %lu send errs (%s)",
                     sent, not_acked, timed_out, send_errors, esp_err_to_name(last_err));
            sent = not_acked = timed_out = send_errors = 0;
            window_start = xTaskGetTickCount();
        }
    }
}

static esp_err_t espnow_init(void)
{
    tx_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_packet_t));
    status_queue = xQueueCreate(1, sizeof(esp_now_send_status_t));
    if (tx_queue == NULL || status_queue == NULL) {
        ESP_LOGE(TAG, "Create queue fail");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_set_pmk((const uint8_t *)ESPNOW_PMK));

    esp_now_peer_info_t peer = {
        .channel = ESPNOW_CHANNEL,
        .ifidx = ESPNOW_WIFI_IF,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, robot_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    return xTaskCreate(tx_task, "espnow_tx", 4096, NULL, 5, NULL) == pdPASS ? ESP_OK : ESP_FAIL;
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

void comm_send(const espnow_packet_t *packet)
{
    // drop the oldest packet if the queue is full
    while (xQueueSend(tx_queue, packet, 0) != pdTRUE) {
        espnow_packet_t oldest;
        xQueueReceive(tx_queue, &oldest, 0);
    }
}
