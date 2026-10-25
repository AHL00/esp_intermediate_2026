#ifndef COMM_H
#define COMM_H

#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "esp_now.h"
#include "esp_wifi.h"

#define ESPNOW_WIFI_MODE WIFI_MODE_STA

// Both ESPs must use the same channel and primary master key
#define ESPNOW_CHANNEL        1
#define ESPNOW_PMK            "pmk1234567890123" // exactly 16 bytes

// Must match the controller's comm.h
typedef struct __attribute__((packed)) {
    float x;     // -1 to 1, turning
    float y;     // -1 to 1, forward/back
    float servo; // absolute angle in degrees, 0 to 180
} control_payload_t;

typedef struct __attribute__((packed)) {
    uint16_t seq;
    control_payload_t payload;
} espnow_packet_t;

esp_err_t comm_init(void);

// waits for latest packet, ESP_ERR_TIMEOUT if none received within timeout
esp_err_t comm_receive(espnow_packet_t *packet, TickType_t timeout);

uint32_t comm_rx_count(void);

#endif
