#ifndef COMM_H
#define COMM_H

#include <stdint.h>

#include "esp_err.h"
#include "esp_now.h"
#include "esp_wifi.h"

#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   WIFI_IF_STA

// Channel and PMK needs to be the same as the robot
#define ESPNOW_CHANNEL        1
#define ESPNOW_PMK            "pmk1234567890123" // exactly 16 bytes
#define ESPNOW_QUEUE_SIZE     32
#define ESPNOW_SEND_DELAY_MS  100

// d4:e9:f4:b1:4b:90
#define ROBOT_MAC {0xD4, 0xE9, 0xF4, 0xB1, 0x4B, 0x90} 

// Must match the robot's comm.h
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

void comm_send(const espnow_packet_t *packet);

#endif
