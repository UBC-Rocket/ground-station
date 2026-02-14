#ifndef TRACKER_H
#define TRACKER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Tuneables — adjust for actual hardware */
#define TRACKER_STEPS_PER_DEG  10
#define TRACKER_RSSI_POLL_MS   200
#define TRACKER_RSSI_TIMEOUT   1000
#define TRACKER_RSSI_DEADZONE  2

/* Rocket GPS packet (COBS-encoded over center antenna) */
#define GPS_PACKET_ID  0x01

typedef struct __attribute__((packed)) {
    uint8_t id;         /* GPS_PACKET_ID */
    float latitude;     /* degrees */
    float longitude;    /* degrees */
    float altitude;     /* meters MSL */
} GpsPacket;

void Tracker_Init(float ground_lat, float ground_lon, float ground_alt);
void Tracker_Poll(void);
void Tracker_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t Size);

#endif /* TRACKER_H */
