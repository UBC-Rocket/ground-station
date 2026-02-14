#ifndef TRACKER_H
#define TRACKER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Gearing — adjust for actual hardware */
#define TRACKER_STEPS_PER_DEG  10

/* Velocity estimation */
#define TRACKER_VEL_ALPHA      0.3f  /* EMA smoothing (0=smooth, 1=responsive) */
#define TRACKER_VEL_MAX        1.0f  /* angular velocity clamp (rad/s, ~57 deg/s) */

/* Fused controller */
#define TRACKER_K_RSSI         0.002f /* RSSI-to-radians gain (tune for antenna pattern) */
#define TRACKER_RSSI_TIMEOUT   1000   /* RSSI stale threshold (ms) */
#define TRACKER_CTRL_INTERVAL_MS 20   /* control loop period (50 Hz) */

void Tracker_Init(float ground_lat, float ground_lon, float ground_alt);
void Tracker_Poll(void);
void Tracker_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t Size);

#endif /* TRACKER_H */
