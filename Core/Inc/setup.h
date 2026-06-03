#ifndef SETUP_H
#define SETUP_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef enum {
    STATE_POSITION_INIT,
    STATE_GPS_INIT,
    STATE_TRACKING
} SystemState;

typedef struct {
    double lat;
    double lon;
    double alt;
} GPSCoord;

/* Blocks until user positions beacon and presses ENTER; zeros both motor axes on exit.
   Limit switch (LIMIT_SW_PORT/PIN HIGH) disables movement commands but not ENTER.
   Call after Stepper_Init, before GPS init. */
void Setup_PositionInit(void);

/* Blocks until user enters GPS coordinates via UART2. Stores result in *out.
   Call after Setup_PositionInit, before Passthrough_Init. */
void Setup_GPSInit(GPSCoord *out);

/* Skeleton for tracking mode — PID feedback loop will go here.
   Call from main loop during STATE_TRACKING. I have not written the detailed code for this yet => its a placeholder*/
void Setup_TrackingPoll(void);

/* Call after Passthrough_Init to enable non-blocking manual control. */
void Setup_Init(void);

/* Process wasd keys from UART2 DMA buffer. Call from main loop. */
void Setup_Poll(void);

/* Call from HAL_UARTEx_RxEventCallback for UART2 write position tracking. */
void Setup_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t Size);

#endif /* SETUP_H */
