#ifndef RSSI_H
#define RSSI_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define RSSI_NUM_ANTENNAS 5
#define RSSI_NUM_OUTER    4
#define RSSI_RX_BUF_SIZE  512
#define RSSI_LINE_BUF_SIZE 128

/* Antenna indices */
#define RSSI_ANT_OUTER1   0  /* USART3 */
#define RSSI_ANT_OUTER2   1  /* UART4  */
#define RSSI_ANT_OUTER3   2  /* UART5  */
#define RSSI_ANT_OUTER4   3  /* USART6 */
#define RSSI_ANT_CENTER   4  /* USART1 */

typedef struct {
    uint8_t left;       /* raw 0-255 */
    uint8_t right;      /* raw 0-255 */
    uint8_t valid;      /* 1 after first successful parse */
    uint32_t timestamp; /* HAL_GetTick() of last update */
} RssiReading;

void RSSI_Init(void);
void RSSI_Poll(void);
void RSSI_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t Size);
const RssiReading *RSSI_GetReading(uint8_t antenna_index);

#endif /* RSSI_H */
