#include "rssi.h"
#include "passthrough.h"
#include <string.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

typedef struct {
    const uint8_t *rx_buf;
    uint8_t line_buf[RSSI_LINE_BUF_SIZE];
    uint16_t line_pos;
    volatile uint16_t write_pos;
    uint16_t read_pos;
    RssiReading reading;
} RssiAntenna;

static uint8_t outer_rx_bufs[RSSI_NUM_OUTER][RSSI_RX_BUF_SIZE];
static RssiAntenna antennas[RSSI_NUM_ANTENNAS];

static int parse_uint(const char *s, uint16_t len, uint16_t *pos)
{
    int val = 0;
    int found = 0;
    while (*pos < len && s[*pos] >= '0' && s[*pos] <= '9') {
        val = val * 10 + (s[*pos] - '0');
        (*pos)++;
        found = 1;
    }
    return found ? val : -1;
}

static void try_parse_rssi(RssiAntenna *ant)
{
    const char *prefix = "L/R RSSI: ";
    const uint16_t prefix_len = 10;

    if (ant->line_pos < prefix_len + 3) /* minimum: "L/R RSSI: 0/0" */
        return;

    /* Find prefix in line */
    char *line = (char *)ant->line_buf;
    char *found = NULL;
    for (uint16_t i = 0; i <= ant->line_pos - prefix_len; i++) {
        if (memcmp(&line[i], prefix, prefix_len) == 0) {
            found = &line[i + prefix_len];
            break;
        }
    }
    if (!found)
        return;

    uint16_t remaining = ant->line_pos - (uint16_t)(found - line);
    uint16_t pos = 0;

    int left = parse_uint(found, remaining, &pos);
    if (left < 0 || left > 255 || pos >= remaining || found[pos] != '/')
        return;
    pos++; /* skip '/' */

    int right = parse_uint(found, remaining, &pos);
    if (right < 0 || right > 255)
        return;

    ant->reading.left = (uint8_t)left;
    ant->reading.right = (uint8_t)right;
    ant->reading.valid = 1;
    ant->reading.timestamp = HAL_GetTick();
}

static void process_byte(RssiAntenna *ant, uint8_t byte)
{
    if (byte == 0x00) {
        /* End of COBS packet — discard accumulated line data */
        ant->line_pos = 0;
        return;
    }

    if (byte == '\n') {
        /* End of text line — try to parse RSSI */
        try_parse_rssi(ant);
        ant->line_pos = 0;
        return;
    }

    if (byte == '\r')
        return;

    /* Accumulate into line buffer */
    if (ant->line_pos < RSSI_LINE_BUF_SIZE) {
        ant->line_buf[ant->line_pos++] = byte;
    } else {
        /* Overflow — reset */
        ant->line_pos = 0;
    }
}

static void poll_antenna(RssiAntenna *ant)
{
    uint16_t wp = ant->write_pos;
    uint16_t rp = ant->read_pos;

    while (rp != wp) {
        process_byte(ant, ant->rx_buf[rp]);
        rp++;
        if (rp >= RSSI_RX_BUF_SIZE)
            rp = 0;
    }

    ant->read_pos = rp;
}

void RSSI_Init(void)
{
    memset(antennas, 0, sizeof(antennas));

    /* Outer antennae — own DMA buffers */
    for (int i = 0; i < RSSI_NUM_OUTER; i++)
        antennas[i].rx_buf = outer_rx_bufs[i];

    /* Center antenna — shares passthrough rx buffer */
    antennas[RSSI_ANT_CENTER].rx_buf = Passthrough_GetCenterRxBuf();

    /* Start DMA reception on outer antennae */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, outer_rx_bufs[0], RSSI_RX_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, outer_rx_bufs[1], RSSI_RX_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, outer_rx_bufs[2], RSSI_RX_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, outer_rx_bufs[3], RSSI_RX_BUF_SIZE);
}

void RSSI_Poll(void)
{
    for (int i = 0; i < RSSI_NUM_ANTENNAS; i++)
        poll_antenna(&antennas[i]);
}

void RSSI_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t pos = Size % RSSI_RX_BUF_SIZE;

    if (huart->Instance == USART3)
        antennas[RSSI_ANT_OUTER1].write_pos = pos;
    else if (huart->Instance == UART4)
        antennas[RSSI_ANT_OUTER2].write_pos = pos;
    else if (huart->Instance == UART5)
        antennas[RSSI_ANT_OUTER3].write_pos = pos;
    else if (huart->Instance == USART6)
        antennas[RSSI_ANT_OUTER4].write_pos = pos;
    else if (huart->Instance == USART1)
        antennas[RSSI_ANT_CENTER].write_pos = pos;
}

const RssiReading *RSSI_GetReading(uint8_t antenna_index)
{
    if (antenna_index >= RSSI_NUM_ANTENNAS)
        return NULL;
    return &antennas[antenna_index].reading;
}
