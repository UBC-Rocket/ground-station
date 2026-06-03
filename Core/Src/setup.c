#include "setup.h"
#include "passthrough.h"
#include "stepper.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

#define SETUP_SMALL_STEP  10
#define SETUP_BIG_STEP    100
#define STEP_DELAY_MS     0.5 // delay between steps to avoid skipping; TODO: verify if necessary

/* Non-blocking stream reader for UART2 (PC) DMA buffer */
static const uint8_t *pc_rx_buf;
static volatile uint16_t write_pos;
static uint16_t read_pos;

static void print(const char *msg)
{
    HAL_UART_Transmit(&huart2, (const uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

static void handle_key(uint8_t ch)
{
    int steps = 0;
    StepperAxis axis = STEPPER_AZ;
    StepperDir dir = STEPPER_CW;

    switch (ch) {
    case 'd': steps = SETUP_SMALL_STEP; axis = STEPPER_AZ; dir = STEPPER_CW;  break;
    case 'a': steps = SETUP_SMALL_STEP; axis = STEPPER_AZ; dir = STEPPER_CCW; break;
    case 'w': steps = SETUP_SMALL_STEP; axis = STEPPER_EL; dir = STEPPER_CW;  break;
    case 's': steps = SETUP_SMALL_STEP; axis = STEPPER_EL; dir = STEPPER_CCW; break;
    case 'D': steps = SETUP_BIG_STEP;   axis = STEPPER_AZ; dir = STEPPER_CW;  break;
    case 'A': steps = SETUP_BIG_STEP;   axis = STEPPER_AZ; dir = STEPPER_CCW; break;
    case 'W': steps = SETUP_BIG_STEP;   axis = STEPPER_EL; dir = STEPPER_CW;  break;
    case 'S': steps = SETUP_BIG_STEP;   axis = STEPPER_EL; dir = STEPPER_CCW; break;
    default:
        return;
    }

    /* Add to stepper target so Stepper_Poll() drives the motor */
    int32_t delta = (dir == STEPPER_CW) ? steps : -steps;
    Stepper_SetTarget(axis, Stepper_GetPosition(axis) + delta);
}

/* Blocks until user positions beacon and presses ENTER.
   WASD moves motors; limit switch GPIO HIGH drops movement commands.
   Zeros both axes on exit. */
void Setup_PositionInit(void)
{
    print("\r\n=== Position Initialization ===\r\n");
    print("w/a/s/d = 10 steps    W/A/S/D = 100 steps\r\n");
    print("Press ENTER when positioned.\r\n\r\n");

    uint8_t ch;

    while (1) {
        /* Short timeout so limit switch state is reflected even with no input */
        if (HAL_UART_Receive(&huart2, &ch, 1, 10) != HAL_OK)
            continue;

        if (ch == '\r' || ch == '\n') {
            Stepper_ZeroPosition(STEPPER_AZ);
            Stepper_ZeroPosition(STEPPER_EL);
            print("\r\nPosition zeroed.\r\n");
            return;
        }

        /* Echo the key */
        HAL_UART_Transmit(&huart2, &ch, 1, HAL_MAX_DELAY);

        int steps = 0;
        StepperAxis axis = STEPPER_AZ;
        StepperDir dir = STEPPER_CW;

        switch (ch) {
        case 'd': steps = SETUP_SMALL_STEP; axis = STEPPER_AZ; dir = STEPPER_CW;  break;
        case 'a': steps = SETUP_SMALL_STEP; axis = STEPPER_AZ; dir = STEPPER_CCW; break;
        case 'w': steps = SETUP_SMALL_STEP; axis = STEPPER_EL; dir = STEPPER_CW;  break;
        case 's': steps = SETUP_SMALL_STEP; axis = STEPPER_EL; dir = STEPPER_CCW; break;
        case 'D': steps = SETUP_BIG_STEP;   axis = STEPPER_AZ; dir = STEPPER_CW;  break;
        case 'A': steps = SETUP_BIG_STEP;   axis = STEPPER_AZ; dir = STEPPER_CCW; break;
        case 'W': steps = SETUP_BIG_STEP;   axis = STEPPER_EL; dir = STEPPER_CW;  break;
        case 'S': steps = SETUP_BIG_STEP;   axis = STEPPER_EL; dir = STEPPER_CCW; break;
        default:
            continue;
        }

        if (HAL_GPIO_ReadPin(LIMIT_SW_PORT, LIMIT_SW_PIN) == GPIO_PIN_SET) {
            print("\r\n[LIMIT] End of travel reached -- movement disabled.\r\n");
            continue;
        }

        for (int i = 0; i < steps; i++) {
            Stepper_Step(axis, dir);
            if (steps > 1)
                HAL_Delay(STEP_DELAY_MS);
        }
    }
}

/* Blocks until user enters valid GPS coordinates via UART2.
   Accepts "lat,lon" or "lat,lon,alt" in decimal degrees. */
void Setup_GPSInit(GPSCoord *out)
{
    char line[64];
    uint16_t pos;
    uint8_t ch;

    while (1) {
        print("\r\nEnter GPS coordinates (lat,lon,alt): ");
        pos = 0;
        memset(line, 0, sizeof(line));

        while (1) {
            if (HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY) != HAL_OK)
                continue;

            if (ch == '\r' || ch == '\n') {
                print("\r\n");
                break;
            }

            if ((ch == '\b' || ch == 0x7F) && pos > 0) {
                pos--;
                print("\b \b");
                continue;
            }

            if (pos < sizeof(line) - 1) {
                line[pos++] = (char)ch;
                HAL_UART_Transmit(&huart2, &ch, 1, HAL_MAX_DELAY);
            }
        }

        line[pos] = '\0';

        char *endptr;
        double lat = strtod(line, &endptr);
        if (endptr == line || *endptr != ',') {
            print("Invalid format. Use: lat,lon or lat,lon,alt (e.g. 49.2,-123.1,100.0)\r\n");
            continue;
        }

        char *next = endptr + 1;
        double lon = strtod(next, &endptr);
        if (endptr == next) {
            print("Invalid format. Use: lat,lon or lat,lon,alt (e.g. 49.2,-123.1,100.0)\r\n");
            continue;
        }

        double alt = 0.0;
        if (*endptr == ',') {
            next = endptr + 1;
            alt = strtod(next, NULL);
        }

        out->lat = lat;
        out->lon = lon;
        out->alt = alt;

        char confirm[80];
        snprintf(confirm, sizeof(confirm),
                 "GPS set: lat=%.6f lon=%.6f alt=%.1f\r\n", lat, lon, alt);
        print(confirm);
        return;
    }
}

/* Skeleton tracking loop — runs each iteration of STATE_TRACKING.
   TODO: implement PID feedback:
     1. Compute target az/el from home_pos and incoming beacon GPS
     2. Compare with Stepper_GetPosition for each axis
     3. PID_compute to get adjustment
     4. Stepper_SetTarget to drive motors */
void Setup_TrackingPoll(void)
{
}

void Setup_Init(void)
{
    pc_rx_buf = Passthrough_GetPcRxBuf();
    write_pos = 0;
    read_pos  = 0;
}

void Setup_Poll(void)
{
    uint16_t wp = write_pos;
    while (read_pos != wp) {
        handle_key(pc_rx_buf[read_pos]);
        read_pos++;
        if (read_pos >= PT_BUF_SIZE)
            read_pos = 0;
    }
}

void Setup_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2)
        write_pos = Size % PT_BUF_SIZE;
}
