#include "setup.h"
#include "stepper.h"
#include "stm32f4xx_hal.h"
#include <string.h>

extern UART_HandleTypeDef huart2;

#define SETUP_SMALL_STEP  1
#define SETUP_BIG_STEP    10
#define STEP_DELAY_MS     2

static void print(const char *msg)
{
    HAL_UART_Transmit(&huart2, (const uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void Setup_ManualAlign(void)
{
    print("\r\n=== Manual Alignment ===\r\n");
    print("u/d/l/r = 1 step    U/D/L/R = 10 steps\r\n");
    print("Press ENTER when pointed at rocket.\r\n\r\n");

    uint8_t ch;
    while (1) {
        if (HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY) != HAL_OK)
            continue;

        int steps = 0;
        StepperAxis axis = STEPPER_AZ;
        StepperDir dir = STEPPER_CW;

        switch (ch) {
        case 'r': steps = SETUP_SMALL_STEP; axis = STEPPER_AZ; dir = STEPPER_CW;  break;
        case 'l': steps = SETUP_SMALL_STEP; axis = STEPPER_AZ; dir = STEPPER_CCW; break;
        case 'u': steps = SETUP_SMALL_STEP; axis = STEPPER_EL; dir = STEPPER_CW;  break;
        case 'd': steps = SETUP_SMALL_STEP; axis = STEPPER_EL; dir = STEPPER_CCW; break;
        case 'R': steps = SETUP_BIG_STEP;   axis = STEPPER_AZ; dir = STEPPER_CW;  break;
        case 'L': steps = SETUP_BIG_STEP;   axis = STEPPER_AZ; dir = STEPPER_CCW; break;
        case 'U': steps = SETUP_BIG_STEP;   axis = STEPPER_EL; dir = STEPPER_CW;  break;
        case 'D': steps = SETUP_BIG_STEP;   axis = STEPPER_EL; dir = STEPPER_CCW; break;
        case '\r':
        case '\n':
            Stepper_ZeroPosition(STEPPER_AZ);
            Stepper_ZeroPosition(STEPPER_EL);
            print("Aligned. Starting tracking.\r\n");
            return;
        default:
            continue;
        }

        /* Echo the key */
        HAL_UART_Transmit(&huart2, &ch, 1, HAL_MAX_DELAY);

        for (int i = 0; i < steps; i++) {
            Stepper_Step(axis, dir);
            if (steps > 1)
                HAL_Delay(STEP_DELAY_MS);
        }
    }
}
