#include "setup.h"
#include "stepper.h"
#include "stm32f4xx_hal.h"
#include <string.h>

extern UART_HandleTypeDef huart2;

#define SETUP_SMALL_STEP  10
#define SETUP_BIG_STEP    100
#define STEP_DELAY_MS     0.5 //delay between steppings so we dont skip TODO: double check if this is neccessary

static void print(const char *msg)
{
    HAL_UART_Transmit(&huart2, (const uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

//uses uart2 to echo keys to the mcu, and translate those to commands for the stepper motor and driver
void Setup_ManualAlign(void)
{
    print("\r\n=== Manual Alignment ===\r\n");
    print("u/d/l/r = 1 step    U/D/L/R = 10 steps\r\n");
    print("Press ENTER when pointed at rocket.\r\n\r\n");

    uint8_t ch;
    while (1) {
        if (HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY) != HAL_OK) //blocking poll until uart2 can recieve something properly; retries when get a bad byte instead of using garbage values
            continue;

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
        case '\r':
        case '\n':
            //zeroing the position once its pointing to rocket
            Stepper_ZeroPosition(STEPPER_AZ); 
            Stepper_ZeroPosition(STEPPER_EL);
            print("Aligned. Starting tracking.\r\n");
            return;
        default:
            continue;
        }

        /* Echo the key */ //here to confirm if the mcu actually recieved the key
        HAL_UART_Transmit(&huart2, &ch, 1, HAL_MAX_DELAY);

        for (int i = 0; i < steps; i++) {
            Stepper_Step(axis, dir);
            if (steps > 1)
                HAL_Delay(STEP_DELAY_MS);
        }
    }
}
