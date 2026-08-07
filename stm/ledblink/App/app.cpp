#include "app.h"

static TIM_HandleTypeDef* pwmTimer = nullptr;

void App_Init(TIM_HandleTypeDef* timer)
{
    pwmTimer = timer;

    HAL_TIM_PWM_Start(pwmTimer, TIM_CHANNEL_1);

    __HAL_TIM_SET_COMPARE(
        pwmTimer,
        TIM_CHANNEL_1,
        500
    );
}

void App_Run(void)
{
    static uint8_t TxBuffer[] = "Hello World! From STM32 over the ST-LINK Virtual COM Port\r\n";

    /* hcom_uart[COM1] is LPUART1, wired to the ST-LINK VCP; main.c ran BSP_COM_Init */
    HAL_UART_Transmit(&hcom_uart[COM1], TxBuffer, sizeof(TxBuffer) - 1, HAL_MAX_DELAY);
    HAL_Delay(100);
}
