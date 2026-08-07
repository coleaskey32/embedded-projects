#include "app.h"
#include "usbd_cdc_if.h"

static TIM_HandleTypeDef* pwmTimer = nullptr;
static UART_HandleTypeDef* huart2 = nullptr;
void App_Init(TIM_HandleTypeDef* timer, UART_HandleTypeDef* huart)
{
    pwmTimer = timer;
    huart2 = huart;
    HAL_TIM_PWM_Start(pwmTimer, TIM_CHANNEL_1);

    __HAL_TIM_SET_COMPARE(
        pwmTimer,
        TIM_CHANNEL_1,
        500
    );
}

void App_Run(void)
{
    static uint8_t TxBuffer[] = "Hello World! From STM32 USB CDC Device To Virtual COM Port\r\n";

    HAL_UART_Transmit(huart2 ,TxBuffer, sizeof(TxBuffer) - 1, 1000);
    HAL_Delay(100);
}
