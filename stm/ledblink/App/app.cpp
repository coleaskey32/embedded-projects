#include "app.h"
#include "usbd_cdc_if.h"

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
    static uint8_t TxBuffer[] = "Hello World! From STM32 USB CDC Device To Virtual COM Port\r\n";

    CDC_Transmit_FS(TxBuffer, sizeof(TxBuffer) - 1);
    HAL_Delay(100);
}
