#include "app.h"

#include <cstdio>

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
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 10);
    uint32_t adc2Raw = HAL_ADC_GetValue(&hadc2);

    HAL_ADC_Start(&hadc5);
    HAL_ADC_PollForConversion(&hadc5, 10);
    uint32_t adc5Raw = HAL_ADC_GetValue(&hadc5);

    char msg[64];
    int len = snprintf(msg, sizeof(msg), "ADC2: %4lu  ADC5: %4lu\r\n",
                       static_cast<unsigned long>(adc2Raw),
                       static_cast<unsigned long>(adc5Raw));

    HAL_UART_Transmit(&hcom_uart[COM1], reinterpret_cast<uint8_t*>(msg), len, 1000);
    HAL_Delay(100);
}
