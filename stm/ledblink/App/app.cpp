#include "app.h"

#include <cstdio>

static TIM_HandleTypeDef* pwmTimer = nullptr;

/* Selects `channel` as the single regular-sequence entry, then does one
 * blocking conversion. Needed because CubeMX generated both ADCs with a
 * one-slot sequence, so reading ADC5_IN1 and ADC5_IN2 means re-pointing
 * the slot between reads. */
static uint32_t ReadAdcChannel(ADC_HandleTypeDef* hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    HAL_ADC_ConfigChannel(hadc, &sConfig);

    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 10);
    const uint32_t value = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);

    return value;
}

void App_Init(TIM_HandleTypeDef* timer)
{
    pwmTimer = timer;
    HAL_TIM_PWM_Start(pwmTimer, TIM_CHANNEL_1);

    __HAL_TIM_SET_COMPARE(
        pwmTimer,
        TIM_CHANNEL_1,
        500
    );

    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);
}

void App_Run(void)
{
    const uint32_t pa7 = ReadAdcChannel(&hadc2, ADC_CHANNEL_4);  // ADC2_IN4
    const uint32_t pa8 = ReadAdcChannel(&hadc5, ADC_CHANNEL_1);  // ADC5_IN1
    const uint32_t pa9 = ReadAdcChannel(&hadc5, ADC_CHANNEL_2);  // ADC5_IN2

    char msg[80];
    const int len = snprintf(msg, sizeof(msg),
                       "PA7: %4lu  PA8: %4lu  PA9: %4lu\r\n",
                       static_cast<unsigned long>(pa7),
                       static_cast<unsigned long>(pa8),
                       static_cast<unsigned long>(pa9));

    HAL_UART_Transmit(&hcom_uart[COM1], reinterpret_cast<uint8_t*>(msg), len, 1000);
    HAL_Delay(100);
}
