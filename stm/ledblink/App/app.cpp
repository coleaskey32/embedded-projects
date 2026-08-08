#include "app.h"
#include "as5600.h"

#include <cstdio>
#include <cstring>

static TIM_HandleTypeDef* pwmTimer = nullptr;
static AS5600 encoder(&hi2c1);

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

static void Print(const char* text)
{
    HAL_UART_Transmit(&hcom_uart[COM1],
                      reinterpret_cast<uint8_t*>(const_cast<char*>(text)),
                      strlen(text), 1000);
}

/* Reports what the encoder sees of its magnet, which separates a wiring
 * problem from a magnet placement problem on the first boot. */
static void ReportEncoderHealth()
{
    if (!encoder.IsPresent())
    {
        Print("AS5600: no ACK - check SDA/SCL wiring and 3V3 power\r\n");
        return;
    }

    uint8_t status = 0;
    uint8_t agc = 0;
    if (!encoder.ReadStatus(status) || !encoder.ReadAgc(agc))
    {
        Print("AS5600: found, but register read failed\r\n");
        return;
    }

    char msg[96];
    snprintf(msg, sizeof(msg), "AS5600: %s (AGC %u)\r\n",
             (status & AS5600::kMagnetTooWeak)   ? "magnet too weak/far" :
             (status & AS5600::kMagnetTooStrong) ? "magnet too strong/close" :
             (status & AS5600::kMagnetDetected)  ? "magnet OK" :
                                                   "no magnet detected",
             static_cast<unsigned>(agc));
    Print(msg);
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

    ReportEncoderHealth();
}

void App_Run(void)
{
    const uint32_t pa7 = ReadAdcChannel(&hadc2, ADC_CHANNEL_4);  // ADC2_IN4
    const uint32_t pc4 = ReadAdcChannel(&hadc2, ADC_CHANNEL_5);  // ADC2_IN5
    const uint32_t pa8 = ReadAdcChannel(&hadc5, ADC_CHANNEL_1);  // ADC5_IN1
    const uint32_t pa9 = ReadAdcChannel(&hadc5, ADC_CHANNEL_2);  // ADC5_IN2


    uint16_t angleCounts = 0;
    const bool angleValid = encoder.ReadAngle(angleCounts);

    char msg[128];
    int len = snprintf(msg, sizeof(msg),
                       "PC4: %4lu PA7: %4lu  PA8: %4lu  PA9: %4lu",
                       static_cast<unsigned long>(pc4),
                       static_cast<unsigned long>(pa7),
                       static_cast<unsigned long>(pa8),
                       static_cast<unsigned long>(pa9));

    if (angleValid)
    {
        /* 4096 counts per turn; the tenths come from integer math so the
         * build keeps its no-float printf. */
        const uint32_t deciDegrees = (static_cast<uint32_t>(angleCounts) * 3600U) / 4096U;
        len += snprintf(msg + len, sizeof(msg) - len,
                        "  ENC: %4u (%lu.%lu deg)\r\n",
                        static_cast<unsigned>(angleCounts),
                        static_cast<unsigned long>(deciDegrees / 10U),
                        static_cast<unsigned long>(deciDegrees % 10U));
    }
    else
    {
        len += snprintf(msg + len, sizeof(msg) - len, "  ENC: ----\r\n");
    }

    HAL_UART_Transmit(&hcom_uart[COM1], reinterpret_cast<uint8_t*>(msg), len, 1000);
    HAL_Delay(100);
}
