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

    huart1.Instance = USART1;
}

void App_Run(void)
{
    uint8_t TxBuffer[] = "Hello World! From STM32 USB CDC Device To Virtual COM Port\r\n";
    uint8_t TxBufferLen = sizeof(TxBuffer);

    void SystemClock_Config(void);
    static void MX_GPIO_Init(void);

      HAL_Init();
      SystemClock_Config();
      MX_GPIO_Init();
      MX_USB_DEVICE_Init();

      while (1)
      {
          CDC_Transmit_FS(TxBuffer, TxBufferLen);
          HAL_Delay(100);
      }
}