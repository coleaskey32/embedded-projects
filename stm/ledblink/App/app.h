#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void App_Init(TIM_HandleTypeDef* timer, UART_HandleTypeDef* huart);
void App_Run(void);

#ifdef __cplusplus
}
#endif