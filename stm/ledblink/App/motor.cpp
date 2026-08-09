#include "motor.h"

void MotorDriver::Begin()
{
    period = __HAL_TIM_GET_AUTORELOAD(timer);

    __HAL_TIM_SET_COMPARE(timer, forwardChannel, 0);
    __HAL_TIM_SET_COMPARE(timer, reverseChannel, 0);

    HAL_TIM_PWM_Start(timer, forwardChannel);
    HAL_TIM_PWM_Start(timer, reverseChannel);
}

void MotorDriver::SetDuty(uint32_t channel, float duty)
{
    /* period + 1 counts make a full cycle, so scaling by period + 1 lets a
     * duty of 1.0 reach a compare value the counter never exceeds. */
    const uint32_t compare = static_cast<uint32_t>(duty * static_cast<float>(period + 1));
    __HAL_TIM_SET_COMPARE(timer, channel, compare);
}

void MotorDriver::SetCommand(float command)
{
    if (command > 1.0f)
    {
        command = 1.0f;
    }
    else if (command < -1.0f)
    {
        command = -1.0f;
    }

    if (command > 0.0f)
    {
        SetDuty(reverseChannel, 0.0f);
        SetDuty(forwardChannel, command);
    }
    else if (command < 0.0f)
    {
        SetDuty(forwardChannel, 0.0f);
        SetDuty(reverseChannel, -command);
    }
    else
    {
        Coast();
    }
}

void MotorDriver::Coast()
{
    SetDuty(forwardChannel, 0.0f);
    SetDuty(reverseChannel, 0.0f);
}
