#include "motor.h"

void MotorDriver::Begin()
{
    period_ = __HAL_TIM_GET_AUTORELOAD(timer_);

    __HAL_TIM_SET_COMPARE(timer_, forwardChannel_, 0);
    __HAL_TIM_SET_COMPARE(timer_, reverseChannel_, 0);

    HAL_TIM_PWM_Start(timer_, forwardChannel_);
    HAL_TIM_PWM_Start(timer_, reverseChannel_);
}

void MotorDriver::SetDuty(uint32_t channel, float duty)
{
    /* period_ + 1 counts make a full cycle, so scaling by period_ + 1 lets a
     * duty of 1.0 reach a compare value the counter never exceeds. */
    const uint32_t compare = static_cast<uint32_t>(duty * static_cast<float>(period_ + 1));
    __HAL_TIM_SET_COMPARE(timer_, channel, compare);
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
        SetDuty(reverseChannel_, 0.0f);
        SetDuty(forwardChannel_, command);
    }
    else if (command < 0.0f)
    {
        SetDuty(forwardChannel_, 0.0f);
        SetDuty(reverseChannel_, -command);
    }
    else
    {
        Coast();
    }
}

void MotorDriver::Coast()
{
    SetDuty(forwardChannel_, 0.0f);
    SetDuty(reverseChannel_, 0.0f);
}
