#pragma once

#include "main.h"

/* Two-channel PWM output for a brushed DC driver of the RPWM/LPWM kind
 * (BTS7960 and friends), where one channel drives the motor forward and the
 * other drives it in reverse.
 *
 * Raising both inputs at once is never a useful command and on some drivers
 * it is a destructive one, so SetCommand() always drops the idle channel to
 * zero before it raises the active one. */
class MotorDriver
{
public:
    MotorDriver(TIM_HandleTypeDef* timer, uint32_t forwardChannel, uint32_t reverseChannel)
        : timer_(timer), forwardChannel_(forwardChannel), reverseChannel_(reverseChannel)
    {
    }

    /* Starts both PWM channels at zero duty. */
    void Begin();

    /* command runs from -1 (full reverse) through 0 (coast) to +1 (full
     * forward) and is clamped to that range. */
    void SetCommand(float command);

    /* Both channels low: the motor is left to spin down on its own. */
    void Coast();

private:
    void SetDuty(uint32_t channel, float duty);

    TIM_HandleTypeDef* timer_;
    uint32_t forwardChannel_;
    uint32_t reverseChannel_;

    /* Read from the timer at Begin() so the duty scaling follows whatever
     * period CubeMX generated instead of assuming one. */
    uint32_t period_ = 0;
};
