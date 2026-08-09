#include "pid.h"

void Pid::Reset()
{
    integral_ = 0.0f;
    previousMeasurement_ = 0.0f;
    hasHistory_ = false;
}

float Pid::Update(float error, float measurement, float dt)
{
    if (dt <= 0.0f)
    {
        return 0.0f;
    }

    const float proportional = kp_ * error;

    /* Derivative of the measurement, negated so it still opposes motion the
     * way a derivative-of-error term would. The first call has no history to
     * difference against, so it contributes nothing. */
    float derivative = 0.0f;
    if (hasHistory_)
    {
        derivative = -kd_ * (measurement - previousMeasurement_) / dt;
    }
    previousMeasurement_ = measurement;
    hasHistory_ = true;

    const float candidateIntegral = integral_ + error * dt;
    const float unclamped = proportional + ki_ * candidateIntegral + derivative;

    /* Conditional integration: accept the new integral only if it does not
     * push the output further past a limit it has already hit. */
    if (unclamped >= outputMax_ && error > 0.0f)
    {
        /* Saturated high and the error would add more: hold the integral. */
    }
    else if (unclamped <= outputMin_ && error < 0.0f)
    {
        /* Saturated low and the error would subtract more: hold. */
    }
    else
    {
        integral_ = candidateIntegral;
    }

    float output = proportional + ki_ * integral_ + derivative;

    if (output > outputMax_)
    {
        output = outputMax_;
    }
    else if (output < outputMin_)
    {
        output = outputMin_;
    }

    return output;
}
