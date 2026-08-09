#include "pid.h"

void Pid::Reset()
{
    integral = 0.0f;
    previousMeasurement = 0.0f;
    hasHistory = false;
}

float Pid::UpdateWithError(float error, float measurement, float dt)
{
    if (dt <= 0.0f)
    {
        return 0.0f;
    }

    const float proportional = kp * error;

    /* Derivative of the measurement, negated so it still opposes motion the
     * way a derivative-of-error term would. The first call has no history to
     * difference against, so it contributes nothing. */
    float derivative = 0.0f;
    if (hasHistory)
    {
        derivative = -kd * (measurement - previousMeasurement) / dt;
    }
    previousMeasurement = measurement;
    hasHistory = true;

    const float candidateIntegral = integral + error * dt;
    const float unclamped = proportional + ki * candidateIntegral + derivative;

    /* Conditional integration: accept the new integral only if it does not
     * push the output further past a limit it has already hit. */
    if (unclamped >= outputMax && error > 0.0f)
    {
        /* Saturated high and the error would add more: hold the integral. */
    }
    else if (unclamped <= outputMin && error < 0.0f)
    {
        /* Saturated low and the error would subtract more: hold. */
    }
    else
    {
        integral = candidateIntegral;
    }

    float output = proportional + ki * integral + derivative;

    if (output > outputMax)
    {
        output = outputMax;
    }
    else if (output < outputMin)
    {
        output = outputMin;
    }

    return output;
}
