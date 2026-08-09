#include "position_controller.h"

int32_t PositionController::ShortestPath(int32_t delta) const
{
    const int32_t half = countsPerRev / 2;

    if (delta > half)
    {
        delta -= countsPerRev;
    }
    else if (delta < -half)
    {
        delta += countsPerRev;
    }

    return delta;
}

void PositionController::Reset()
{
    integral = 0.0f;
    previousMeasurement = 0.0f;
    hasHistory = false;

    continuousCounts = 0;
    previousCounts = 0;
    hasPrevious = false;
}

float PositionController::Update(uint16_t measuredCounts, float dt)
{
    if (dt <= 0.0f)
    {
        return 0.0f;
    }

    /* --- 1. Unwrap the measurement --------------------------------------
     * Accumulate real motion rather than absolute position: the step between
     * two consecutive reads is small and unambiguous even when the raw count
     * wraps, so the running total stays continuous for the derivative. */
    if (hasPrevious)
    {
        continuousCounts += ShortestPath(static_cast<int32_t>(measuredCounts) -
                                         static_cast<int32_t>(previousCounts));
    }
    else
    {
        continuousCounts = static_cast<int32_t>(measuredCounts);
        hasPrevious = true;
    }

    previousCounts = measuredCounts;

    /* --- 2. Error and measurement, normalised ---------------------------
     * Dividing by half a revolution makes a full-scale error 1.0, so the
     * gains stay unitless whatever the encoder's resolution is. */
    const float half = static_cast<float>(countsPerRev / 2);

    const float error = static_cast<float>(
        ShortestPath(static_cast<int32_t>(targetCounts) -
                     static_cast<int32_t>(measuredCounts))) / half;

    const float measurement = static_cast<float>(continuousCounts) / half;

    /* --- 3. PID ---------------------------------------------------------- */
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
    const bool saturatedHigh = unclamped >= outputMax && error > 0.0f;
    const bool saturatedLow = unclamped <= outputMin && error < 0.0f;

    if (!saturatedHigh && !saturatedLow)
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
