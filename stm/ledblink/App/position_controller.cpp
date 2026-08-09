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
    pid.Reset();
    continuousCounts = 0;
    previousCounts = 0;
    hasPrevious = false;
}

float PositionController::Update(uint16_t measuredCounts, float dt)
{
    /* Accumulate real motion rather than absolute position: the step between
     * two consecutive reads is small and unambiguous even when the raw count
     * wraps, so the running total stays continuous for the derivative. */
    if (hasPrevious)
    {
        const int32_t step = ShortestPath(static_cast<int32_t>(measuredCounts) -
                                          static_cast<int32_t>(previousCounts));
        continuousCounts += step;
    }
    else
    {
        continuousCounts = static_cast<int32_t>(measuredCounts);
        hasPrevious = true;
    }

    previousCounts = measuredCounts;

    const int32_t errorCounts = ShortestPath(static_cast<int32_t>(targetCounts) -
                                             static_cast<int32_t>(measuredCounts));

    /* Normalise both by half a revolution so a full-scale error is 1.0 and the
     * gains stay unitless regardless of the encoder's resolution. */
    const float half = static_cast<float>(countsPerRev / 2);
    const float error = static_cast<float>(errorCounts) / half;
    const float measurement = static_cast<float>(continuousCounts) / half;

    return pid.UpdateWithError(error, measurement, dt);
}
