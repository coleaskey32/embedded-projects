#pragma once

#include "pid.h"

#include <cstdint>

/* Closes a position loop around a sensor that wraps, such as the AS5600.
 *
 * This is the layer that knows the measurement is an angle. It turns encoder
 * counts into the two things a PID can actually use:
 *
 * - an error that takes the short way round, so a target just across the zero
 *   crossing does not send the motor most of a turn the wrong way, and
 * - a continuous position for the derivative to difference, because the raw
 *   count jumps the full scale every time the shaft crosses zero and that
 *   jump would otherwise read as near-infinite velocity. */
class PositionController
{
public:
    explicit PositionController(int32_t countsPerRev)
        : countsPerRev_(countsPerRev)
    {
    }

    void SetGains(float kp, float ki, float kd) { pid_.SetGains(kp, ki, kd); }
    void SetOutputLimits(float minimum, float maximum) { pid_.SetOutputLimits(minimum, maximum); }

    void SetTarget(uint16_t counts) { targetCounts_ = counts; }
    uint16_t Target() const { return targetCounts_; }

    /* Returns the motor command, -1 to +1. */
    float Update(uint16_t measuredCounts, float dt);

    /* Drops the integral, the derivative history and the unwrapped position,
     * so the next Update() starts clean from wherever the shaft now is. */
    void Reset();

private:
    /* Folds a count difference into -half..+half of a revolution. */
    int32_t ShortestPath(int32_t delta) const;

    Pid pid_;
    int32_t countsPerRev_;

    uint16_t targetCounts_ = 0;

    /* Running total that keeps counting past a wrap instead of jumping. */
    int32_t continuousCounts_ = 0;
    uint16_t previousCounts_ = 0;
    bool hasPrevious_ = false;
};
