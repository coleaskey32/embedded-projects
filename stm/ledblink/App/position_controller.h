#pragma once

#include <cstdint>

/* Closed-loop position control for a motor with a wrapping absolute encoder.
 *
 * Owns everything the loop needs: the target, the gains, and the PID state.
 * Feed it the encoder's current count and the time since the last call, and
 * it returns a motor command from -1 (full reverse) to +1 (full forward).
 *
 * Four details are worth knowing, because each fixes a specific way a naive
 * position loop misbehaves:
 *
 * - The error takes the short way round, so a target just across the zero
 *   crossing does not send the motor most of a turn the wrong way.
 * - The derivative works off a continuous position rather than the raw count,
 *   which jumps full scale every time the shaft crosses zero and would
 *   otherwise read as near-infinite velocity.
 * - The derivative works on the measurement rather than the error, so moving
 *   the target does not spike the motor.
 * - The integral only accumulates while the output is off its limits, which
 *   keeps it from winding up during a long saturated push. */
class PositionController
{
public:
    explicit PositionController(int32_t countsPerRev)
        : countsPerRev(countsPerRev)
    {
    }

    /* this-> is load bearing: the parameters share their names with the
     * members, so a bare kp = kp would assign the parameter to itself and the
     * gains would silently stay at zero. */
    void SetGains(float kp, float ki, float kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }

    float Kp() const { return kp; }
    float Ki() const { return ki; }
    float Kd() const { return kd; }

    void SetOutputLimits(float minimum, float maximum)
    {
        outputMin = minimum;
        outputMax = maximum;
    }

    /* Where the shaft should be, in encoder counts. */
    void SetTarget(uint16_t counts) { targetCounts = counts; }
    uint16_t Target() const { return targetCounts; }

    /* Feed in where the shaft actually is; get back the motor command.
     * dt is in seconds and the caller owns the timing. */
    float Update(uint16_t measuredCounts, float dt);

    /* Drops the integral, the derivative history and the unwrapped position,
     * so the next Update() starts clean from wherever the shaft now is. Call
     * it whenever the loop has been open for a while, otherwise stale state
     * lands on the motor the moment control resumes. */
    void Reset();

    float Integral() const { return integral; }

private:
    /* Folds a count difference into -half..+half of a revolution. */
    int32_t ShortestPath(int32_t delta) const;

    int32_t countsPerRev;

    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    float outputMin = -1.0f;
    float outputMax = 1.0f;

    uint16_t targetCounts = 0;

    /* PID state. */
    float integral = 0.0f;
    float previousMeasurement = 0.0f;
    bool hasHistory = false;

    /* Running total that keeps counting past a wrap instead of jumping. */
    int32_t continuousCounts = 0;
    uint16_t previousCounts = 0;
    bool hasPrevious = false;
};
