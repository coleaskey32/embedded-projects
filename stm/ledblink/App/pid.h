#pragma once

/* Fixed-step PID controller.
 *
 * Deliberately knows nothing about what it is controlling: it sees numbers,
 * not angles or counts. Anything domain-specific -- units, sensor wraparound,
 * gearing -- belongs in the layer above, which is why UpdateWithError() exists
 * alongside the plain Update().
 *
 * Two details differ from the textbook form and both matter on a motor:
 *
 * - The derivative works on the measurement rather than the error, so moving
 *   the setpoint does not produce a derivative spike into the motor.
 * - The integral only accumulates while the output is off its limits, which
 *   keeps it from winding up during a long saturated push. */
class Pid
{
public:
    void SetGains(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void SetOutputLimits(float minimum, float maximum)
    {
        outputMin_ = minimum;
        outputMax_ = maximum;
    }

    /* For quantities where the error really is just setpoint minus
     * measurement: temperature, velocity, a linear axis. dt is in seconds and
     * the caller owns the timing. */
    float Update(float setpoint, float measurement, float dt)
    {
        return UpdateWithError(setpoint - measurement, measurement, dt);
    }

    /* For quantities where it is not. A wrapping angle is the usual case: the
     * shortest path from 350 to 10 degrees is +20, not -340, and only the
     * caller knows that. `measurement` must still be continuous, since the
     * derivative differences it against the previous call. */
    float UpdateWithError(float error, float measurement, float dt);

    /* Clears the integral and derivative history. Call whenever the loop has
     * been open for a while, otherwise stale state lands on the motor the
     * moment control resumes. */
    void Reset();

    float Integral() const { return integral_; }

private:
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;

    float integral_ = 0.0f;
    float previousMeasurement_ = 0.0f;
    bool hasHistory_ = false;

    float outputMin_ = -1.0f;
    float outputMax_ = 1.0f;
};
