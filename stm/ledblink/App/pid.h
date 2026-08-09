#pragma once

/* Fixed-step PID controller.
 *
 * Two details differ from the textbook form and both matter on a motor:
 *
 * - The derivative works on the measurement rather than the error, so turning
 *   the setpoint pot does not produce a derivative spike into the motor.
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

    /* dt is in seconds. Call at a fixed rate; the caller owns the timing. */
    float Update(float error, float measurement, float dt);

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
