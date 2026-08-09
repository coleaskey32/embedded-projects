#include "app.h"
#include "as5600.h"
#include "motor.h"
#include "pid.h"

#include <cstdio>
#include <cstring>

namespace
{

/* --- Hardware map -------------------------------------------------------
 * TIM1_CH1 (PC0) is RPWM, TIM1_CH2 (PC1) is LPWM.
 * Pots: PC4 = setpoint, PA7 = Kp, PA8 = Ki, PA9 = Kd. */
constexpr uint32_t kForwardChannel = TIM_CHANNEL_1;
constexpr uint32_t kReverseChannel = TIM_CHANNEL_2;

constexpr uint32_t kAdcFullScale = 4095;

/* --- Loop rates ---------------------------------------------------------
 * Control runs fast and fixed because the PID depends on a steady dt; the
 * pots and the serial log run slowly because nothing needs them faster. */
constexpr uint32_t kControlPeriodMs = 1;
constexpr uint32_t kTuningPeriodMs  = 50;
constexpr uint32_t kPrintPeriodMs   = 100;

/* --- Encoder ------------------------------------------------------------ */
constexpr int32_t kCountsPerRev = 4096;
constexpr float   kHalfRev      = kCountsPerRev / 2.0f;

/* Consecutive failed encoder reads before the loop gives up and disarms.
 * Holding a motor command against a dead sensor is how things get broken. */
constexpr uint32_t kMaxEncoderFailures = 10;

/* --- Bring-up aid -------------------------------------------------------
 * Set true to take the encoder and the PID out of the loop entirely: the
 * setpoint pot then drives the motor directly, centre being stop and either
 * side ramping to full speed in that direction. It exists so motor, driver
 * and PWM wiring can be proven on their own, without a working encoder as a
 * precondition. Set it back to false for real closed-loop control. */
constexpr bool kOpenLoopTest = false;

/* Below this fraction the pot counts as centred, so a stationary knob near
 * the middle does not creep the motor. */
constexpr float kOpenLoopDeadband = 0.05f;

/* --- Gain ranges --------------------------------------------------------
 * Each pot sweeps its gain from zero to the value below. Error is normalised
 * to +/-1 over half a turn before it reaches the PID, so these are unitless
 * and stay in a sane range whatever the mechanics do. */
constexpr float kMaxKp = 8.0f;
constexpr float kMaxKi = 4.0f;
constexpr float kMaxKd = 0.5f;

MotorDriver motor(&htim1, kForwardChannel, kReverseChannel);
AS5600 encoder(&hi2c1);
Pid pid;

/* Both are touched by the button ISR as well as the control loop. */
volatile bool armed = false;
volatile bool encoderHealthy = false;

uint16_t setpointCounts = 0;
float gainKp = 0.0f;
float gainKi = 0.0f;
float gainKd = 0.0f;

uint16_t measuredCounts = 0;
float lastCommand = 0.0f;
uint32_t encoderFailures = 0;

/* Selects `channel` as the single regular-sequence entry, then does one
 * blocking conversion. Needed because CubeMX generated both ADCs with a
 * one-slot sequence, so reading ADC5_IN1 and ADC5_IN2 means re-pointing
 * the slot between reads. */
uint32_t ReadAdcChannel(ADC_HandleTypeDef* hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    HAL_ADC_ConfigChannel(hadc, &sConfig);

    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 10);
    const uint32_t value = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);

    return value;
}

void Print(const char* text)
{
    HAL_UART_Transmit(&hcom_uart[COM1],
                      reinterpret_cast<uint8_t*>(const_cast<char*>(text)),
                      strlen(text), 1000);
}

/* Reports what the encoder sees of its magnet, which separates a wiring
 * problem from a magnet placement problem on the first boot. */
void ReportEncoderHealth()
{
    if (!encoder.IsPresent())
    {
        Print("AS5600: no ACK at 0x36 - scanning bus...\r\n");

        uint8_t found[8] = {};
        const uint8_t count = encoder.ScanBus(found, sizeof(found));

        if (count == 0)
        {
            Print("  bus empty - check SCL/SDA wiring, 3V3 power, and GND\r\n");
        }
        else
        {
            char msg[64];
            for (uint8_t i = 0; i < count; ++i)
            {
                snprintf(msg, sizeof(msg), "  device found at 0x%02X\r\n",
                         static_cast<unsigned>(found[i]));
                Print(msg);
            }
        }
        return;
    }

    uint8_t status = 0;
    uint8_t agc = 0;
    if (!encoder.ReadStatus(status) || !encoder.ReadAgc(agc))
    {
        Print("AS5600: found, but register read failed\r\n");
        return;
    }

    char msg[96];
    snprintf(msg, sizeof(msg), "AS5600: %s (AGC %u)\r\n",
             (status & AS5600::kMagnetTooWeak)   ? "magnet too weak/far" :
             (status & AS5600::kMagnetTooStrong) ? "magnet too strong/close" :
             (status & AS5600::kMagnetDetected)  ? "magnet OK" :
                                                   "no magnet detected",
             static_cast<unsigned>(agc));
    Print(msg);
}

/* Shortest signed distance from measurement to setpoint. Without the wrap the
 * controller would drive the long way round whenever the target sits across
 * the encoder's zero crossing. */
float PositionError(uint16_t setpoint, uint16_t measurement)
{
    int32_t error = static_cast<int32_t>(setpoint) - static_cast<int32_t>(measurement);

    if (error > kCountsPerRev / 2)
    {
        error -= kCountsPerRev;
    }
    else if (error < -kCountsPerRev / 2)
    {
        error += kCountsPerRev;
    }

    return static_cast<float>(error) / kHalfRev;
}

void ReadTuningInputs()
{
    const uint32_t setpointPot = ReadAdcChannel(&hadc2, ADC_CHANNEL_5);  // PC4
    const uint32_t kpPot       = ReadAdcChannel(&hadc2, ADC_CHANNEL_4);  // PA7
    const uint32_t kiPot       = ReadAdcChannel(&hadc5, ADC_CHANNEL_1);  // PA8
    const uint32_t kdPot       = ReadAdcChannel(&hadc5, ADC_CHANNEL_2);  // PA9

    /* The pot spans the same 0..4095 range as the encoder, so the setpoint
     * maps straight across without scaling. */
    setpointCounts = static_cast<uint16_t>(setpointPot);

    gainKp = (static_cast<float>(kpPot) / kAdcFullScale) * kMaxKp;
    gainKi = (static_cast<float>(kiPot) / kAdcFullScale) * kMaxKi;
    gainKd = (static_cast<float>(kdPot) / kAdcFullScale) * kMaxKd;

    pid.SetGains(gainKp, gainKi, gainKd);
}

/* Setpoint pot straight to motor command, no encoder involved. */
void RunOpenLoopStep()
{
    if (!armed)
    {
        motor.Coast();
        lastCommand = 0.0f;
        return;
    }

    /* Remap 0..4095 to -1..+1 so the pot's centre is a stopped motor. */
    float command = (static_cast<float>(setpointCounts) / (kAdcFullScale / 2.0f)) - 1.0f;

    if (command > -kOpenLoopDeadband && command < kOpenLoopDeadband)
    {
        command = 0.0f;
    }

    lastCommand = command;
    motor.SetCommand(command);
}

void RunControlStep(float dt)
{
    if (kOpenLoopTest)
    {
        /* Still read the encoder so the log shows position, but never let a
         * failure stop the test. */
        uint16_t probe = 0;
        encoderHealthy = encoder.ReadAngle(probe);
        if (encoderHealthy)
        {
            measuredCounts = probe;
        }

        RunOpenLoopStep();
        return;
    }

    uint16_t angleCounts = 0;
    if (!encoder.ReadAngle(angleCounts))
    {
        if (encoderFailures < kMaxEncoderFailures)
        {
            ++encoderFailures;
        }

        if (encoderFailures >= kMaxEncoderFailures)
        {
            encoderHealthy = false;
            armed = false;
            motor.Coast();
            pid.Reset();
            lastCommand = 0.0f;
        }
        return;
    }

    encoderFailures = 0;
    encoderHealthy = true;
    measuredCounts = angleCounts;

    if (!armed)
    {
        motor.Coast();
        pid.Reset();
        lastCommand = 0.0f;
        return;
    }

    const float error = PositionError(setpointCounts, measuredCounts);
    const float measurement = static_cast<float>(measuredCounts) / kHalfRev;

    lastCommand = pid.Update(error, measurement, dt);
    motor.SetCommand(lastCommand);
}

void PrintStatus()
{
    /* Gains print as thousandths because nano.specs leaves float support out
     * of printf; the same trick keeps the command in per-mille. */
    char msg[160];
    const int len = snprintf(
        msg, sizeof(msg),
        "%s SP:%4u POS:%4u CMD:%5d  Kp:%4lu Ki:%4lu Kd:%4lu%s\r\n",
        armed ? "RUN " : "STOP",
        static_cast<unsigned>(setpointCounts),
        static_cast<unsigned>(measuredCounts),
        static_cast<int>(lastCommand * 1000.0f),
        static_cast<unsigned long>(gainKp * 1000.0f),
        static_cast<unsigned long>(gainKi * 1000.0f),
        static_cast<unsigned long>(gainKd * 1000.0f),
        encoderHealthy ? "" : "  ENC FAULT");

    HAL_UART_Transmit(&hcom_uart[COM1], reinterpret_cast<uint8_t*>(msg), len, 1000);
}

}  // namespace

/* Overrides the BSP's weak handler; the EXTI chain for the blue user button
 * is already wired up by BSP_PB_Init(). */
extern "C" void BSP_PB_Callback(Button_TypeDef Button)
{
    if (Button != BUTTON_USER)
    {
        return;
    }

    /* Refuse to arm against an encoder that is not reporting, unless the
     * encoder is deliberately out of the loop for a wiring test. */
    if (!armed && !encoderHealthy && !kOpenLoopTest)
    {
        return;
    }

    armed = !armed;
}

void App_Init()
{
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);

    motor.Begin();
    motor.Coast();

    pid.SetOutputLimits(-1.0f, 1.0f);
    pid.Reset();

    ReportEncoderHealth();
    ReadTuningInputs();

    /* Seed the health flag so the first button press can arm. */
    uint16_t angleCounts = 0;
    encoderHealthy = encoder.ReadAngle(angleCounts);
    if (encoderHealthy)
    {
        measuredCounts = angleCounts;
    }

    if (kOpenLoopTest)
    {
        Print("Mode: OPEN LOOP - setpoint pot drives the motor directly.\r\n");
    }
    else
    {
        Print("Mode: closed loop position control.\r\n");
        if (!encoderHealthy)
        {
            Print("  encoder not reporting, so arming is blocked\r\n");
        }
    }

    Print("Press the blue user button to arm/disarm the motor.\r\n");
}

void App_Run(void)
{
    static uint32_t nextControlTick = 0;
    static uint32_t lastControlTick = 0;
    static uint32_t nextTuningTick = 0;
    static uint32_t nextPrintTick = 0;

    const uint32_t now = HAL_GetTick();

    if (static_cast<int32_t>(now - nextControlTick) >= 0)
    {
        nextControlTick = now + kControlPeriodMs;

        /* Measured rather than assumed: the blocking serial write below steals
         * several milliseconds every print, and feeding the PID a dt it did
         * not actually wait would distort the I and D terms each time. */
        const uint32_t elapsed = now - lastControlTick;
        lastControlTick = now;

        RunControlStep(static_cast<float>(elapsed) / 1000.0f);
    }

    if (static_cast<int32_t>(now - nextTuningTick) >= 0)
    {
        nextTuningTick = now + kTuningPeriodMs;
        ReadTuningInputs();
    }

    if (static_cast<int32_t>(now - nextPrintTick) >= 0)
    {
        nextPrintTick = now + kPrintPeriodMs;
        BSP_LED_Toggle(LED_GREEN);
        PrintStatus();

        if (!encoderHealthy)
        {
            ReportEncoderHealth();
        }
    }
}
