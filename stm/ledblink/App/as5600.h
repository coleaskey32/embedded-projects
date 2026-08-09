#pragma once

#include "main.h"

/* AMS AS5600 12-bit magnetic rotary encoder.
 *
 * Every call is a blocking I2C transfer and returns false if the transfer
 * failed, which on this bus almost always means the device did not ACK. */
class AS5600
{
public:
    /* Bits in the value returned by ReadStatus(). */
    static constexpr uint8_t kMagnetTooStrong = 0x08;
    static constexpr uint8_t kMagnetTooWeak   = 0x10;
    static constexpr uint8_t kMagnetDetected  = 0x20;

    explicit AS5600(I2C_HandleTypeDef* bus) : bus(bus) {}

    /* Probes for an ACK without reading anything. */
    bool IsPresent() const;

    /* Walks every valid 7-bit address and records the ones that answer, which
     * separates "nothing on the bus at all" from "device at an address other
     * than the one this driver targets". Returns how many were found. */
    uint8_t ScanBus(uint8_t* found, uint8_t maxFound) const;

    /* 0..4095 over a full turn, after the zero position and angular range
     * settings are applied. ReadRawAngle() skips that scaling. */
    bool ReadAngle(uint16_t& counts) const;
    bool ReadRawAngle(uint16_t& counts) const;

    /* Magnet diagnostics: status carries the kMagnet* bits, AGC sits near the
     * middle of 0..255 at the right magnet distance, magnitude is the raw
     * field strength the CORDIC saw. */
    bool ReadStatus(uint8_t& status) const;
    bool ReadAgc(uint8_t& agc) const;
    bool ReadMagnitude(uint16_t& magnitude) const;

    /* Writes land in the AS5600's volatile registers, so they take effect
     * immediately and are lost on power cycle. Making them permanent needs a
     * burn command, which the part only accepts three times ever and which
     * this driver deliberately does not implement. */
    bool SetZeroPosition(uint16_t counts) const;
    bool SetMaxPosition(uint16_t counts) const;
    bool WriteConfig(uint16_t config) const;

    /* Direct register access for anything not wrapped above. */
    bool ReadRegister8(uint8_t reg, uint8_t& value) const;
    bool ReadRegister12(uint8_t reg, uint16_t& value) const;
    bool WriteRegister8(uint8_t reg, uint8_t value) const;
    bool WriteRegister16(uint8_t reg, uint16_t value) const;

    /* Register map, exposed for use with the direct access calls. */
    static constexpr uint8_t kRegZmco      = 0x00;
    static constexpr uint8_t kRegZpos      = 0x01;
    static constexpr uint8_t kRegMpos      = 0x03;
    static constexpr uint8_t kRegMang      = 0x05;
    static constexpr uint8_t kRegConf      = 0x07;
    static constexpr uint8_t kRegRawAngle  = 0x0C;
    static constexpr uint8_t kRegAngle     = 0x0E;
    static constexpr uint8_t kRegStatus    = 0x0B;
    static constexpr uint8_t kRegAgc       = 0x1A;
    static constexpr uint8_t kRegMagnitude = 0x1B;

private:
    /* The AS5600's 7-bit address is 0x36; the HAL wants it left-aligned. */
    static constexpr uint16_t kDeviceAddress = 0x36 << 1;
    static constexpr uint32_t kTimeoutMs     = 10;

    I2C_HandleTypeDef* bus;
};
