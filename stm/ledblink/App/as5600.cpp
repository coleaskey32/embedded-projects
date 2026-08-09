#include "as5600.h"

bool AS5600::IsPresent() const
{
    return HAL_I2C_IsDeviceReady(bus, kDeviceAddress, 3, kTimeoutMs) == HAL_OK;
}

uint8_t AS5600::ScanBus(uint8_t* found, uint8_t maxFound) const
{
    uint8_t count = 0;

    /* 7-bit addresses 0x08..0x77; the rest are reserved by the I2C spec. */
    for (uint8_t address = 0x08; address <= 0x77 && count < maxFound; ++address)
    {
        if (HAL_I2C_IsDeviceReady(bus, static_cast<uint16_t>(address << 1),
                                  1, 2) == HAL_OK)
        {
            found[count++] = address;
        }
    }

    return count;
}

bool AS5600::ReadRegister8(uint8_t reg, uint8_t& value) const
{
    return HAL_I2C_Mem_Read(bus, kDeviceAddress, reg, I2C_MEMADD_SIZE_8BIT,
                            &value, 1, kTimeoutMs) == HAL_OK;
}

bool AS5600::ReadRegister12(uint8_t reg, uint16_t& value) const
{
    /* The AS5600 auto-increments, so the high and low halves of a 12-bit
     * value come back in a single transaction. */
    uint8_t raw[2] = {};
    if (HAL_I2C_Mem_Read(bus, kDeviceAddress, reg, I2C_MEMADD_SIZE_8BIT,
                         raw, 2, kTimeoutMs) != HAL_OK)
    {
        return false;
    }

    value = static_cast<uint16_t>(((raw[0] & 0x0F) << 8) | raw[1]);
    return true;
}

bool AS5600::WriteRegister8(uint8_t reg, uint8_t value) const
{
    return HAL_I2C_Mem_Write(bus, kDeviceAddress, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, kTimeoutMs) == HAL_OK;
}

bool AS5600::WriteRegister16(uint8_t reg, uint16_t value) const
{
    uint8_t raw[2] = {
        static_cast<uint8_t>((value >> 8) & 0x0F),
        static_cast<uint8_t>(value & 0xFF),
    };

    return HAL_I2C_Mem_Write(bus, kDeviceAddress, reg, I2C_MEMADD_SIZE_8BIT,
                             raw, 2, kTimeoutMs) == HAL_OK;
}

bool AS5600::ReadAngle(uint16_t& counts) const
{
    return ReadRegister12(kRegAngle, counts);
}

bool AS5600::ReadRawAngle(uint16_t& counts) const
{
    return ReadRegister12(kRegRawAngle, counts);
}

bool AS5600::ReadStatus(uint8_t& status) const
{
    return ReadRegister8(kRegStatus, status);
}

bool AS5600::ReadAgc(uint8_t& agc) const
{
    return ReadRegister8(kRegAgc, agc);
}

bool AS5600::ReadMagnitude(uint16_t& magnitude) const
{
    return ReadRegister12(kRegMagnitude, magnitude);
}

bool AS5600::SetZeroPosition(uint16_t counts) const
{
    if (!WriteRegister16(kRegZpos, counts))
    {
        return false;
    }

    /* The datasheet asks for a settling delay after a position write. */
    HAL_Delay(2);
    return true;
}

bool AS5600::SetMaxPosition(uint16_t counts) const
{
    if (!WriteRegister16(kRegMpos, counts))
    {
        return false;
    }

    HAL_Delay(2);
    return true;
}

bool AS5600::WriteConfig(uint16_t config) const
{
    return WriteRegister16(kRegConf, config);
}
