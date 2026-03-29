/**
 * PCA9685 PWM driver implementation.
 *
 * Register map (relevant subset):
 *   0x00  MODE1     — oscillator, auto-increment, sleep
 *   0x01  MODE2     — output config
 *   0x06  LED0_ON_L — channel 0 ON time (low byte)
 *   0x07  LED0_ON_H — channel 0 ON time (high byte)
 *   0x08  LED0_OFF_L— channel 0 OFF time (low byte)
 *   0x09  LED0_OFF_H— channel 0 OFF time (high byte)
 *   ...             — channels 1-15 at +4 byte offsets
 *   0xFE  PRESCALE  — PWM frequency prescaler
 *
 * PWM frequency:
 *   prescale = round(25MHz / (4096 * freq)) - 1
 *   For 50Hz: prescale = round(25000000 / (4096 * 50)) - 1 = 121
 *
 * Pulse width to tick count:
 *   ticks = pulse_us * 4096 / 20000  (at 50Hz, period = 20000us)
 */

#include "pca9685.h"

// PCA9685 registers
static constexpr uint8_t REG_MODE1    = 0x00;
static constexpr uint8_t REG_MODE2    = 0x01;
static constexpr uint8_t REG_LED0     = 0x06;
static constexpr uint8_t REG_PRESCALE = 0xFE;

// MODE1 bits
static constexpr uint8_t MODE1_SLEEP   = 0x10;
static constexpr uint8_t MODE1_AI      = 0x20;  // auto-increment
static constexpr uint8_t MODE1_RESTART = 0x80;

// 50Hz for servos: prescale = round(25MHz / (4096 * 50)) - 1
static constexpr uint8_t PRESCALE_50HZ = 121;

// At 50Hz, one period = 20000us, 4096 ticks per period
static constexpr float US_PER_TICK = 20000.0f / 4096.0f;


void Pca9685::Init(daisy::I2CHandle* i2c, uint8_t addr)
{
    i2c_ = i2c;
    addr_ = addr;

    // Put to sleep before changing prescaler (required by datasheet)
    WriteReg(REG_MODE1, MODE1_SLEEP);

    // Set prescaler for 50Hz
    WriteReg(REG_PRESCALE, PRESCALE_50HZ);

    // Wake up with auto-increment enabled
    WriteReg(REG_MODE1, MODE1_AI);

    // Wait 500us for oscillator to stabilize (datasheet requirement)
    daisy::System::DelayUs(500);

    // Clear restart bit
    WriteReg(REG_MODE1, MODE1_AI | MODE1_RESTART);
}


void Pca9685::SetPulse(uint8_t channel, uint16_t pulse_us)
{
    if(channel > 15)
        return;

    // Convert microseconds to 12-bit tick count
    // ticks = pulse_us / US_PER_TICK = pulse_us * 4096 / 20000
    uint16_t ticks = static_cast<uint16_t>(
        static_cast<float>(pulse_us) / US_PER_TICK);

    if(ticks > 4095)
        ticks = 4095;

    // ON time = 0 (pulse starts at beginning of cycle)
    // OFF time = ticks (pulse ends after 'ticks' counts)
    uint8_t reg = REG_LED0 + 4 * channel;
    uint8_t data[4];
    data[0] = 0x00;                        // ON_L
    data[1] = 0x00;                        // ON_H
    data[2] = ticks & 0xFF;                // OFF_L
    data[3] = (ticks >> 8) & 0x0F;         // OFF_H

    i2c_->WriteDataAtAddress(addr_, &reg, 1, data, 4, 100);
}


void Pca9685::SetAllPulse(uint16_t pulse_us)
{
    for(uint8_t ch = 0; ch < 16; ch++)
    {
        SetPulse(ch, pulse_us);
    }
}


void Pca9685::WriteReg(uint8_t reg, uint8_t value)
{
    i2c_->WriteDataAtAddress(addr_, &reg, 1, &value, 1, 100);
}


uint8_t Pca9685::ReadReg(uint8_t reg)
{
    uint8_t value = 0;
    i2c_->ReadDataAtAddress(addr_, &reg, 1, &value, 1, 100);
    return value;
}
