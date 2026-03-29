/**
 * PCA9685 16-channel 12-bit PWM driver over I2C.
 *
 * Simplified driver for servo control — sets PWM pulse widths in
 * microseconds. The PCA9685 runs at 50Hz (20ms period) for servos.
 *
 * Wiring:
 *   Daisy D11 (PB8) → SCL (with 4.7kΩ pullup to 3.3V)
 *   Daisy D12 (PB9) → SDA (with 4.7kΩ pullup to 3.3V)
 *   PCA9685 V+       → 6V servo power (NOT from Daisy)
 *   GND              → shared
 */

#pragma once

#include "daisy_seed.h"

class Pca9685
{
  public:
    /**
     * Initialize the PCA9685 on the given I2C peripheral.
     * Sets the PWM frequency to ~50Hz for servo control.
     *
     * @param i2c  Pointer to an initialized I2CHandle.
     * @param addr 7-bit I2C address (default 0x40).
     */
    void Init(daisy::I2CHandle* i2c, uint8_t addr = 0x40);

    /**
     * Set a channel's PWM pulse width in microseconds.
     * Typical servo range: 500-2500us.
     *
     * @param channel 0-15
     * @param pulse_us Pulse width in microseconds
     */
    void SetPulse(uint8_t channel, uint16_t pulse_us);

    /**
     * Set all 16 channels to the same pulse width.
     */
    void SetAllPulse(uint16_t pulse_us);

  private:
    daisy::I2CHandle* i2c_;
    uint8_t           addr_;

    void WriteReg(uint8_t reg, uint8_t value);
    uint8_t ReadReg(uint8_t reg);
};
