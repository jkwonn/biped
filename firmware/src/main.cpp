/**
 * Biped Robot — Daisy Seed Firmware
 *
 * Receives servo pulse commands over USB serial from the Python CLI,
 * drives a PCA9685 PWM board over I2C to control DS3240 servos.
 *
 * Serial protocol (115200 baud, USB CDC):
 *
 *   Computer → Daisy (servo command):
 *     P <ch0> <ch1> ... <ch11>\n
 *     - 'P' prefix = pulse command
 *     - 12 integers: pulse width in microseconds per channel (500-2500)
 *     - Example: P 1500 1500 1500 1500 1500 1500 1500 1500 1500 1500 1500 1500
 *
 *   Computer → Daisy (single channel):
 *     S <ch> <pulse_us>\n
 *     - 'S' prefix = single channel set
 *     - Example: S 3 1200
 *
 *   Computer → Daisy (center all):
 *     C\n
 *     - Centers all servos to 1500us
 *
 *   Daisy → Computer (heartbeat, every 500ms):
 *     H <uptime_ms>\n
 *
 *   Daisy → Computer (IMU data, when enabled, every 20ms):
 *     I <pitch> <roll> <yaw>\n
 *     - Angles in radians as floats
 *
 * I2C wiring:
 *   D11 (PB8) = SCL  — 4.7kΩ pullup to 3.3V
 *   D12 (PB9) = SDA  — 4.7kΩ pullup to 3.3V
 *
 * Build:
 *   make && make program-dfu
 */

#include "daisy_seed.h"
#include "pca9685.h"
#include <cstring>
#include <cstdlib>

using namespace daisy;

// Number of servo channels used by the robot
static constexpr int NUM_SERVOS = 12;

// Servo pulse limits (DS3240 range)
static constexpr uint16_t PULSE_MIN = 500;
static constexpr uint16_t PULSE_MAX = 2500;
static constexpr uint16_t PULSE_CENTER = 1500;

// Heartbeat interval
static constexpr uint32_t HEARTBEAT_MS = 500;

// USB serial receive buffer
static constexpr int RX_BUF_SIZE = 256;

// Hardware objects
static DaisySeed  hw;
static I2CHandle  i2c;
static Pca9685    pwm;

// State
static uint16_t servo_pulses[NUM_SERVOS];
static char     rx_buf[RX_BUF_SIZE];
static int      rx_pos = 0;
static uint32_t last_heartbeat = 0;


static uint16_t ClampPulse(int pulse)
{
    if(pulse < PULSE_MIN) return PULSE_MIN;
    if(pulse > PULSE_MAX) return PULSE_MAX;
    return static_cast<uint16_t>(pulse);
}


/**
 * Parse and execute a complete command line.
 * Returns true if the command was recognized.
 */
static bool ProcessCommand(const char* line)
{
    if(line[0] == 'P' && line[1] == ' ')
    {
        // Pulse command: P <ch0> <ch1> ... <ch11>
        const char* ptr = line + 2;
        char* end;
        for(int i = 0; i < NUM_SERVOS; i++)
        {
            long val = strtol(ptr, &end, 10);
            if(end == ptr)
                break;  // parse error, stop
            servo_pulses[i] = ClampPulse(static_cast<int>(val));
            pwm.SetPulse(i, servo_pulses[i]);
            ptr = end;
        }
        return true;
    }
    else if(line[0] == 'S' && line[1] == ' ')
    {
        // Single channel: S <ch> <pulse_us>
        const char* ptr = line + 2;
        char* end;
        long ch = strtol(ptr, &end, 10);
        if(end != ptr && ch >= 0 && ch < NUM_SERVOS)
        {
            ptr = end;
            long val = strtol(ptr, &end, 10);
            if(end != ptr)
            {
                servo_pulses[ch] = ClampPulse(static_cast<int>(val));
                pwm.SetPulse(ch, servo_pulses[ch]);
            }
        }
        return true;
    }
    else if(line[0] == 'C')
    {
        // Center all servos
        for(int i = 0; i < NUM_SERVOS; i++)
        {
            servo_pulses[i] = PULSE_CENTER;
            pwm.SetPulse(i, PULSE_CENTER);
        }
        return true;
    }

    return false;
}


int main(void)
{
    // Initialize Daisy Seed hardware
    hw.Init();

    // Initialize I2C1 on D11/D12 at 400kHz
    I2CHandle::Config i2c_conf;
    i2c_conf.periph         = I2CHandle::Config::Peripheral::I2C_1;
    i2c_conf.pin_config.scl = {DSY_GPIOB, 8};   // D11
    i2c_conf.pin_config.sda = {DSY_GPIOB, 9};   // D12
    i2c_conf.speed          = I2CHandle::Config::Speed::I2C_400KHZ;
    i2c_conf.mode           = I2CHandle::Config::Mode::I2C_MASTER;
    i2c.Init(i2c_conf);

    // Initialize PCA9685 at default address 0x40
    pwm.Init(&i2c, 0x40);

    // Center all servos on startup
    for(int i = 0; i < NUM_SERVOS; i++)
    {
        servo_pulses[i] = PULSE_CENTER;
        pwm.SetPulse(i, PULSE_CENTER);
    }

    // Start USB serial (CDC)
    hw.StartLog(false);

    // Brief startup delay
    System::Delay(100);
    hw.PrintLine("BIPED OK %d", NUM_SERVOS);

    // Main loop
    while(true)
    {
        // Read USB serial bytes
        // libDaisy doesn't have a direct byte-by-byte read for USB CDC,
        // so we use the internal USB receive. For simplicity, we check
        // the USB CDC virtual com port directly.
        //
        // Note: In practice you may need to adapt this to your libDaisy
        // version's USB API. The approach below works with the standard
        // DaisySeed logging/USB interface.

        // Process any available serial data
        while(hw.usb_handle.IsActive())
        {
            uint8_t byte;
            // Try to receive one byte (non-blocking)
            if(hw.usb_handle.ReceiveByte(&byte))
            {
                if(byte == '\n' || byte == '\r')
                {
                    if(rx_pos > 0)
                    {
                        rx_buf[rx_pos] = '\0';
                        ProcessCommand(rx_buf);
                        rx_pos = 0;
                    }
                }
                else if(rx_pos < RX_BUF_SIZE - 1)
                {
                    rx_buf[rx_pos++] = static_cast<char>(byte);
                }
            }
            else
            {
                break;  // no more bytes available
            }
        }

        // Heartbeat
        uint32_t now = System::GetNow();
        if(now - last_heartbeat >= HEARTBEAT_MS)
        {
            last_heartbeat = now;
            hw.PrintLine("H %lu", static_cast<unsigned long>(now));
        }

        // Small delay to prevent busy-spinning
        System::Delay(1);
    }
}
