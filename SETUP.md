# Hardware Setup & Verification Guide

Step-by-step instructions from fresh hardware to walking robot.

## Prerequisites

**Hardware:**
- Daisy Seed (STM32H7) microcontroller
- PCA9685 16-channel PWM driver board
- 8x DS3240 servo motors (4 per leg)
- 4x DS3240 servos for head/neck (optional)
- BNO055 or MPU6050 IMU (optional, for balance — not needed for initial walking)
- 6V high-current power supply or BEC for servos (DS3240 draws up to 3A stall per servo)
- 3.3V logic power for Daisy Seed (USB or separate regulator)
- USB cable for Daisy Seed to computer

**Software:**
- Python 3.10+ with venv
- ARM GCC toolchain (for Daisy Seed firmware)
- libDaisy (Electro-Smith Daisy library)

## Phase 0: Software Setup

```bash
cd biped-cli
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Verify everything works
python -m pytest tests/ -v
python -m biped gait --stand
```

## Phase 1: Wiring

```
                        6V servo power
                             │
  Computer ──USB── Daisy Seed ──I2C── PCA9685 ──PWM── DS3240 servos
                     (3.3V)          (addr 0x40)       (x12 channels)
                       │
                       └──I2C── IMU (optional, same bus or I2C2)

  I2C connections (PCA9685 and optional IMU):
    Daisy D11 (PB8) ── SCL ── 4.7kΩ pullup to 3.3V
    Daisy D12 (PB9) ── SDA ── 4.7kΩ pullup to 3.3V
    GND ── GND (shared between Daisy, PCA9685, and servos)
```

**PCA9685 channel assignment** (must match `configs/default.yaml`):

```
  CH 0  ── left_hip_roll      CH 4  ── right_hip_roll
  CH 1  ── left_hip_pitch     CH 5  ── right_hip_pitch
  CH 2  ── left_knee_pitch    CH 6  ── right_knee_pitch
  CH 3  ── left_ankle_pitch   CH 7  ── right_ankle_pitch
  CH 8  ── head_pan           CH 10 ── neck_pan
  CH 9  ── head_tilt          CH 11 ── neck_tilt
```

**Power warning:** DO NOT power the DS3240 servos from the Daisy Seed's 3.3V rail.
They need 6V at high current. Use a separate BEC or bench supply connected to the
PCA9685's V+ terminal. Share GND between all boards.

## Phase 2: Flash Firmware

```bash
cd firmware

# First time: clone libDaisy and build it
git clone https://github.com/electro-smith/libDaisy.git
cd libDaisy && make -j && cd ..

# Build and flash firmware
make
make program-dfu
```

After flashing, the Daisy Seed will appear as a USB serial device. Verify:
```bash
# macOS
ls /dev/cu.usbmodem*

# Linux
ls /dev/ttyACM*
```

## Phase 3: Servo Calibration

This is the most important step. Bad calibration = broken robot.

### 3a. Find center pulse widths

With the robot frame assembled but servo horns NOT attached:

```bash
# Center all servos at 1500us (default)
python -m biped run --mode stand --port /dev/cu.usbmodem*
```

For each servo:
1. The servo moves to 1500us (nominal center)
2. Attach the servo horn so the joint is at its zero/neutral position:
   - Hip roll: leg hanging straight down
   - Hip pitch: upper leg pointing straight down
   - Knee: leg fully straight (upper and lower leg aligned)
   - Ankle: foot sole perpendicular to lower leg
3. If the horn spline doesn't allow perfect alignment, note the offset

If you can't get a perfect zero with the horn, adjust `center_us` in the config:
```bash
# Test a specific channel interactively
python -m biped run --mode sweep --port /dev/cu.usbmodem* --channel 0
```

Write each `center_us` value into `configs/default.yaml`.

### 3b. Find servo directions

For each servo:
1. Command a small positive angle (+0.3 rad ≈ +17 deg)
2. Watch which direction the joint moves
3. Expected directions:
   - `hip_roll` positive → leg moves outward (away from body)
   - `hip_pitch` positive → leg moves backward
   - `knee_pitch` positive → knee bends (lower leg moves backward)
   - `ankle_pitch` positive → foot tilts toes up
4. If the joint moves opposite to expected: set `dir: -1`

Left and right legs are mirrored, so corresponding joints will typically have
opposite `dir` values.

### 3c. Verify stand pose

```bash
python -m biped run --mode stand --port /dev/cu.usbmodem*
```

The robot should stand with both legs straight-ish under the body. Compare the
physical pose to the simulation:

```bash
python -m biped gait --stand
```

If any joint looks wrong, recheck `center_us` and `dir` for that channel.

## Phase 4: First Steps

Start SLOW. Increase speed only after stable walking is confirmed.

### 4a. Tune standing stability

In `configs/default.yaml`, adjust:
- `stand_height`: lower = more bent knees = more stable but weaker servos.
  Start around 70-80% of max leg reach.
- Hold the robot by hand and verify it can support its own weight.

### 4b. Tune walking gait

```bash
python -m biped tune
```

In the tuner GUI:
1. Set `step_period` HIGH (2.0-3.0s) — start very slow
2. Set `step_length` LOW (1.0-2.0cm) — small steps
3. Set `lateral_sway` to roughly the hip offset (2-4cm) — needs to shift weight
4. Click "Walk" and watch the simulation
5. When the sim looks reasonable, try on hardware:

```bash
python -m biped run --mode walk --port /dev/cu.usbmodem*
```

**Hold the robot by hand for the first tests.** Only let go when it's clearly
shifting weight and not tipping.

### 4c. Iterate

Gradually:
- Decrease `step_period` (faster walking)
- Increase `step_length` (bigger steps)
- Fine-tune `lateral_sway` and `sway_advance` (weight shift timing)
- Adjust `duty_factor` (time spent on ground vs in air)

### 4d. Test turning

```bash
python -m biped run --mode walk --port /dev/cu.usbmodem* --yaw 0.3
```

Positive yaw = turn left, negative = turn right. Start with small values (0.1-0.3).

## Phase 5: Balance (Optional, after walking works)

Once open-loop walking is stable, add IMU feedback:
1. Wire BNO055 or MPU6050 to the same I2C bus (or I2C2)
2. Enable IMU in firmware config
3. The firmware sends pitch/roll data back over serial
4. `balance.py` computes corrective offsets added to gait angles

This is not needed for initial walking — do it after open-loop gait works.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Servo jitters at rest | Insufficient power supply current, or servo fighting against mechanical stop |
| Robot leans to one side | `center_us` wrong on hip_roll, or `lateral_sway` / `sway_advance` needs tuning |
| Foot drags during swing | `step_height` too low, or `step_period` too fast for servo speed |
| Robot falls forward/backward | `forward_lean` too high, or `stand_height` wrong |
| Serial connection drops | USB power issue — use powered hub, or check cable |
| "IK unreachable" warning | `stand_height` is set higher than the legs can reach |

## Config Quick Reference

All lengths in **meters**, angles in **radians**, times in **seconds**.

```
upper_leg + lower_leg + foot_height  >  stand_height  (or IK fails)
step_period × servo_speed  >  step arc  (or servos can't keep up)
```
