# biped cli

software toolkit for a bipedal desk robot. handles gait generation, inverse kinematics,
servo control, vision tracking, and parameter tuning.

## getting started

```bash
cd biped-cli
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## commands

```bash
# see everything available
python -m biped --help

# send commands to the robot over usb serial
python -m biped run --port /dev/cu.usbmodem* --mode stand
python -m biped run --port /dev/cu.usbmodem* --mode walk
python -m biped run --port /dev/cu.usbmodem* --mode walk --yaw 0.3
python -m biped run --port /dev/cu.usbmodem* --mode sweep --channel 0
python -m biped run --port /dev/cu.usbmodem* --mode center

# visualize the gait in a matplotlib window
python -m biped gait --walk
python -m biped gait --stand

# tune gait parameters with sliders and live animation
python -m biped tune

# run person detection and tracking
python -m biped vision
python -m biped vision --source path/to/video.mp4

# show current servo calibration values
python -m biped calibrate

# save your current config
python -m biped export --output my_robot.yaml
```

## project layout

```
biped-cli/
├── biped/
│   ├── __main__.py          cli entry point
│   ├── kinematics.py        inverse and forward kinematics (4 dof per leg)
│   ├── gait.py              walking gait generator with differential steering
│   ├── balance.py           pid balance controller using imu feedback
│   ├── servo.py             angle to pwm mapping for ds3240 servos
│   ├── tuner.py             interactive gui for tuning gait parameters
│   ├── vision.py            person detection and tracking with yolo
│   ├── comms.py             serial link to daisy seed + udp for vision
│   └── config.py            loads and merges yaml/json configs
├── firmware/
│   ├── src/main.cpp         daisy seed firmware (serial > pca9685 > servos)
│   ├── src/pca9685.cpp      i2c pwm driver
│   └── Makefile
├── configs/
│   └── default.yaml         all robot and gait parameters (commented)
├── tests/                   ik, gait, and balance tests
├── scripts/
│   └── servo_test.py        standalone servo sweep utility
├── SETUP.md                 full hardware setup and calibration guide
└── README.md
```

## hardware

- 8x ds3240 servos for legs (4 per leg) at 6v
- 4x ds3240 servos for head/neck (optional)
- pca9685 pwm driver board over i2c
- daisy seed (stm32h7) as the servo controller
- bno055 or mpu6050 imu for balance (optional, not needed for initial walking)
- 3s lipo with separate servo and logic power

## how the legs work

each leg has 4 joints chained together:

1. **hip roll** — the internal hip joint, swings the leg in and out laterally (x axis)
2. **hip pitch** — the external hip joint, swings the leg forward and backward (y axis)
3. **knee pitch** — bends the knee (y axis)
4. **ankle pitch** — tilts the foot to keep the sole flat on the ground (y axis)

the inverse kinematics solver in `kinematics.py` figures out the angles for all four
joints given a target foot position. the gait generator in `gait.py` produces those
foot positions over time to make the robot walk.

turning works by giving each leg a different step length. the leg on the inside of the
turn takes smaller steps while the outside leg takes bigger ones.

## config

everything lives in `configs/default.yaml` with comments explaining what each value
does. all lengths are in meters, angles in radians, times in seconds.

the most important things to get right:
- your actual leg measurements (`upper_leg`, `lower_leg`, `foot_height`)
- servo center pulse widths (`center_us`) — found by testing each servo
- servo directions (`dir`) — flip to -1 if a joint moves the wrong way

see `SETUP.md` for the full calibration walkthrough.
