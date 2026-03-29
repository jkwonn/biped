# Bipedal Desk Robot — CLI Toolkit

Software toolkit for a bipedal desk robot with DS3240 servos, BNO055/MPU6050 IMU,
ESP32-CAM vision, and Daisy Seed controller.

## Setup

```bash
cd biped-cli
python -m venv venv
source venv/bin/activate  # macOS/Linux
pip install -r requirements.txt
```

## Commands

```bash
# Show all available commands
python -m biped --help

# Gait visualization (animated stick figure)
python -m biped gait --walk
python -m biped gait --stand

# Tune parameters interactively with live visualization
python -m biped tune

# Run vision tracking pipeline (webcam)
python -m biped vision
python -m biped vision --source path/to/video.mp4
python -m biped vision --source http://192.168.1.100:81/stream

# Export YOLO model to ONNX for ZimaBoard deployment
python -m biped vision --export-onnx

# Calibrate servos interactively
python -m biped calibrate

# Run mock UDP receiver (test command pipeline)
python -m biped mock-receiver --port 8888

# Export current parameters to YAML
python -m biped export --output my_robot.yaml

# Load custom parameters
python -m biped gait --walk --config configs/my_robot.yaml
```

## Project Structure

```
biped-cli/
├── biped/
│   ├── __init__.py
│   ├── __main__.py          # CLI entry point
│   ├── kinematics.py        # IK/FK solver (4-DOF: hip_roll → hip_pitch → knee → ankle)
│   ├── gait.py              # Parametric gait generator
│   ├── balance.py           # IMU-based balance controller
│   ├── servo.py             # Servo mapping and calibration
│   ├── vision.py            # Person detection and tracking
│   ├── comms.py             # UDP command sender/receiver
│   ├── tuner.py             # Interactive parameter tuning GUI
│   └── config.py            # Parameter management (YAML/JSON)
├── configs/
│   └── default.yaml         # Default robot/gait parameters (commented)
├── scripts/
│   └── servo_test.py        # Standalone servo sweep test
├── requirements.txt
└── README.md
```

## Hardware

- 12x DS3240 servos (8 leg + 4 head/neck) at 6V via high-current buck
- PCA9685 16-channel PWM driver over I2C
- BNO055 or MPU6050 IMU over I2C
- ESP32-CAM with OV5640 for MJPEG streaming
- Daisy Seed (STM32H7) as locomotion controller
- 3S LiPo with separate servo/logic power rails

## Leg Joint Layout (4 DOF per leg)

Each leg has four joints in series:

1. **Hip roll** — internal hip joint, rotates around X axis (forward axis), swings leg in/out laterally
2. **Hip pitch** — external hip joint, rotates around Y axis (lateral axis), swings leg forward/backward
3. **Knee pitch** — rotates around Y axis (lateral axis), bends knee
4. **Ankle pitch** — rotates around Y axis (lateral axis), tilts foot to keep sole flat

## Configuration

All parameters live in `configs/default.yaml` with inline comments explaining
units and purpose. Edit this file to match your robot's measurements and
servo calibration.

Key things to calibrate:
- Leg segment lengths (`upper_leg`, `lower_leg`, `foot_height`)
- Servo center pulse widths (`center_us`) — found experimentally per servo
- Servo directions (`dir`) — flip to `-1` if a joint moves the wrong way
