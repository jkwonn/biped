# setup guide

how to go from a pile of parts to a walking robot.

## what you need

**hardware**
- daisy seed microcontroller
- pca9685 pwm driver board
- 8x ds3240 servos for legs (4 per leg)
- 4x ds3240 servos for head/neck if you want those
- 6v power supply that can handle the current (ds3240s pull up to 3a each at stall)
- usb cable to connect the daisy seed to your computer
- optionally a bno055 or mpu6050 imu for balance later

**software**
- python 3.10+
- arm gcc toolchain for building the daisy firmware
- libdaisy (gets cloned during the firmware build)

## step 1: install the python side

```bash
cd biped-cli
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# make sure everything works
python -m pytest tests/ -v
python -m biped gait --stand
```

## step 2: wire it up

```
                        6v servo power
                             │
  computer ──usb── daisy seed ──i2c── pca9685 ──pwm── servos (x12)
                     (3.3v)          (addr 0x40)
                       │
                       └──i2c── imu (optional, same bus or i2c2)
```

for the i2c connections:
- daisy pin d11 (pb8) is scl, needs a 4.7k pullup to 3.3v
- daisy pin d12 (pb9) is sda, needs a 4.7k pullup to 3.3v
- all boards share a common ground

the pca9685 channel assignments need to match `configs/default.yaml`:

```
  ch 0  left_hip_roll        ch 4  right_hip_roll
  ch 1  left_hip_pitch       ch 5  right_hip_pitch
  ch 2  left_knee_pitch      ch 6  right_knee_pitch
  ch 3  left_ankle_pitch     ch 7  right_ankle_pitch
  ch 8  head_pan             ch 10 neck_pan
  ch 9  head_tilt            ch 11 neck_tilt
```

**important:** don't try to power the servos from the daisy's 3.3v rail. they need
6v at high current. use a separate supply connected to the pca9685's v+ screw terminal.

## step 3: flash the firmware

```bash
cd firmware

# first time only: grab libdaisy and build it
git clone https://github.com/electro-smith/libDaisy.git
cd libDaisy && make -j && cd ..

# build and flash
make
make program-dfu
```

after flashing, the daisy should show up as a usb serial device:
```bash
ls /dev/cu.usbmodem*     # macos
ls /dev/ttyACM*           # linux
```

## step 4: calibrate the servos

this is the most tedious part but also the most important. if calibration is off,
nothing else will work right.

### find the center positions

assemble the robot frame but don't attach the servo horns yet. then center all servos:

```bash
python -m biped run --mode center --port /dev/cu.usbmodem*
```

now for each servo:
1. the servo moves to 1500us (its default center)
2. attach the horn so the joint is at its natural zero position:
   - hip roll: leg hanging straight down
   - hip pitch: upper leg pointing straight down
   - knee: fully straight
   - ankle: foot perpendicular to the lower leg
3. if the horn spline doesn't let you get a perfect zero, tweak `center_us` in
   the config until it does

you can test individual channels with:
```bash
python -m biped run --mode sweep --port /dev/cu.usbmodem* --channel 0
```

### find the servo directions

for each servo, send a small positive angle and see which way it moves:
- hip_roll positive should move the leg outward, away from the body
- hip_pitch positive should move the leg backward
- knee_pitch positive should bend the knee
- ankle_pitch positive should tilt the toes up

if a joint moves the opposite way, set `dir: -1` for that channel in the config.
left and right legs are mirrored so they'll usually have opposite dir values.

### check the stand pose

```bash
python -m biped run --mode stand --port /dev/cu.usbmodem*
```

the robot should stand with both legs roughly straight under the body. compare it
to what the simulator shows:

```bash
python -m biped gait --stand
```

if something looks off, go back and check `center_us` and `dir` for that joint.

## step 5: get it walking

start slow. you can always speed up later.

### tune the standing height

in `configs/default.yaml`, set `stand_height` to about 70-80% of your total leg
reach (upper_leg + lower_leg). hold the robot by hand and make sure it can support
its own weight without the servos straining.

### tune the gait

open the tuner:
```bash
python -m biped tune
```

start with conservative settings:
- step_period high (2-3 seconds per cycle)
- step_length small (1-2cm)
- lateral_sway around 2-4cm (needs to shift weight over the stance leg)

click walk and watch the simulation. when it looks reasonable, try it on hardware:

```bash
python -m biped run --mode walk --port /dev/cu.usbmodem*
```

**hold the robot the first few times.** only let go once you can see it shifting
weight properly and not tipping over.

### speed it up gradually

once slow walking works:
- lower step_period for faster steps
- increase step_length for bigger steps
- fine tune lateral_sway and sway_advance for smoother weight shifting
- adjust duty_factor if the foot is spending too much or too little time in the air

### try turning

```bash
python -m biped run --mode walk --port /dev/cu.usbmodem* --yaw 0.3
```

positive yaw turns left, negative turns right. start with small values like 0.1-0.3.

## step 6: add balance (after walking works)

once the robot can walk without falling over in open loop, you can add imu feedback:

1. wire a bno055 or mpu6050 to the i2c bus
2. enable imu reading in the firmware
3. the firmware sends pitch/roll back over serial
4. the balance controller in `balance.py` computes corrections that get added to the gait angles

you don't need this for initial walking. get the gait working first.

## troubleshooting

| what's happening | probably why |
|---|---|
| servo jitters when standing still | not enough current from the power supply, or servo is fighting a mechanical stop |
| robot leans to one side | center_us is off on a hip_roll servo, or lateral_sway needs tuning |
| foot drags during swing phase | step_height is too low, or the step_period is faster than the servos can move |
| robot tips forward or backward | forward_lean is too high, or stand_height needs adjusting |
| serial connection drops | usb power issue, try a powered hub or different cable |
| "ik unreachable" warning | stand_height is set higher than the legs can physically reach |

## quick reference

all lengths are in meters, angles in radians, times in seconds.

your stand_height needs to be less than upper_leg + lower_leg + foot_height,
otherwise the ik solver can't reach the ground.
