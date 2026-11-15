# robot_base_controller

A single C++ ROS 2 Jazzy package for the 4WD DDSM115 rover. It combines what
used to be three separate pieces:

| Was | Now |
| --- | --- |
| `robot_4wd_dds115_base_controller` (C++) | `base_controller_node` |
| `code_BNO085.py` (Python sample) | `bno085_imu_node` |
| `flysky_bridge` (Python) | `flysky_ibus_node` |

plus the new piece that ties them together:

| New | Purpose |
| --- | --- |
| `odom_fusion_node` | Fuses wheel odometry and the IMU into `/odom` + TF |

## Nodes

### `base_controller_node`

Drives the four DDSM115 wheels over RS485, one serial port per motor.

* Subscribes `cmd_vel` (`geometry_msgs/Twist`)
* Publishes `wheel_odom` (`nav_msgs/Odometry`), `motor_rpms`, `motor_currents`
* Ramps RPM at `accel_rpm_per_sec` so key release, watchdog timeout, and radio
  signal loss all decelerate smoothly instead of stopping dead
* Stops the motors if no `cmd_vel` arrives within `cmd_vel_timeout`
* Clamps every wheel command to `max_rpm`

It does **not** broadcast TF by default; `odom_fusion_node` owns
`odom -> base_link`. Set `publish_tf: true` only when running it standalone.

### `bno085_imu_node`

Talks to the BNO085 over `/dev/i2c-<bus>` directly — a C++ implementation of the
SHTP / SH-2 protocol, so there is no Python or Adafruit dependency at runtime.
It enables the same reports the original `code_BNO085.py` sample did.

* Publishes `imu/data` (`sensor_msgs/Imu`), `imu/mag` (`sensor_msgs/MagneticField`)
* Runs at `report_rate_hz` (100 Hz by default, verified on hardware)
* Reports the sensor's own orientation accuracy in `orientation_covariance`, so
  a badly calibrated magnetometer automatically gets de-weighted downstream

**Magnetometer calibration matters.** Straight after power-up the sensor reports
an orientation accuracy of pi radians, i.e. "heading unknown", and the fusion
node will then lean on the gyro alone. Rotate the robot through a few figure-8s
to calibrate. If motor magnetic fields make the compass unusable, set
`use_game_rotation_vector: true` — heading then comes from gyro plus
accelerometer only, which is immune to magnetic interference but drifts slowly.

### `odom_fusion_node`

An EKF over the state `[x, y, yaw, v, omega]`, unicycle motion model.

| Measurement | Source | Default |
| --- | --- | --- |
| forward speed `v` | wheel odometry | on |
| yaw rate `omega` | IMU gyro z | on |
| absolute yaw | IMU rotation vector | on |
| yaw rate `omega` | wheel odometry | **off** |

The wheel-derived yaw rate is off by default on purpose. A skid-steer 4WD
chassis scrubs its tyres in every turn, so the wheels are a poor heading source
while the IMU is a good one. The wheels contribute the forward speed, which the
IMU cannot observe. Turn `use_wheel_angular` on only if the IMU is unavailable.

* Subscribes `wheel_odom`, `imu/data`
* Publishes `odom` (`nav_msgs/Odometry`) and broadcasts `odom -> base_link`
* Service `reset_odometry` (`std_srvs/Empty`) returns the pose to the origin
* If an input goes quiet for `input_timeout`, the velocity estimate decays to
  zero rather than coasting on the motion model and running the pose away

`zero_yaw_on_start` (default true) makes the odom frame start aligned with the
robot's initial heading instead of magnetic north.

If the IMU is mounted upside down, set `invert_imu_yaw: true`; use
`imu_yaw_offset_deg` for a mounting yaw offset.

### `flysky_ibus_node`

Reads a FlySky i-BUS receiver and publishes `cmd_vel`, so the rover is drivable
from the radio.

* Publishes `cmd_vel_topic` (default `cmd_vel`) at `publish_hz` (100 Hz)
* Arming switch on `arm_channel`; when disarmed the target is forced to zero
* Publishes a stop if no valid frame arrives within `timeout_sec`
* Always takes the *newest* buffered frame, so a serial backlog never turns into
  control lag

## Running

```bash
colcon build --packages-select robot_base_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 launch robot_base_controller bringup.launch.py
```

### Manual vs autonomous

The radio bridge is optional. Everything else runs identically either way.

```bash
# manual driving from the radio (default)
ros2 launch robot_base_controller bringup.launch.py

# autonomous: no radio node, cmd_vel is left free for a planner
ros2 launch robot_base_controller bringup.launch.py flysky:=false

# both: radio publishes elsewhere so a twist mux can arbitrate
ros2 launch robot_base_controller bringup.launch.py \
    flysky_cmd_vel_topic:=cmd_vel_manual
```

The third form matters because the bridge publishes continuously at 100 Hz,
including zeros when the sticks are centred. Left on `cmd_vel` it would fight a
planner for the topic, and the base controller would act on whichever message
landed last. Give each source its own topic and let a mux decide.

Note the base controller stops the motors if nothing publishes `cmd_vel` within
`cmd_vel_timeout` (0.5 s), so in autonomous mode the planner has to publish
continuously — that watchdog is what makes a crashed planner safe.

Available launch arguments: `base`, `imu`, `fusion`, `flysky`,
`flysky_cmd_vel_topic`, `imu_static_tf`, `imu_x/y/z`, `imu_roll/pitch/yaw`,
`params_file`.

All parameters live in [`config/robot_base_controller.yaml`](config/robot_base_controller.yaml).

## Topic graph

```
        radio                     I2C bus 7
          |                           |
  flysky_ibus_node            bno085_imu_node
          |                           |
       cmd_vel                    imu/data
          |                           |
  base_controller_node               |
          |                           |
     wheel_odom  ------->  odom_fusion_node  ------->  odom + TF
```

## Hardware notes

* Jetson Orin NX header pins 3 (SDA) and 5 (SCL) are **I2C bus 7**; the BNO085
  answers at `0x4A`. Your user must be in the `i2c` group.
* Motors are the four channels of a Waveshare USB TO 4CH RS485 (WCH CH344),
  in the order front-left, front-right, rear-left, rear-right; the radio
  receiver is a separate CH340-type USB serial. The config addresses all of
  them by `/dev/serial/by-id/...` paths because the raw `/dev/ttyACM*`
  numbers depend on USB enumeration order and have shuffled in practice
  (the radio adapter has appeared as both `ttyACM6` and `ttyACM4`).
* The serial ports need `dialout` group membership.

## Verified on hardware

* BNO085 driver: identifies as part 10004148 firmware 3.2.13, streams
  accelerometer / gyro / magnetometer / rotation vector at a measured 100.1 Hz
  with a correct gravity vector and unit-norm quaternions.
* EKF: converges to a commanded forward speed, integrates position at the
  correct rate, and freezes rather than running away when an input dies.
* i-BUS parser: unit-checked against synthetic frames (checksum rejection,
  resync after garbage, newest-frame-from-backlog, channel and knob mapping).
* `flysky_ibus_node` opens the receiver port and holds its 100 Hz failsafe.
  End-to-end stick response was **not** verified because the transmitter was
  off during testing.
* `base_controller_node` was **not** run against the motors, since that moves
  the robot.

## Credits and license

This package is licensed under the MIT license (see `LICENSE`), with one
exception: the DDSM115 protocol layer and the base controller architecture
are derived from
[alduxvm/robot_4wd_dds115_base_controller](https://github.com/alduxvm/robot_4wd_dds115_base_controller)
(BSD, per its package manifest), itself inspired by
[belovictor/ros_ddsm115_driver](https://github.com/belovictor/ros_ddsm115_driver).
Those portions remain under the upstream BSD license; see the third-party
notice in `LICENSE`.
