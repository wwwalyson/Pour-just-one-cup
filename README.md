# Pour-just-one-cup — Robotic Arm and Hand-Eye Calibration Code

## Project Architecture

```text
.
├── 3rdparty/serial_ros2/          # ROS2 serial communication library dependency
├── arm_msg/                       # Custom ROS2 message definitions
│   └── msg/
│       ├── Arm.msg                # End-effector target coordinates x/y/z (cm)
│       ├── ArmStatus.msg          # Current end-effector status/position
│       └── GripperCmd.msg         # Gripper angle and movement duration
├── arm_base/                      # Robotic arm serial communication bridge (C++)
│   ├── src/arm_base.cpp           # ROS2 topic -> LeArm MCU serial protocol
│   └── launch/arm_bringup.launch.py
├── arm_cmd/                       # Simple coordinate command publisher node (C++)
├── arm_test/                      # IK, test sequences, hand-eye calibration scripts
│   ├── scripts/ik_node.py         # ROS2-side inverse kinematics: /arm_cmd -> /joint_target
│   ├── scripts/test_sequence.py   # Grasp/release test sequences
│   ├── scripts/calib_collect.py   # TF + arm_status point-pair collection and solving
│   ├── scripts/hand_eye_calibrate.py
│   ├── config/hand_eye_calibration.yaml
│   └── launch/ik_bringup.launch.py
├── arm_task/                      # Tea-pouring task node
│   ├── arm_task/tea_task_node.py  # Subscribes to vision detection results and orchestrates grasp/pour actions
│   └── launch/tea_task.launch.py
├── LeArm/                         # STM32/Keil MCU firmware source code and project files
│   ├── Core/                      # CubeMX-generated HAL initialization code
│   ├── Hiwonder/                  # Arm, servo, peripheral, and porting layer code
│   └── MDK-ARM/                   # Keil project files (build outputs excluded)
├── calibration_data/              # Saved hand-eye calibration result JSON
├── docs/                          # Markdown documentation
└── tools/                         # Serial debugging scripts
```

## ROS2 Nodes and Topics

| Module | Node/Executable | Primary Purpose | Main Topics |
|--------|----------------|-----------------|-------------|
| `arm_base` | `arm_base` | Open serial port and send LeArm protocol frames; supports coordinate mode and IK joint mode | Subscribes to `/arm_cmd`, `/gripper_cmd`, `/joint_target`, `/grip_cmd`; publishes `/arm_status` |
| `arm_cmd` | `arm_cmd` | Publish a single target coordinate | Publishes `/arm_cmd`; can subscribe to `/arm_status` |
| `arm_test` | `ik_node.py` | Convert end-effector coordinates to 6 servo target joint angles | Subscribes to `/arm_cmd`; publishes `/joint_target`, `/arm_status` |
| `arm_test` | `calib_collect.py` | Read ArUco pose from TF and record arm coordinates; solve camera-to-base transform | Subscribes to TF, `/arm_status` |
| `arm_task` | `tea_task_node` | Execute tea box / teapot grasping and pouring sequences based on YOLO/vision detection results | Subscribes to `/target_grasp_array`, `/task_cmd`, `/joint_target`; publishes `/arm_cmd`, `/joint_target`, `/grip_cmd` |

Default coordinate unit is **cm**. Joint angles in `ik_node.py` use ROS `JointState` in **radians**.

## Environment Dependencies

Recommended environment: Ubuntu + ROS2 (organized as a ROS2/colcon workspace). Required ROS2 packages include:

- `rclcpp`, `rclpy`
- `std_msgs`, `sensor_msgs`, `geometry_msgs`
- `tf2_ros`
- `ament_cmake`, `ament_python`
- Python: `numpy`, `PyYAML` (used by hand-eye calibration scripts)

For automatic ArUco-based calibration, an external camera driver and ArUco detection node that continuously publishes marker poses or TF are also required.

## Build

Run the following at the repository root:

```bash
# If the current directory is the ROS2 workspace root
colcon build --symlink-install
source install/setup.bash
```

If this repository is part of a larger workspace, place these packages into that workspace and run the same commands.

## Basic Arm Control

### 1. Start the Serial Communication Bridge

Change the serial device name to the actual device, e.g., `/dev/ttyUSB0`:

```bash
source install/setup.bash
ros2 launch arm_base arm_bringup.launch.py \
  usart_port_name:=/dev/ttyUSB0 \
  serial_baud_rate:=9600
```

Common `arm_base` parameters:

- `usart_port_name`: serial device name, default `/dev/ttyUSB0`
- `serial_baud_rate`: baud rate, default `9600`
- `gripper_servo_id`: gripper servo ID, default `1`
- `ik_mode`: whether to enable ROS2-side IK joint control, default `false`
- `move_time_ms`: servo movement duration, default `1500`

### 2. Send an End-Effector Target Point

```bash
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 15.0, y: 0.0, z: 2.0}"
```

Or use the `arm_cmd` node:

```bash
ros2 run arm_cmd arm_cmd --ros-args \
  -p target_x:=15.0 -p target_y:=0.0 -p target_z:=2.0
```

### 3. Gripper Control

In coordinate mode, publish custom gripper messages:

```bash
ros2 topic pub -1 /gripper_cmd arm_msg/msg/GripperCmd "{angle: 90.0, time_ms: 800}"
```

In IK mode, use the independent `/grip_cmd` channel instead (unit: rad):

```bash
ros2 topic pub -1 /grip_cmd std_msgs/msg/Float64 "{data: 1.57}"
```

## IK Control Mode

ROS2-side IK can bypass MCU coordinate cumulative errors:

```bash
source install/setup.bash
ros2 launch arm_test ik_bringup.launch.py \
  target_x:=15.0 target_y:=0.0 target_z:=2.0 \
  preferred_pitch:=-50.0
```

After launching, you can continue publishing coordinate targets:

```bash
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 18.0, y: 4.0, z: 8.0}"
```

`ik_node.py` subscribes to `/arm_cmd`, publishes `/joint_target`, which `arm_base` then converts into multi-servo serial frames.

## Hand-Eye Calibration

This project uses the **Eye-to-Hand** approach: the camera is fixed outside the workspace, an ArUco marker is attached to the gripper/wrist, and the goal is to solve for `T_base_camera` (the rigid transform from camera coordinates to robot arm base coordinates).

### Method A: Using `hand_eye_calibrate.py`

1. Start the arm IK pipeline:

```bash
ros2 launch arm_test ik_bringup.launch.py
```

2. Start the camera and ArUco detection so that it publishes `aruco_pose` (`geometry_msgs/PoseStamped`, unit: m).

3. Run automatic calibration:

```bash
ros2 run arm_test hand_eye_calibrate.py
```

If no automatic detection topic is available, camera coordinates can also be entered manually:

```bash
ros2 run arm_test hand_eye_calibrate.py --manual
```

Calibration poses come from `arm_test/config/hand_eye_calibration.yaml`. The script iterates through multiple arm end-effector positions, records camera-coordinate and base-coordinate point pairs, and updates/outputs `T_base_camera`.

### Method B: Using `calib_collect.py` to Collect TF Point Pairs

Suitable for setups where an ArUco TF publishing chain already exists:

```bash
# 1. Start camera and ArUco detection. Ensure TF contains camera_frame -> marker_frame.
# 2. Start the arm communication or IK pipeline.
ros2 launch arm_test ik_bringup.launch.py

# 3. Start the collection node.
ros2 run arm_test calib_collect.py --ros-args \
  -p camera_frame:=ascamera_color_0 \
  -p marker_frame:=aruco_marker \
  -p min_samples:=25 \
  -p data_dir:=./calibration_data
```

After the arm reaches each new pose, press Enter in the collection node terminal to record a point pair. Once `min_samples` is reached, the script automatically solves and saves `hand_eye_result.json`.

Existing calibration results are saved in `calibration_data/hand_eye_result.json` and can be used for comparison or experiment reproduction.

## Tea-Pouring Task

`arm_task` is designed for the full application workflow: it subscribes to vision detection results from `/target_grasp_array` and executes tea box or teapot grasping, moving, pouring, and returning sequences based on `/task_cmd`.

Startup:

```bash
source install/setup.bash
ros2 launch arm_task tea_task.launch.py move_time_ms:=1500
```

Trigger a task:

```bash
# Tea box / tea leaf task
ros2 topic pub -1 /task_cmd std_msgs/msg/String "{data: '1'}"

# Teapot / water pouring task
ros2 topic pub -1 /task_cmd std_msgs/msg/String "{data: '2'}"
```

The vision node should publish JSON string arrays to `/target_grasp_array`. Example element:

```json
[
  {"class": "caddy", "x": 15.0, "y": 4.0, "z": 3.0, "score": 0.92},
  {"class": "bottle", "x": 18.0, "y": -3.0, "z": 3.0, "score": 0.88},
  {"class": "cup", "x": 17.0, "y": 0.0, "z": 3.0, "score": 0.95}
]
```

## MCU Firmware

The `LeArm/` directory contains CubeMX/HAL source code for the STM32F103RB, Hiwonder robotic arm control code, and Keil project files. Open with Keil MDK:

```text
LeArm/MDK-ARM/LeArm.uvprojx
```

The repository only retains source code and project configuration; local build output / cache directories such as `LeArm/MDK-ARM/LeArm/`, `.pack/`, and `.cmsis/` are not uploaded.

## Serial Debugging

`tools/serial_debug.py` and `tools/serial_test.py` can be used to test the serial protocol directly. Before use, ensure proper serial port permissions, for example:

```bash
sudo usermod -aG dialout $USER
# Log out and log back in for the change to take effect
python3 tools/serial_test.py
```

## Notes

- This repository does not include `.doc`/`.docx` report files.
- Directories such as `build/`, `install/`, `log/`, and `.venv/` should be regenerated locally and not committed.
- Coordinates, servo angles, and workspace boundaries are bound to the current LeArm parameters. When switching to a different arm model, `ik_node.py`, `arm_base`, and firmware parameters must be updated accordingly.
