# Pour-just-one-cup 机械臂与手眼标定代码

本仓库收录“Pour just one cup”项目中与 **LeArm 机械臂控制**、**ROS2 上位机**、**手眼标定** 和 **倒茶/倒水任务编排** 相关的代码。仓库已排除 `.doc/.docx` 报告文件、ROS2 构建产物、Python 虚拟环境、Keil 编译输出等非源码内容。

## 项目架构

```text
.
├── 3rdparty/serial_ros2/          # ROS2 串口库依赖
├── arm_msg/                       # 自定义 ROS2 消息
│   └── msg/
│       ├── Arm.msg                # 末端目标坐标 x/y/z，单位 cm
│       ├── ArmStatus.msg          # 当前末端状态/位置
│       └── GripperCmd.msg         # 夹爪角度与运动时间
├── arm_base/                      # 机械臂串口通信桥（C++）
│   ├── src/arm_base.cpp           # ROS2 topic -> LeArm 下位机串口协议
│   └── launch/arm_bringup.launch.py
├── arm_cmd/                       # 简单坐标命令发布节点（C++）
├── arm_test/                      # IK、测试序列、手眼标定脚本
│   ├── scripts/ik_node.py         # ROS2 侧逆运动学，/arm_cmd -> /joint_target
│   ├── scripts/test_sequence.py   # 抓取/释放测试序列
│   ├── scripts/calib_collect.py   # TF + arm_status 采集点对并求解
│   ├── scripts/hand_eye_calibrate.py
│   ├── config/hand_eye_calibration.yaml
│   └── launch/ik_bringup.launch.py
├── arm_task/                      # 倒茶/倒水任务节点
│   ├── arm_task/tea_task_node.py  # 订阅视觉检测结果并编排抓取/倾倒动作
│   └── launch/tea_task.launch.py
├── LeArm/                         # STM32/Keil 下位机固件源码与工程文件
│   ├── Core/                      # CubeMX 生成的 HAL 初始化代码
│   ├── Hiwonder/                  # 机械臂、舵机、外设、移植层代码
│   └── MDK-ARM/                   # Keil 工程文件（不含编译输出）
├── calibration_data/              # 已保存的手眼标定结果 JSON
├── docs/                          # Markdown 说明文档
└── tools/                         # 串口调试脚本
```

## ROS2 节点与话题

| 模块 | 节点/可执行文件 | 主要作用 | 主要话题 |
| --- | --- | --- | --- |
| `arm_base` | `arm_base` | 打开串口并发送 LeArm 协议帧；支持坐标模式和 IK 关节模式 | 订阅 `/arm_cmd`、`/gripper_cmd`、`/joint_target`、`/grip_cmd`；发布 `/arm_status` |
| `arm_cmd` | `arm_cmd` | 发布一次目标坐标 | 发布 `/arm_cmd`，可订阅 `/arm_status` |
| `arm_test` | `ik_node.py` | 将末端坐标转换为 6 个舵机目标关节 | 订阅 `/arm_cmd`；发布 `/joint_target`、`/arm_status` |
| `arm_test` | `calib_collect.py` | 从 TF 读取 ArUco 位姿并记录机械臂坐标，求解相机到基座变换 | 订阅 TF、`/arm_status` |
| `arm_task` | `tea_task_node` | 根据 YOLO/视觉检测结果执行茶盒、茶壶抓取与倾倒流程 | 订阅 `/target_grasp_array`、`/task_cmd`、`/joint_target`；发布 `/arm_cmd`、`/joint_target`、`/grip_cmd` |

坐标默认单位为 **cm**；`ik_node.py` 中关节角使用 ROS `JointState` 的 **rad**。

## 环境依赖

建议环境：Ubuntu + ROS2（已在工作区内按 ROS2/colcon 工程组织）。需要的 ROS2 包包括：

- `rclcpp`、`rclpy`
- `std_msgs`、`sensor_msgs`、`geometry_msgs`
- `tf2_ros`
- `ament_cmake`、`ament_python`
- Python：`numpy`、`PyYAML`（手眼标定脚本使用）

如需使用自动 ArUco 采集，还需要外部相机驱动和 ArUco 检测节点持续发布 marker 位姿或 TF。

## 编译

在仓库根目录执行：

```bash
# 如果当前目录就是 ROS2 工作空间根目录
colcon build --symlink-install
source install/setup.bash
```

若把本仓库作为更大工作空间的一部分，也可以将这些包放入工作空间后执行同样命令。

## 基础机械臂控制

### 1. 启动串口通信桥

将串口设备名改成实际设备，例如 `/dev/ttyUSB0`：

```bash
source install/setup.bash
ros2 launch arm_base arm_bringup.launch.py \
  usart_port_name:=/dev/ttyUSB0 \
  serial_baud_rate:=9600
```

`arm_base` 常用参数：

- `usart_port_name`：串口设备名，默认 `/dev/ttyUSB0`
- `serial_baud_rate`：波特率，默认 `9600`
- `gripper_servo_id`：夹爪舵机 ID，默认 `1`
- `ik_mode`：是否启用 ROS2 侧 IK 关节控制，默认 `false`
- `move_time_ms`：舵机运动时间，默认 `1500`

### 2. 发送一个末端目标点

```bash
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 15.0, y: 0.0, z: 2.0}"
```

或使用 `arm_cmd` 节点：

```bash
ros2 run arm_cmd arm_cmd --ros-args \
  -p target_x:=15.0 -p target_y:=0.0 -p target_z:=2.0
```

### 3. 控制夹爪

坐标模式下可发布自定义夹爪消息：

```bash
ros2 topic pub -1 /gripper_cmd arm_msg/msg/GripperCmd "{angle: 90.0, time_ms: 800}"
```

IK 模式下推荐使用独立 `/grip_cmd` 通道，单位为 rad：

```bash
ros2 topic pub -1 /grip_cmd std_msgs/msg/Float64 "{data: 1.57}"
```

## IK 控制模式

ROS2 侧 IK 可绕过下位机坐标累积误差：

```bash
source install/setup.bash
ros2 launch arm_test ik_bringup.launch.py \
  target_x:=15.0 target_y:=0.0 target_z:=2.0 \
  preferred_pitch:=-50.0
```

启动后也可以继续发布坐标目标：

```bash
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 18.0, y: 4.0, z: 8.0}"
```

`ik_node.py` 会订阅 `/arm_cmd`，发布 `/joint_target`，再由 `arm_base` 转换成多舵机串口帧。

## 手眼标定使用方法

本项目采用 **Eye-to-Hand** 思路：相机固定在工作区外，ArUco 标记贴在机械爪/腕部，求解 `T_base_camera`（相机坐标到机械臂基座坐标的刚体变换）。

### 方式 A：使用 `hand_eye_calibrate.py`

1. 启动机械臂 IK 流水线：

```bash
ros2 launch arm_test ik_bringup.launch.py
```

2. 启动相机和 ArUco 检测，使其发布 `aruco_pose`（`geometry_msgs/PoseStamped`，单位 m）。

3. 运行自动标定：

```bash
ros2 run arm_test hand_eye_calibrate.py
```

若没有自动检测话题，也可手动输入相机坐标：

```bash
ros2 run arm_test hand_eye_calibrate.py --manual
```

标定姿态来自 `arm_test/config/hand_eye_calibration.yaml`。脚本会遍历多个机械臂末端坐标，记录相机坐标与基座坐标点对，并更新/输出 `T_base_camera`。

### 方式 B：使用 `calib_collect.py` 采集 TF 点对

适合已有 ArUco TF 发布链路的情况：

```bash
# 1. 启动相机和 ArUco 检测，确保 TF 中存在 camera_frame -> marker_frame
# 2. 启动机械臂通信或 IK 流水线
ros2 launch arm_test ik_bringup.launch.py

# 3. 启动采集节点
ros2 run arm_test calib_collect.py --ros-args \
  -p camera_frame:=ascamera_color_0 \
  -p marker_frame:=aruco_marker \
  -p min_samples:=25 \
  -p data_dir:=./calibration_data
```

每次机械臂到达新姿态后，在采集节点终端按 Enter 记录一组点对；达到 `min_samples` 后自动求解并保存 `hand_eye_result.json`。

已有标定结果保存在 `calibration_data/hand_eye_result.json`，可用于比对或复现实验。

## 倒茶/倒水任务

`arm_task` 面向完整应用流程：订阅视觉检测结果 `/target_grasp_array`，并根据 `/task_cmd` 执行茶盒或茶壶抓取、移动、倾倒和归位。

启动：

```bash
source install/setup.bash
ros2 launch arm_task tea_task.launch.py move_time_ms:=1500
```

触发任务：

```bash
# 茶盒/茶叶任务
ros2 topic pub -1 /task_cmd std_msgs/msg/String "{data: '1'}"

# 茶壶/倒水任务
ros2 topic pub -1 /task_cmd std_msgs/msg/String "{data: '2'}"
```

视觉节点需要向 `/target_grasp_array` 发布 JSON 字符串数组，元素示例：

```json
[
  {"class": "caddy", "x": 15.0, "y": 4.0, "z": 3.0, "score": 0.92},
  {"class": "bottle", "x": 18.0, "y": -3.0, "z": 3.0, "score": 0.88},
  {"class": "cup", "x": 17.0, "y": 0.0, "z": 3.0, "score": 0.95}
]
```

## 下位机固件

`LeArm/` 目录包含 STM32F103RB 相关的 CubeMX/HAL 源码、Hiwonder 机械臂控制代码和 Keil 工程文件。可用 Keil MDK 打开：

```text
LeArm/MDK-ARM/LeArm.uvprojx
```

仓库只保留源码与工程配置，未上传 `LeArm/MDK-ARM/LeArm/`、`.pack/`、`.cmsis/` 等本地编译输出/缓存目录。

## 串口调试

`tools/serial_debug.py`、`tools/serial_test.py` 可用于直接测试串口协议。使用前请确认串口权限，例如：

```bash
sudo usermod -aG dialout $USER
# 重新登录后生效
python3 tools/serial_test.py
```

## 备注

- 本仓库不包含 `.doc/.docx` 报告文件。
- `build/`、`install/`、`log/`、`.venv/` 等目录应在本地重新生成，不应提交。
- 坐标、舵机角度和工作空间边界与当前 LeArm 机械臂参数绑定，换机型时需同步修改 `ik_node.py`、`arm_base` 和固件参数。
