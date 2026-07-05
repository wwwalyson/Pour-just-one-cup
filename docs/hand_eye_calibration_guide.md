# LeArm 手眼标定操作指南

## 原理

- **眼在手外 (Eye-to-Hand)**: 相机固定，ArUco 标记贴在机械爪
- **采集**: 移动机械臂到 20 个不同位置，每处记录一对点
  - `P_base`: 机械臂末端在底座坐标系下的坐标 (来自 arm_status)
  - `P_cam`: ArUco 标记在相机坐标系下的坐标 (来自 TF)
- **求解**: SVD (Kabsch-Umeyama) 求 `T_base_camera`

## 准备工作

1. **ArUco 标记**贴在腕关节（机械爪根部），确保相机能看到
2. **PC 连接 STM32**（USB 串口 `/dev/ttyUSB0`，波特率 9600）
3. 确保 `arm_ws` 已编译:

   ```bash
   cd ~/workspace/project/arm_ws
   mv .venv .venv_bak && colcon build && mv .venv_bak .venv
   ```

## 启动命令

### 终端 1 (RDK/树莓派): 相机 + ArUco 检测

```bash
# 启动深度相机
cd ~/ros_ws && source install/setup.bash
ros2 launch ascamera ascamera.launch.py

# 新终端: 启动 ArUco 检测器
cd ~/easy_handeye2_ws && source install/setup.bash
ros2 run aruco_detector aruco_detector_node \
  --ros-args -p marker_id:=0 -p marker_size:=0.05 \
             -p marker_frame:=aruco_marker \
             -p camera_frame:=ascamera_color_0
```

### 终端 2 (PC): 机械臂通信 + IK

```bash
cd ~/workspace/project/arm_ws
source install/setup.bash
ros2 launch arm_test ik_bringup.launch.py
```

### 终端 3 (PC): 标定采集

```bash
cd ~/workspace/project/arm_ws
source install/setup.bash
ros2 run arm_test calib_collect.py \
  --ros-args -p camera_frame:=ascamera_color_0 \
             -p marker_frame:=aruco_marker \
             -p min_samples:=20
```

## 20 组标定 Pose (v6 — 非对称 Y, 最大化 cam_X 跨度)

**设计原则**:
- 相机视角: 右侧遮挡 Y≥-7, 左侧开阔 Y 可达 18+
- Y ∈ [-7, 18] **跨度 25cm** (v5 仅 14cm) → cam_X 直接受益
- X > 20, pitch > -35° 不变
- 10 组 Z 对比对保持

|  # |  X |    Y |  Z | pitch | 对 | 说明 |
|---:|---:|-----:|---:|------:|:--:|------|
|  1 | 20 |   -7 | 11 | -33° | ⓐ | X_min 右侧 Z低 |
|  2 | 20 |   -7 | 21 |  -2° | ⓐ | X_min 右侧 Z高 |
|  3 | 28 |   -7 |  7 | -31° | ⓑ | 中距 右侧 Z低 |
|  4 | 28 |   -7 | 18 |  -2° | ⓑ | 中距 右侧 Z高 |
|  5 | 20 |    0 | 11 | -33° | ⓒ | X_min 中心 Z低 |
|  6 | 20 |    0 | 22 |   0° | ⓒ | X_min 中心 Z高 |
|  7 | 26 |    0 |  9 | -33° | ⓓ | 中距 中心 Z低 |
|  8 | 26 |    0 | 20 |  -1° | ⓓ | 中距 中心 Z高 |
|  9 | 30 |    0 |  5 | -33° | ⓔ | 远距 中心 Z低 |
| 10 | 30 |    0 | 17 |  -2° | ⓔ | 远距 中心 Z高 |
| 11 | 34 |    0 |  4 | -22° | ⓕ | 更远 中心 Z低 |
| 12 | 34 |    0 | 13 |   0° | ⓕ | 更远 中心 Z高 |
| 13 | 22 |  +10 | 10 | -33° | ⓖ | 中左 Z低 |
| 14 | 22 |  +10 | 20 |  -3° | ⓖ | 中左 Z高 |
| 15 | 30 |  +10 |  5 | -28° | ⓗ | 远左 Z低 |
| 16 | 30 |  +10 | 15 |  -4° | ⓗ | 远左 Z高 |
| 17 | 20 |  +18 |  8 | -33° | ⓘ | **Y 左极限** Z低 |
| 18 | 20 |  +18 | 17 |  -8° | ⓘ | Y 左极限 Z高 |
| 19 | 28 |  +18 |  5 | -23° | ⓙ | 中距左极限 Z低 |
| 20 | 28 |  +18 | 14 |  -1° | ⓙ | 中距左极限 Z高 |

**范围**: X∈[20,34], **Y∈[-7, 18] 跨度 25cm**, Z∈[4,22]

### 直接复制的命令

```bash
# ⓐ X_min 右侧
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 20.0, y: -15.0, z: 7.0}"
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 20.0, y: -15.0, z: 18.0}"

# ⓑ 中距 右侧
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 28.0, y: -10.0, z: 11.0}"
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 28.0, y: -10.0, z: 17.0}"

# ⓒ X_min 中心
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 20.0, y: -5.0, z: 11.0}"
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 20.0, y: -5.0, z: 20.0}"

# ⓓ 中距 中心
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 26.0, y: 0.0, z: 9.0}"
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 26.0, y: 0.0, z: 20.0}"

# ⓔ 远距 中心
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 30.0, y: 0.0, z: 5.0}"
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 30.0, y: 0.0, z: 17.0}"

# ⓕ 更远 中心
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 34.0, y: 0.0, z: 4.0}"
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 34.0, y: 0.0, z: 13.0}"

# ⓖ 中左 (Y=10)
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 22.0, y: 5.0, z: 10.0}"
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 22.0, y: 5.0, z: 20.0}"

# ⓗ 远左 (Y=10)
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 30.0, y: 10.0, z: 5.0}"
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 30.0, y: 10.0, z: 15.0}"

# ⓘ Y左极限 (Y=18)
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 20.0, y: 15.0, z: 8.0}"
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 20.0, y: 15.0, z: 17.0}"

# ⓙ 中距左极限 (Y=18)
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 28.0, y: 18.0, z: 5.0}"
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: 28.0, y: 18.0, z: 14.0}"
```

## 数据流

```
ros2 topic pub /arm_cmd → ik_node → /joint_target → arm_base → STM32
                                     → /arm_status → calib_collect.py
相机 → aruco_detector → /tf (marker pose) → calib_collect.py
                                                → SVD → hand_eye_result.json
```

## 结果

采集完 20 组后自动求解并保存到 `./calibration_data/hand_eye_result.json`。

```json
{
  "num_samples": 20,
  "rmse_cm": 0.85,
  "rotation": [[0.998, -0.052, 0.034], ...],
  "translation": [25.3, -5.1, 42.7],
  "transform_4x4": [...]
}
```

- **RMSE < 2.0 cm** → 标定质量合格
- **RMSE > 3.0 cm** → 检查 ArUco 检测是否稳定、arm 是否到位

## 标定后使用

```python
# P_base = R @ P_cam + t
# 转换后直接发 /arm_cmd 控制机械臂抓取
ros2 topic pub -1 /arm_cmd arm_msg/msg/Arm "{x: ..., y: ..., z: ...}"
```
