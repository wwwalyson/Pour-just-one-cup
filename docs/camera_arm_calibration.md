# 相机-机械臂标定方法（无 URDF，眼在手外）

## 1. 场景分析

```
┌──────────────────────────────────────┐
│                                      │
│   ┌─────────┐                        │
│   │  相机    │  (固定)                │
│   └────┬────┘                        │
│        │                             │
│        │  视场                        │
│        ▼                             │
│   ┌─────────────────┐                │
│   │   桌面工作区     │                │
│   │                 │                │
│   │  🍵  🥄  ☕    │ ← YOLO检测目标  │
│   │                 │                │
│   └─────────────────┘                │
│             ▲                        │
│             │                        │
│      ┌──────┴──────┐                 │
│      │  机械臂底座  │  (固定)         │
│      └─────────────┘                 │
└──────────────────────────────────────┘

眼在手外 (Eye-to-Hand)：相机和机械臂底座都固定不动
目标：求出 相机坐标系 → 机械臂基坐标系 的变换 [R|t]
```

## 2. 运动学模型（LeArm）

### 2.1 连杆-关节映射

LeArm 六自由度机械臂，基于几何法逆运动学：

```
         l4 (腕)
    ●───●  ← 舵机3 (θ4)
   /     \
  /  l3   \  ← 舵机4 (θ3)
 ●────────●
  \  l2   /  ← 舵机5 (θ2)
   \     /
    ●───●  ← 舵机6 (θ1) — 底座旋转
    l1
    │
   ┌─┴─┐
   │底座│ (固定)
   └───┘
```

| 关节 | 舵机 ID | 连杆 | 说明 |
|------|---------|------|------|
| θ1 | 6 | l1 | 底座旋转，控制机械臂在 XY 平面的投影方向 |
| θ2 | 5 | l2 | 肩关节，l2 与水平面的夹角 |
| θ3 | 4 | l3 | 肘关节，相对于 l2 延长线的转角 |
| θ4 | 3 | l4 | 腕关节，相对于 l3 延长线的转角 |
| — | 1, 2 | — | 机械爪旋转和张合，**不影响末端位置** |

### 2.2 工作空间（cm）

| 轴 | 最小值 | 最大值 | 默认值 |
|----|--------|--------|--------|
| X | 10.0 | 20.0 | 15.0 |
| Y | -10.0 | 10.0 | 0.0 |
| Z | 0.0 | 25.0 | 2.0 |

坐标系为右手系。控制增量精度 0.1 cm（通过 int8_t 发送，每单位 = 0.1 cm）。

### 2.3 逆运动学接口

```c
// robot_arm_coordinate_set 参数说明
robot_arm_coordinate_set(
    x, y, z,       // 目标位置 (cm)
    target_pitch,  // 目标俯仰角 (度)
    min_pitch,     // 俯仰角最小值
    max_pitch,     // 俯仰角最大值
    time_ms        // 运动时间 (ms)
);
```

逆运动学核心（`set_pitch_range`）在闭源 `LeArm.lib` 中，但外部只需知道坐标 → 关节角的映射即可。

### 2.4 关节角 → 舵机脉宽

```c
// theta2servo(): 4.1667 = 1000 / 240, 将 240° 范围映射为 1000 脉宽
pulse = 500 + (int)(4.1667 * joint_angle);

// 关节 → 舵机ID 映射
knot[0] (底座旋转 θ1) → 舵机6
knot[1] (肩关节 θ2) → 舵机5
knot[2] (肘关节 θ3) → 舵机4
knot[3] (腕关节 θ4) → 舵机3
```

---

## 3. 不需要 URDF 的原因

URDF 的作用：描述正向运动学链（关节关系 → 末端位姿），供 ROS 的 TF 树和 MoveIt 等使用。

本项目的特殊性：

1. **已有逆运动学库**：`LeArm.lib` 直接通过坐标增量控制末端位置，不走 TF/ROS 运动学链
2. **已有位置反馈**：`arm_status` 话题直接给出末端在基坐标系下的 (x, y, z)，跳过了正向运动学
3. **只需平移**：手眼标定本质上只需要 "相机坐标 → 基坐标" 的 3D 点映射，用点集配准即可求解，不依赖运动学模型

**结论**：手眼标定只关心两组 3D 点集之间的关系，与机械臂内部运动学模型无关。

---

## 4. 手眼标定方法（Eye-to-Hand）

### 4.1 原理

这是一个 **点集配准 (Point Set Registration)** 问题：

```
采集 N 组点对 {P_cam_i, P_base_i}, i=1..N
  P_cam_i  : ArUco 标记中心在相机坐标系下的 3D 位置
  P_base_i : 同一物理点（ArUco 标记中心）在基坐标系下的 3D 位置

求解刚体变换 [R|t] 使得:
  P_base = R × P_camera + t

使用 Kabsch-Umeyama 算法（SVD 分解）求解
```

#### 为什么只需要 XYZ，不需要旋转角？

点集配准的本质是：**同一个物理点在两个坐标系下的两组坐标之间，只差一个旋转 + 平移**。

```
┌────────────────────────────────────────────────────┐
│                   同一个物理点 M                     │
│                                                    │
│   相机看到: P_cam = (x_c, y_c, z_c)                 │
│   基座看到: P_base = (x_b, y_b, z_b)                │
│                                                    │
│   关系: P_base = R × P_cam + t                      │
│                                                    │
│   R 和 t 是相机坐标系 → 基坐标系的刚体变换           │
│   一旦求出 R 和 t，任意相机坐标都能转为基坐标        │
└────────────────────────────────────────────────────┘
```

不需要末端旋转角的原因：R 和 t 描述的是 **两个坐标系之间的关系**，而不是机械臂末端的姿态。只要我们能准确记录 **同一个点** 在两个坐标系下的 3D 坐标，SVD 就能求解。

不需要姿态信息，也就不依赖 URDF/正向运动学。

#### 关键前提：标记必须放在 arm_status 汇报的位置

`arm_status` 汇报的是 **腕关节末端（机械爪根部，l4 末端）** 的 XYZ：

```
            l4        机械爪
    ●─────────────●~~~~~~~~~~[ ArUco ]
  肘关节      腕关节(arm_status汇报位置)
```

如果把 ArUco 标记贴在机械爪指尖，则 arm_status 的 XYZ ≠ 标记中心的位置，存在一个偏移向量 d。这个偏移 d 随着机械爪姿态变化在基坐标系下会旋转，不能简单加常数。

**解决方案（三选一）：**

| 方案 | 做法 | 是否需要姿态 |
|------|------|-------------|
| **A：标记贴在腕关节（推荐）** | 将 ArUco 精确贴在机械爪根部（腕关节旋转中心），使标记中心与 arm_status 汇报点重合 | 否 |
| **B：标记偏移 + 姿态修正** | 标记可贴任意位置，但需额外记录末端姿态（pitch + yaw），将偏移变换到基坐标系 | 是，需扩展 arm_status |
| **C：AX=XB 经典手眼标定** | 使用相对运动增量，不需知道标记偏移，但需完整的末端 6D 位姿（xyz + rpy）和标记 6D 位姿 | 是，需求完整正向运动学 |

> **推荐方案 A**：最简单，不需要改任何现有代码，不需要姿态信息。将 ArUco 标记贴在机械爪根部平面，标记中心对准腕关节旋转轴心即可。

#### 补充：如果要用方案 C（AX=XB 经典手眼标定）

当标记无法贴在腕关节时，可以用 Tsai/Park 方法。它与点集配准的区别：

```
点集配准 (本文方法):            AX=XB (经典方法):
─────────────────────          ─────────────────────
输入: 同一物理点的 XYZ          输入: 末端的 6D 位姿 (xyz+rpy)
      (只需位置)                     标记的 6D 位姿 (rvec+tvec)
                               
需求: 标记 = 腕关节位置          需求: 完整正向运动学 (URDF 或 DH 参数)
                                               
求解: SVD, 直接得 [R|t]         求解: 解 AX=XB 方程, 得 T_cam_to_base
                                               
精度: 取决于标记位置精度          精度: 更鲁棒，可容忍标记偏移
```

AX=XB 方法不要求标记在腕关节，因为它利用的是 **相对运动的增量**：机械臂从位置 1 运动到位置 2，末端在基坐标系下的运动量 A，和标记在相机坐标系下的运动量 B，满足 AX = XB。

但 LeArm 当前不提供末端姿态，实现 AX=XB 需要先扩展 `arm_status` 消息增加姿态字段，并通过几何法正向运动学计算姿态。这是完整的路线但工作量更大。

### 4.2 标定流程

```
┌─────────────────────┐
│ 1. 相机内参标定      │ → camera_params.yaml
│   棋盘格 + OpenCV     │
└────────┬────────────┘
         ▼
┌─────────────────────┐
│ 2. 末端固定 ArUco    │ → marker 物理尺寸已知
│   标记               │
└────────┬────────────┘
         ▼
┌─────────────────────┐
│ 3. 采集 15~20 组点对│ → calibration_data/
│   移动机械臂到不同位  │   每个位置录: arm_status + ArUco位姿
│   置，录制数据        │
└────────┬────────────┘
         ▼
┌─────────────────────┐
│ 4. SVD 求解 [R\|t]   │ → hand_eye.yaml
│   Kabsch-Umeyama     │
└────────┬────────────┘
         ▼
┌─────────────────────┐
│ 5. 验证             │ → 误差 < 1 cm
│   用未参与标定的点   │
│   预测 vs 实测       │
└─────────────────────┘
```

### 4.3 数据采集细节

标定过程中，ArUco 标记贴在机械臂末端（机械爪根部，即 l4 末端关节处）。

每个采样位置需要满足：

- 标记在相机视场内且清晰可见
- 位置覆盖整个工作空间（不要集中在某一小块区域）
- 至少 4 个非共面点（建议 15~20 个）
- 每个位置保持机械臂静止后再记录数据

数据格式（每行一组）：

```
P_cam_x P_cam_y P_cam_z  P_base_x P_base_y P_base_z
```

---

## 5. 核心算法：Kabsch-Umeyama（SVD）

```python
import numpy as np


def solve_rigid_transform(P_cam, P_base):
    """
    用 SVD 求解相机坐标系到机械臂基坐标系的刚体变换。

    Args:
        P_cam:  (N, 3) 相机坐标系下的 3D 点集
        P_base: (N, 3) 机械臂基坐标系下的 3D 点集

    Returns:
        R: (3, 3) 旋转矩阵
        t: (3,)   平移向量
        rmse:     均方根误差 (cm)
    """
    assert P_cam.shape == P_base.shape
    assert P_cam.shape[0] >= 4, "至少需要4个点对"

    # 1. 计算质心
    centroid_cam = np.mean(P_cam, axis=0)
    centroid_base = np.mean(P_base, axis=0)

    # 2. 去质心
    Q_cam = P_cam - centroid_cam
    Q_base = P_base - centroid_base

    # 3. 互协方差矩阵
    H = Q_cam.T @ Q_base  # (3, 3)

    # 4. SVD 分解
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # 5. 反射修正 (确保 det(R) = +1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # 6. 平移向量
    t = centroid_base - R @ centroid_cam

    # 7. 误差评估
    P_pred = (R @ P_cam.T).T + t
    errors = np.linalg.norm(P_pred - P_base, axis=1)
    rmse = np.sqrt(np.mean(errors ** 2))

    return R, t, rmse
```

### 最小二乘法（仅平移，无旋转场景）

如果相机安装角度恰好与基坐标系平行（正上方垂直拍摄），只需标定平移：

```python
def solve_translation_only(P_cam, P_base):
    """相机与基坐标系轴对齐时，只需求平移向量"""
    delta = P_base - P_cam
    t = np.mean(delta, axis=0)
    rmse = np.sqrt(np.mean(np.sum((delta - t) ** 2, axis=1)))
    return t, rmse
```

---

## 6. 深度相机 + ArUco 检测器方案（已有基础设施）

现有 `easy_handeye2_ws` 中的 `aruco_detector` 节点通过 PnP 解算 ArUco 标记在相机坐标系下的 6D 位姿，并通过 TF 广播（`camera_color_optical_frame` → `aruco_marker`）。

标定采集节点 `calib_collect.py` 直接监听这个 TF，获取标记位置，无需自己处理图像或深度数据。

### 6.1 原理

```
ascamera (深度相机驱动)
  ↓ /rgb0/image + /rgb0/camera_info
aruco_detector (PnP 解算)
  ↓ TF: camera_frame → aruco_marker  (标记3D位姿)
calib_collect (标定采集)
  ← TF (标记相机坐标, m → cm)
  ← arm_status (末端基坐标, cm)
  → 记录点对 → SVD 求解
```

### 6.2 为什么比之前更简单

| 之前 (自己处理图像) | 现在 (复用 ArUco TF) |
|---------------------|----------------------|
| 订阅 RGB + Depth + CameraInfo | 只监听 TF |
| 自己检测 ArUco + 深度反投影 | 已有节点完成 PnP |
| 需处理深度空洞 | 不存在，PnP 不依赖深度图 |
| ~200 行 | ~90 行 |

### 6.3 ArUco 检测器参数

---

## 7. 标定节点使用说明

标定采集节点在 `arm_test/scripts/calib_collect.py`。它复用已有的 `aruco_detector` (来自 easy_handeye2) 发布的 TF 来获取标记在相机坐标系下的位置，不需要自己处理图像或深度数据。

### 数据流

```
┌─ 机器A (RDK/树莓派) ──────────────────────────────┐
│                                                    │
│  ascamera (深度相机驱动)                             │
│    ↓ /ascamera/camera_publisher/rgb0/image          │
│    ↓ /ascamera/camera_publisher/rgb0/camera_info    │
│                                                    │
│  aruco_detector (ArUco 检测)                        │
│    ↓ TF: camera_color_optical_frame → aruco_marker │
│      (标记在相机坐标系下的位姿, PnP 解算)             │
│                                                    │
└────────────────────┬───────────────────────────────┘
                     │  网络 (ROS2 DDS)
                     ▼
┌─ 机器B (PC) ───────────────────────────────────────┐
│                                                    │
│  calib_collect (标定采集)                            │
│    ← TF: aruco_marker 位置 (相机坐标系, m→cm)        │
│    ← arm_status 位置 (基坐标系, cm)                  │
│    Enter 键触发 → 记录点对 → 15组 → SVD 求解         │
│                                                    │
└────────────────────────────────────────────────────┘
```

### 完整启动步骤

**机器A (RDK/树莓派) — 相机 + ArUco 检测:**

```bash
# 1. 深度相机
cd ~/ros_ws && source install/setup.bash
ros2 launch ascamera ascamera.launch.py

# 2. ArUco 检测器 (发布 TF)
cd ~/easy_handeye2_ws && source install/setup.bash
ros2 run aruco_detector aruco_detector_node \
  --ros-args -p marker_id:=0 \
             -p marker_size:=0.05 \
             -p camera_frame:=camera_color_optical_frame \
             -p marker_frame:=aruco_marker
```

**机器B (PC) — 机械臂 + 标定:**

```bash
# 3. 机械臂通信
cd ~/arm_ws && source install/setup.bash
ros2 launch arm_base arm_bringup.launch.py

# 4. PD 控制器 (平滑移动, 可选但推荐)
ros2 run arm_test pd_controller.py

# 5. 标定采集
ros2 run arm_test calib_collect.py \
  --ros-args -p camera_frame:=camera_color_optical_frame \
             -p marker_frame:=aruco_marker \
             -p min_samples:=15
```

### 操作

```bash
# 移动机械臂 → 到位后按 Enter → 重复 15+ 次 → 自动求解
ros2 topic pub /target arm_msg/msg/Arm "{x: 14.0, y: 2.0,  z: 8.0}"  # Enter
ros2 topic pub /target arm_msg/msg/Arm "{x: 17.0, y: -3.0, z: 5.0}"  # Enter
ros2 topic pub /target arm_msg/msg/Arm "{x: 12.0, y: -5.0, z: 12.0}" # Enter
# ... 覆盖整个工作空间, 不同高度至少3层
```

终端日志示例:
```
[ 1] cam=( 5.23, -2.11, 45.30)  base=(14.00,  2.00,  8.00)
[ 2] cam=( 8.15,  4.32, 42.10)  base=(17.00, -3.00,  5.00)
...
[15] cam=(-1.20,  0.85, 48.70)  base=(13.00,  6.00, 10.00)
已达 15 组, 开始求解...
========== 标定结果 ==========
样本数: 15,  RMSE: 0.432 cm
旋转矩阵 R (cam→base):
[[-0.012  0.998 -0.055]
 [-0.999 -0.014 -0.034]
 [-0.034  0.054  0.998]]
平移向量 t (cam→base, cm): [12.34 -5.67 48.21]
已保存至 ./calibration_data/hand_eye_result.json
```

### 可配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `camera_frame` | `camera_color_optical_frame` | 相机 TF 帧名 |
| `marker_frame` | `aruco_marker` | ArUco 标记 TF 帧名 |
| `min_samples` | 15 | 最少采集组数 |
| `data_dir` | `./calibration_data` | 结果保存目录 |
| `tf_timeout` | 0.5 s | TF 数据过期时间 |

---

## 8. 标定后使用

标定完成后，YOLO 检测到的物体坐标转换流程：

```python
# 加载标定结果
with open('calibration_data/hand_eye_result.json') as f:
    calib = json.load(f)

R_cam_to_base = np.array(calib['rotation'])
t_cam_to_base = np.array(calib['translation'])


def camera_to_base(P_camera):
    """将相机坐标系下的 3D 点转换到机械臂基坐标系"""
    return R_cam_to_base @ P_camera + t_cam_to_base


# 示例: YOLO 检测到茶杯在相机坐标系下的位置
teacup_cam = np.array([5.2, -3.1, 25.0])  # (x, y, z) cm
teacup_base = camera_to_base(teacup_cam)

# 发送坐标增量指令给机械臂
# ...
```

---

## 9. 实施路线

| 序号 | 任务 | 依赖 | 产出 |
|------|------|------|------|
| 1 | 打印棋盘格，拍摄 ~20 张照片 | — | 照片集 |
| 2 | 相机内参标定 | 步骤1 | `camera_params.yaml` |
| 3 | 打印 ArUco 标记，贴在机械臂末端 | 步骤2 | — |
| 4 | 实现 `arm_calibrate` 节点 | 步骤3 | 采集数据 |
| 5 | 移动机械臂到 15~20 个位置，采集点对 | 步骤4 | 点对数据集 |
| 6 | SVD 求解变换矩阵 | 步骤5 | `hand_eye.yaml` |
| 7 | 验证：新位置预测 vs 实测 | 步骤6 | 误差报告 (< 1cm) |
| 8 | 集成到 YOLO 检测 → 坐标转换 → 机械臂抓取 | 步骤7 | 完整抓取链路 |

---

## 10. 误差来源与对策

| 误差来源 | 影响 | 对策 |
|----------|------|------|
| 相机内参不准确 | 3D 坐标系统偏差 | 采集 > 20 张棋盘格，覆盖图像四角 |
| ArUco 标记尺寸不准 | 深度估计偏大/偏小 | 精确测量标记边长（误差 < 0.5mm） |
| 采样点分布不均 | 旋转矩阵在某方向不准确 | 覆盖整个工作空间，不要挤在一个平面 |
| 机械臂实际连杆与名义值不符 | 基座标示位置 ≠ 实际末端位置 | 采样时等待完全静止后再记录 |
| 图像模糊 / 运动模糊 | 角点检测跑偏 | 充足光照，静止后拍摄 |
