
# Omni-Vision — 基于地平线 RDK X5 的机器人 3D 视觉定位系统

## 项目简介

本项目部署在 **地平线 RDK X5 开发板** 上，运行于 **ROS 2 Humble** 环境。系统通过 RGB-D 深度相机实时感知桌面物体（杯子、勺子、瓶子、收纳盒），利用 BPU 硬件加速的 YOLOv8 模型进行目标检测，并结合深度信息与手眼标定矩阵，将物体的像素坐标转换为 **机械臂基座坐标系下的真实 3D 抓取坐标**，最后通过 ROS 2 Topic 分发给下游抓取模块。

## 项目结构

```
my_vision/
├── yolo_3d.py                          # 主节点：3D检测 + 深度估计 + 坐标变换 + 时域滤波
├── yolo_bpu_detect_only.py             # 纯检测节点：仅2D目标检测与可视化（调试用）
├── test.py                             # 模型诊断工具：打印BPU模型输入/输出张量信息
├── hand_eye_result.json                # 手眼标定结果（相机→机械臂 4×4 变换矩阵）
├── best_640_480_bayese_640x640_nv12.bin# YOLOv8 BPU编译模型（3.7MB, NV12输入）
├── setup.py                            # ROS 2 包构建配置
├── start_vision.sh                     # 一键启动脚本
└── README.md                           # 本文件
```

## 环境依赖

| 依赖 | 说明 |
|---|---|
| **硬件** | 地平线 RDK X5 开发板（BPU 推理） |
| **相机** | RGB-D 深度相机 |
| **OS** | Ubuntu + ROS 2 Humble |
| **SDK** | `hobot_dnn_rdkx5`（地平线 BPU Python SDK） |
| **Python** | `rclpy`, `sensor_msgs`, `cv_bridge`, `opencv-python`, `numpy`, `message_filters` |

## 算法流程

```
相机驱动
  ├── RGB 图像 (/ascamera/.../rgb0/image)
  ├── Depth 图像 (/ascamera/.../depth0/image_raw)
  └── 相机内参 (/ascamera/.../camera_info)
          │
          ▼
  时间同步 (ApproximateTimeSynchronizer, 200ms容差)
          │
          ▼
  BGR → Letterbox 640×640 → NV12 转换
          │
          ▼
  BPU YOLOv8 推理 (地平线 RDK X5 硬件加速)
          │
          ▼
  DFL 边界框解码 → NMS 去重
          │
          ▼
  ROI深度提取 (29×29区域 + 20%分位数)
          │
          ▼
  针孔成像反投影 → 相机3D坐标
          │
          ▼
  手眼标定变换 (4×4齐次矩阵) → 机械臂3D坐标
          │
          ▼
  三级时域抗抖滤波 (中值→步长裁剪→指数平滑)
          │
          ▼
  置信度(>0.60) + 可达性(臂展<37cm) 双条件过滤
          │
          ▼
  发布 JSON 至 /target_grasp_array
```

## 核心算法说明

### 1. YOLOv8 DFL 边界框解码

YOLOv8 不直接回归边界框坐标，而是对每条边预测 16 个离散概率值（Distribution Focal Loss）。解码时对 16 个值做 Softmax 归一化后求期望值，实现亚像素精度的边界框定位。

### 2. 鲁棒深度估计

在目标中心周围取 **29×29 像素的 ROI**，滤除无效深度值（0值）后取 **20% 分位数**。相比单点读取或均值滤波，这一策略能有效剥离远景背景噪声，稳定锁定目标表面真实深度。

### 3. 三级时域抗抖滤波

对每个目标独立维护抗抖动状态机，三层防护：

| 级别 | 方法 | 参数 | 作用 |
|---|---|---|---|
| 第一级 | 滑窗中值滤波 | 窗口=6帧 | 剔除偶发野值与脉冲噪声 |
| 第二级 | 步长极限裁剪 | 最大=2.5cm/帧 | 防止异常跳变，物理安全兜底 |
| 第三级 | 指数平滑（EMA） | α=0.35 | 磨平随机抖动，轨迹平滑 |

### 4. 手眼标定与坐标变换

系统读取 `hand_eye_result.json` 中通过 ArUco 标记点 + SVD 算法预先标定的 4×4 齐次变换矩阵，将相机系坐标一步映射为机械臂基坐标系坐标。标定基于 19 组采样点，RMSE 约 3.8 cm。

### 5. 软件偏置补偿

在矩阵变换后叠加经验性补偿，修正机械臂结构件形变等系统性偏差，并针对不同物体（杯子/瓶子）预设抓取高度。最终通过臂展极限（37cm）安全校验。

## 模型信息

| 项目 | 详情 |
|---|---|
| 架构 | YOLOv8 (Anchor-Free, DFL Head) |
| 输入 | 640×480 RGB，Letterbox 至 640×640，NV12 格式 |
| 类别 | `cup(0)`, `spoon(1)`, `bottle(2)`, `caddy(3)` |
| 置信度阈值 | 0.45（检测）/ 0.60（发布） |
| NMS IoU 阈值 | 0.45 |

## 运行方式

### 模型诊断

```bash
python3 test.py
```

### 纯2D检测节点（调试用）

```bash
python3 yolo_bpu_detect_only.py
```

### 完整3D定位节点（生产用）

```bash
python3 yolo_3d.py
```

节点启动后将自动：
- 订阅相机话题
- 加载 BPU 模型与手眼标定矩阵
- 实时显示检测画面（OpenCV 窗口）
- 向 `/target_grasp_array` 话题发布目标的 JSON 坐标

### 输出格式

`/target_grasp_array` 话题发布的数据格式：

```json
[
  {
    "class": "cup",
    "x": 15.2,
    "y": -3.1,
    "z": -2.0,
    "score": 0.87
  }
]
```

> 坐标单位：厘米，坐标系：机械臂基座坐标系。
> 仅当置信度 > 0.60 且 X < 37.0cm 时发布。
