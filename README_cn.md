## English [README.md](./README.md)

# 语音控制机械臂自动泡茶系统

本系统由三个子项目组成，实现从自然语言指令到机械臂物理执行的全自动闭环：用户通过语音下达指令，系统经语音识别与 LLM 意图解析后，调度视觉模块完成目标 3D 定位，最终由机械臂执行抓取与倾倒动作。

---

## 项目组成

| 项目 | 路径 | 角色 | 职责 |
|------|------|------|------|
| xiaozhi-server | `xiaozhi-server\` | 语音交互与任务调度 | 语音对话管线（VAD/ASR/LLM/TTS）、意图解析、Function Calling |
| my_vision | `my_vision\` | 视觉定位 | RGB-D 目标检测、3D 坐标解算、手眼标定 |
| Pour-just-one-cup-arm | `Pour-just-one-cup-arm\` | 机械臂控制 | 串口通信、逆运动学、抓取与倾倒动作编排 |

---

## 系统架构

```
                              +------------------------+
                              |         用户           |
                              |   "帮我泡杯茶"（语音）  |
                              +-----------+------------+
                                          | WebSocket (Opus)
                                          v
+-----------------------------------------------------------------------+
|  RDK X5 上位机                                                         |
|                                                                        |
|  +-------------------------+     +-------------------------+           |
|  |  xiaozhi-server         |     |  my_vision              |           |
|  |                         |     |                         |           |
|  |  VAD -> ASR -> LLM     |     |  RGB-D 相机             |           |
|  |    |                    |     |    |                    |           |
|  |    v                    |     |    v                    |           |
|  |  Function Calling      |<--->|  YOLOv8 (BPU 加速)     |           |
|  |    |                    |     |    |                    |           |
|  |    v                    |     |    v                    |           |
|  |  TTS -> 语音回复        |     |  3D 坐标 + 手眼标定    |           |
|  +-----------+-------------+     +-----------+-------------+           |
|              |                               |                         |
|              |  ROS2 Service                 |  ROS2 Topic             |
|              |  /tea_command                 |  /target_grasp_array    |
|              +-------------+-----------------+                         |
+----------------------------+------------------------------------------+
                             | 串口 (USB-TTL)
                             v
+-----------------------------------------------------------------------+
|  STM32 下位机 + LeArm 六自由度机械臂                                   |
|                                                                        |
|  逆运动学 -> 舵机角度 -> 轨迹规划 -> 抓取 -> 倾倒 -> 复位              |
+-----------------------------------------------------------------------+
```

---

## 数据流

```
用户语音
  |
  v
ESP32 设备 (Opus 编码, 24kHz, 60ms 帧)
  |
  v  WebSocket
xiaozhi-server
  +-- VAD (Silero ONNX, 本地推理)
  +-- ASR (FunASR SenseVoiceSmall, 本地免费)
  +-- LLM (ChatGLM + Function Calling)
  |     +-- 普通对话 -> TTS (EdgeTTS) -> 语音回复
  |     +-- 泡茶指令 -> ros2_arm_control 工具函数
  |                       |
  |                       v  ROS2 Service: /tea_command
  |              arm_task/tea_task_node.py
  |                 |
  |                 +-- 请求目标坐标
  |                 |     |
  |                 |     v  ROS2 Topic: /target_grasp_array
  |                 |  my_vision/yolo_3d.py
  |                 |     +-- RGB-D 图像采集
  |                 |     +-- BPU YOLOv8 目标检测
  |                 |     +-- 深度估计 + 坐标变换
  |                 |     +-- 返回 JSON: [{class, x, y, z, score}]
  |                 |
  |                 v
  |          抓取规划 -> arm_base 串口桥 -> STM32 -> 舵机执行
  |
  +-- 动作完成 -> TTS 语音播报结果
```

---

## xiaozhi-server — 语音交互与任务调度

基于 [xiaozhi-esp32-server](https://github.com/xinnan-tech/xiaozhi-esp32-server) 开源项目扩展开发，部署于地平线 RDK X5 上位机。

### 核心文件

| 文件 | 说明 |
|------|------|
| `app.py` | 主入口，启动 WebSocket/HTTP 服务，加载 ROS2 环境 |
| `config.yaml` | 全局配置文件（VAD/ASR/TTS/LLM/插件/ROS2 参数） |
| `core/handle/receiveAudioHandle.py` | 语音接收处理管线：VAD -> ASR -> LLM 对话 |
| `core/handle/sendAudioHandle.py` | TTS 流式合成与帧率控制发送 |
| `plugins_func/functions/ros2_arm_control.py` | 机械臂泡茶控制插件（LLM Function Calling） |
| `core/providers/tools/ros2_arm_client.py` | ROS2 Service 异步客户端封装 |

### 语音处理管线

1. VAD 语音检测：Silero VAD（ONNX 本地推理），双阈值迟滞机制判定语音起止
2. ASR 语音识别：FunASR SenseVoiceSmall（本地免费），支持情绪与语种检测
3. LLM 意图解析：ChatGLM + Function Calling，常规对话生成文本回复，泡茶指令调用 `ros2_arm_control` 工具函数
4. TTS 语音合成：EdgeTTS（免费），分句后流式合成，经 Opus 编码回传设备播报

### 技术特性

- 多引擎切换：支持 15 种 ASR 方案与 18 种 TTS 方案，通过配置文件切换
- 插件注册机制：基于 `@register_function` 装饰器的插件系统，自动发现与注册
- 流式 TTS 输出：双线程队列配合 AudioRateController 实现 60ms 精准帧率控制
- 异步解耦：Python asyncio 与 ROS2 异步 Service 协同，LLM 网络请求与实时控制分离

### 环境要求

- 地平线 RDK X5 开发板
- Ubuntu 22.04 / ROS2 Humble / Python 3.10+

---

## my_vision — 视觉定位

部署于 RDK X5 的 ROS2 Python 节点，利用 BPU 硬件加速引擎进行实时 3D 目标检测与定位。

### 核心文件

| 文件 | 说明 |
|------|------|
| `yolo_3d.py` | 主节点：3D 检测、深度估计、坐标变换、时域滤波、结果发布 |
| `yolo_bpu_detect_only.py` | 纯 2D 检测节点，用于调试与可视化 |
| `test.py` | BPU 模型诊断工具，打印输入/输出张量信息 |
| `hand_eye_result.json` | 手眼标定结果（4x4 齐次变换矩阵，19 组采样，RMSE 约 3.8cm） |
| `best_640_480_bayese_640x640_nv12.bin` | YOLOv8 BPU 编译模型（3.7MB，NV12 输入格式） |
| `setup.py` | ROS2 包构建配置 |
| `start_vision.sh` | 一键启动脚本 |

### 算法管线

```
RGB-D 图像 -> Letterbox 640x640 -> NV12 格式转换 -> BPU YOLOv8 推理
    -> DFL 边界框解码 -> NMS 非极大值抑制
    -> ROI 深度提取（29x29 像素区域，20% 分位数）
    -> 针孔成像反投影 -> 相机坐标系 3D 坐标
    -> 手眼标定变换（4x4 齐次矩阵）-> 机械臂基座坐标系 3D 坐标
    -> 三级时域抗抖滤波（中值滤波 -> 步长裁剪 -> 指数平滑）
    -> 置信度（>0.60）与可达性（臂展 < 37cm）双条件过滤
    -> 发布 JSON 至 /target_grasp_array
```

### 目标检测

| 项目 | 参数 |
|------|------|
| 模型架构 | YOLOv8 (Anchor-Free, DFL Head) |
| 输入规格 | 640x480 RGB，Letterbox 至 640x640，NV12 格式 |
| 检测类别 | cup（杯子）、spoon（勺子）、bottle（瓶子）、caddy（收纳盒） |
| 置信度阈值 | 0.45（检测）/ 0.60（发布） |
| NMS IoU 阈值 | 0.45 |
| 推理帧率 | 不低于 30 FPS（BPU 硬件加速） |

### 时域抗抖滤波

对每个目标独立维护抗抖动状态机，分三级处理：

| 级别 | 方法 | 参数 | 作用 |
|------|------|------|------|
| 第一级 | 滑窗中值滤波 | 窗口长度 6 帧 | 剔除偶发野值与脉冲噪声 |
| 第二级 | 步长极限裁剪 | 最大 2.5cm/帧 | 防止异常跳变，物理安全兜底 |
| 第三级 | 指数平滑（EMA） | 平滑系数 0.35 | 消除随机抖动，输出平滑轨迹 |

### 手眼标定

采用 Eye-to-Hand 方案，相机固定于工作区外。通过 ArUco 标记点结合 SVD 算法预先标定 4x4 齐次变换矩阵，将相机坐标系一步映射为机械臂基座坐标系。标定基于 19 组采样点，RMSE 约 3.8cm。标定后在矩阵变换后叠加经验性软件偏置补偿，修正结构件形变等系统性偏差。

### 输出格式

节点向 `/target_grasp_array` 话题发布 JSON 字符串：

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

坐标单位为厘米，参考系为机械臂基座坐标系。仅当置信度大于 0.60 且 X 方向距离小于 37.0cm 时发布。

### 运行方式

```bash
# 模型诊断
python3 test.py

# 纯 2D 检测（调试用）
python3 yolo_bpu_detect_only.py

# 完整 3D 定位（生产环境）
python3 yolo_3d.py
```

---

## Pour-just-one-cup-arm — 机械臂控制

完整的六自由度 LeArm 机械臂 ROS2 控制栈，覆盖下位机固件至上位机任务编排的全部层级。

### 模块结构

| 模块 | 语言 | 说明 |
|------|------|------|
| `arm_base/` | C++ | 串口通信桥，ROS2 topic 与 LeArm 串口协议双向转换 |
| `arm_msg/` | — | 自定义 ROS2 消息定义（Arm / ArmStatus / GripperCmd） |
| `arm_cmd/` | C++ | 坐标命令发布节点，用于手动发送目标点 |
| `arm_test/` | Python | 逆运动学求解、测试序列、手眼标定采集与解算脚本 |
| `arm_task/` | Python | 倒茶/倒水任务编排节点，状态机驱动 |
| `LeArm/` | C | STM32F103RB 下位机固件（CubeMX HAL / Hiwonder / Keil 工程） |
| `calibration_data/` | — | 手眼标定结果存档 |
| `tools/` | Python | 串口调试脚本 |

### ROS2 节点与数据通道

| 节点 | 订阅的话题 | 发布的话题 | 作用 |
|------|-----------|-----------|------|
| `arm_base` | `/arm_cmd`, `/gripper_cmd`, `/joint_target` | `/arm_status` | 串口桥接与协议转换 |
| `ik_node.py` | `/arm_cmd` | `/joint_target`, `/arm_status` | 解析几何法逆运动学求解 |
| `tea_task_node` | `/target_grasp_array`, `/task_cmd` | `/arm_cmd`, `/gripper_cmd` | 泡茶任务状态机编排 |
| `calib_collect.py` | TF, `/arm_status` | — | 手眼标定点对采集与 SVD 求解 |

### 逆运动学控制模式

系统实现了解析几何法 IK，在 ROS2 侧（上位机）完成逆运动学求解，绕过 STM32 下位机闭源 IK 库，直接发送六路舵机目标角度。此方案消除了下位机坐标累积误差，同时开放了关节运动插值与轨迹规划的完全控制权。

DH 参数：L1=2.89cm, L2=10.43cm, L3=8.90cm, L4=17.70cm。

### 泡茶任务流程

`tea_task_node` 采用严格的三阶段状态机（MOVING -> ARRIVED/SETTLING -> ACTING），支持两条完整执行管线：

- **茶盒/茶叶任务**（10 步）：接近 -> 抓取 -> 提升 -> 移动 -> 翻腕倾倒（-90 度）-> 归位 -> 释放
- **茶壶/倒水任务**（13 步）：归零 -> 接近（Y 方向 -3cm 偏置）-> 抓取 -> 后撤 -> 倾斜注水（-60 度）-> 归位 -> 释放

夹爪通过独立话题 `/grip_cmd` 控制，与机械臂关节通道隔离，避免相互干扰。

### 手眼标定

采用 Eye-to-Hand 方案，ArUco 标记固定于机械爪/腕部，相机固定于工作区外。通过 `calib_collect.py` 从 TF 树读取标记位姿并记录机械臂末端坐标，在采集足够点对后自动通过 Kabsch-Umeyama SVD 算法求解相机到基座的刚体变换矩阵。标定结果保存为 `hand_eye_result.json`，供 `my_vision` 模块加载使用。

### 编译与运行

```bash
# 编译
colcon build --symlink-install
source install/setup.bash

# 启动串口通信桥（传统坐标模式）
ros2 launch arm_base arm_bringup.launch.py \
  usart_port_name:=ttyUSB0 \
  serial_baud_rate:=9600

# 启动 IK 控制模式
ros2 launch arm_test ik_bringup.launch.py \
  target_x:=15.0 target_y:=0.0 target_z:=2.0

# 启动泡茶任务
ros2 launch arm_task tea_task.launch.py move_time_ms:=1500

# 触发任务
ros2 topic pub -1 /task_cmd std_msgs/msg/String "{data: '1'}"   # 茶盒任务
ros2 topic pub -1 /task_cmd std_msgs/msg/String "{data: '2'}"   # 茶壶任务
```

---

## 完整启动流程

### 硬件清单

- 地平线 RDK X5 开发板
- Nuwa-HP60C3D 结构光深度相机（或兼容的 RGB-D 相机）
- STM32F103RB + LeArm 六自由度机械臂
- ESP32 语音交互设备

### 启动步骤

**终端 1：机械臂串口桥**

```bash
cd ./arm_ws && source install/setup.bash
ros2 launch arm_base arm_bringup.launch.py \
  usart_port_name:=ttyUSB0 \
  serial_baud_rate:=9600
```

**终端 2：视觉检测节点**

```bash
cd ./vision_ws && source install/setup.bash
python3 yolo_3d.py
```

**终端 3：泡茶任务节点**

```bash
cd ./arm_ws && source install/setup.bash
ros2 run arm_task tea_task_node
```

**终端 4：语音服务**

```bash
cd ./cup_arm
python app.py
```

### 使用方式

1. ESP32 设备通过 WebSocket 连接 `ws://<RDK_X5_IP>:8000/xiaozhi/v1/`
2. 说出"帮我泡杯茶"，系统自动完成语音识别、意图解析、视觉定位、机械臂抓取与倾倒全流程
3. 执行完毕后经 TTS 语音播报结果

---

## 技术栈总览

| 层级 | 技术选型 |
|------|----------|
| 语音检测 | Silero VAD (ONNX 本地推理) |
| 语音识别 | FunASR SenseVoiceSmall（本地）/ 阿里云 / 百度 / 腾讯 / OpenAI 等 15 种方案 |
| 大语言模型 | ChatGLM + Function Calling，兼容 OpenAI / Coze / Dify / Ollama 等 9 种后端 |
| 语音合成 | EdgeTTS（免费）/ 阿里云 / 字节豆包等 18 种方案 |
| 视觉检测 | YOLOv8 + BPU 硬件加速（地平线 RDK X5） |
| 3D 定位 | RGB-D 深度估计 + 手眼标定矩阵 + 三级时域滤波 |
| 机器人框架 | ROS2 Humble (rclpy / rclcpp) |
| 运动学 | 解析几何法逆运动学（上位机求解） |
| 下位机 | STM32F103RB + HAL + LeArm 舵机协议 |
| 通信协议 | WebSocket / ROS2 Topic / ROS2 Service / 串口 (USB-TTL, RS-485) |
| 音频编码 | Opus (24kHz, 60ms 帧, 单声道) |

---

## 性能指标

| 指标 | 数值 |
|------|------|
| YOLOv8 推理帧率 | 不低于 30 FPS（BPU 硬件加速） |
| 3D 定位精度 | RMSE 约 3.8cm（手眼标定，19 组采样点） |
| 坐标抖动 | 不超过正负 2mm（空间滤波 + 时域滤波） |
| 语音响应延迟 | 流式处理，首字延迟小于 1 秒 |

---

## 许可证

MIT

---

## 相关资源

- [xiaozhi-esp32-server](https://github.com/xinnan-tech/xiaozhi-esp32-server) — 开源语音对话服务框架
- 地平线 RDK X5 — BPU 硬件加速推理平台

---

> 三个子项目各有独立的 README.md 文件，提供更详细的技术说明。本文档为系统级汇总概览。
