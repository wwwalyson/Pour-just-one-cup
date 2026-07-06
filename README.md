
# cup_arm

语音控制机械臂自动泡茶系统。基于 [xiaozhi-server](https://github.com/xinnan-tech/xiaozhi-esp32-server) 开源项目扩展开发，部署于地平线 **RDK X5** 上位机，搭配 STM32 下位机、六自由度 LeArm 机械臂及深度相机，实现从自然语言指令到物理抓取执行的全自动茶艺闭环。

## 整体架构

```
┌──────────────────────────────────────────────────┐
│                    用户                           │
│           "帮我泡杯茶"（自然语音指令）              │
└──────────────────┬───────────────────────────────┘
                   ▼
┌──────────────────────────────────────────────────┐
│  RDK X5 上位机 (xiaozhi-server)                   │
│                                                   │
│  语音管线：VAD → ASR → LLM → TTS（语音回复）        │
│  视觉管线：深度相机 → YOLOv8(BPU) → 3D 坐标计算    │
│  控制管线：LLM 意图解析 → Function Calling         │
│                → ROS2 Service → tea_task_node      │
└──────────────────┬───────────────────────────────┘
                   │  ROS2 + 串口
                   ▼
┌──────────────────────────────────────────────────┐
│  STM32 下位机 + 六自由度 LeArm 机械臂              │
│                                                   │
│  舵机驱动 → 轨迹规划 → 夹取 → 翻腕倒茶 → 注水 → 复位 │
└──────────────────────────────────────────────────┘
```

## 语音处理流程

设备端（ESP32）采集用户语音经由 Opus 编码（24kHz / 60ms 帧 / 单声道），通过 WebSocket 发送至 RDK X5 上的 xiaozhi-server。服务端依次完成：

1. **VAD 语音检测**（Silero VAD，ONNX 本地推理）—— 实时判定语音起止
2. **ASR 语音识别**（FunASR SenseVoiceSmall）—— 语音转文字，支持情绪与语种检测
3. **LLM 意图解析**（ChatGLM + Function Calling）—— 若为普通对话，生成文本回复；若为泡茶指令，调用 `ros2_arm_control` 工具函数
4. **TTS 语音合成**（EdgeTTS，免费）—— 文本分句后流式合成语音，Opus 编码回传设备播放

核心处理文件：[core/handle/receiveAudioHandle.py](core/handle/receiveAudioHandle.py)、[core/handle/sendAudioHandle.py](core/handle/sendAudioHandle.py)。模块选型与参数均通过 `config.yaml` 配置。

## 视觉定位流程

深度相机（Nuwa-HP60C3D 结构光）实时采集 RGB-D 图像。YOLOv8 目标检测模型部署于 RDK X5 的 BPU 硬件加速引擎上，实现 ≥30 FPS 的高帧率端侧推理。检测到茶罐、茶壶等目标后，结合手眼标定矩阵和"空间中值 + 时域 EMA"双重滤波算法，解算出目标在机械臂基坐标系下的稳定三维抓取坐标。

## ROS2 机械臂控制（新增代码）

在原 xiaozhi-server 基础上，新增了三处关键代码实现语音到机械臂的通信：

**【1】ROS2 环境加载 — [app.py:8-34](app.py#L8-L34)**

启动时动态将 `~/arm_ws` 和 `/opt/ros/humble` 路径注入 `sys.path`，使 Python 进程可直接 `import rclpy`。

**【2】机械臂控制插件 — [plugins_func/functions/ros2_arm_control.py](plugins_func/functions/ros2_arm_control.py)（新增文件）**

利用 xiaozhi-server 的 `@register_function` 插件机制，将机械臂控制注册为 LLM Function Calling 工具。LLM 根据用户语音自动匹配动作：

| 用户指令 | 调用参数 | 机械臂动作 |
|----------|----------|------------|
| "帮我泡茶"、"冲茶" | `make_tea` | 完整流程：倒茶叶 → 倒水 |
| "倒茶叶"、"放茶叶" | `pour_tea` | 夹取茶盒，翻腕倒茶叶 |
| "倒水"、"冲水" | `pour_water` | 夹取茶壶，倾斜注水 |

执行后将机械臂返回的状态转为口语反馈（如"泡茶流程已完成，请享用！"），经 TTS 播报给用户。

**【3】ROS2 客户端库 — [core/providers/tools/ros2_arm_client.py](core/providers/tools/ros2_arm_client.py)（新增文件）**

封装 `Ros2ArmClient` 类：source arm_ws 环境 → 预加载 `libarm_msg__rosidl_typesupport_*.so` 共享库 → 创建 rclpy 节点 → 异步调用 `/tea_command` 服务，发送 JSON 指令并等待响应。提供 `list_services()` 和 `is_service_visible()` 用于通信故障诊断。

**对应配置**（`config.yaml`）：

```yaml
# 插件参数（第 156-159 行）
plugins:
  ros2_arm_control:
    service_name: "/tea_command"
    ros_domain_id: 0
    timeout: 90

# 启用插件（第 281 行）
Intent:
  function_call:
    functions:
      - ros2_arm_control
```

## 目录结构

```
cup_arm/
├── app.py                      # 主入口，启动 WebSocket/HTTP 服务 + 加载 ROS2 环境
├── config.yaml                 # 全局配置（VAD/ASR/TTS/LLM/插件/ROS2 等）
├── core/
│   ├── connection.py           # 连接处理器，会话状态管理
│   ├── handle/                 # 消息处理
│   │   ├── receiveAudioHandle.py   # 语音接收 → VAD → ASR → 对话
│   │   ├── sendAudioHandle.py      # TTS 音频发送，精准帧率控制
│   │   └── intentHandler.py        # 意图分析
│   ├── providers/
│   │   ├── asr/                # 15 种 ASR 方案
│   │   ├── tts/                # 18 种 TTS 方案
│   │   ├── vad/                # Silero VAD
│   │   ├── llm/                # LLM 适配器
│   │   └── tools/              # 工具调用框架
│   │       ├── ros2_arm_client.py       # ★ ROS2 机械臂客户端
│   │       ├── ros2_drone_client.py     # ★ ROS2 无人机客户端
│   │       └── unified_tool_handler.py  # 统一工具调用处理器
│   └── utils/                  # Opus 编解码、声纹识别、音频控速等
├── plugins_func/
│   └── functions/
│       ├── ros2_arm_control.py  # ★ 机械臂泡茶控制插件
│       └── ros2_drone_control.py # ★ 无人机控制插件
├── tools/                      # 调试工具
│   ├── rdkx5_audio_bridge.py   # RDK X5 ALSA 音频桥接
│   └── ros2_drone_service_client.py
├── models/                     # 本地模型（Silero VAD）
├── config/                     # 配置模块 & 音频资源
└── performance_tester/         # TTS/ASR 性能测试
```

## 快速开始

**环境要求**

- 地平线 RDK X5 / Ubuntu 22.04 / ROS2 Humble
- Python 3.10+ / STM32 下位机 / LeArm 六自由度机械臂
- 语音设备 / Nuwa-HP60C3D 深度相机

**安装**

```bash
git clone https://github.com/wwwalyson/cup_arm.git
cd cup_arm
pip install -r requirements.txt
```

**配置**

```bash
mkdir -p data
cp config.yaml data/.config.yaml
# 编辑 data/.config.yaml 填写 API 密钥等，系统优先读取此文件
```

**启动**

```bash
# 终端 1：启动机械臂 ROS2 任务节点
cd ~/arm_ws && source install/setup.bash
ros2 run arm_task tea_task_node

# 终端 2：启动语音服务
python app.py
```

服务启动后 ESP32 通过 `ws://<RDK_X5_IP>:8000/xiaozhi/v1/` 连接，说出"帮我泡茶"即可体验完整流程。

## 主要技术特性

- **双阈值 VAD**：Silero ONNX 本地推理，迟滞机制（0.5/0.3）抗瞬时噪声
- **多引擎切换**：15 种 ASR + 18 种 TTS，通过配置一行切换
- **流式 TTS**：双线程队列 + AudioRateController 60ms 精准帧率
- **LLM 工具调用**：插件注册 + Function Calling，自动路由意图到物理动作
- **非阻塞解耦**：Python asyncio + ROS2 异步 Service，LLM 网络请求与实时控制分离
- **BPU 加速推理**：YOLOv8 模型部署于 BPU，视觉检测 ≥ 30 FPS
- **毫米级滤波**：空间中值 + 时域 EMA 双重降噪，3D 坐标抖动 ≤ ±2mm

## License

MIT
