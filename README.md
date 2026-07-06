A voice-controlled robotic arm automatic tea-making system. Extended from the open-source [xiaozhi-server](https://github.com/xinnan-tech/xiaozhi-esp32-server) project and deployed on the Horizon **RDK X5** host computer, together with an STM32 MCU, a 6-DOF LeArm robotic arm, and a depth camera, this system achieves a fully automatic tea-art closed loop from natural language commands to physical grasping and execution.

## Overall Architecture

```
+--------------------------------------------------+
|                      User                         |
|        "Make me a cup of tea" (voice command)     |
+------------------------+-------------------------+
                         v
+--------------------------------------------------+
|  RDK X5 Host Computer (xiaozhi-server)            |
|                                                   |
|  Voice Pipeline:  VAD -> ASR -> LLM -> TTS        |
|  Vision Pipeline:  Depth Camera -> YOLOv8 (BPU)   |
|                    -> 3D Coordinate Calculation   |
|  Control Pipeline: LLM Intent Parsing             |
|                    -> Function Calling            |
|                    -> ROS2 Service                |
|                    -> tea_task_node               |
+------------------------+-------------------------+
                         |  ROS2 + Serial
                         v
+--------------------------------------------------+
|  STM32 MCU + 6-DOF LeArm Robotic Arm              |
|                                                   |
|  Servo Drive -> Trajectory Planning -> Grasping   |
|  -> Wrist-Roll Pouring -> Water Filling -> Reset  |
+--------------------------------------------------+
```

## Voice Processing Pipeline

The device side (ESP32) captures user speech, encodes it via Opus (24 kHz / 60 ms frames / mono), and sends it over WebSocket to xiaozhi-server running on the RDK X5. The server performs the following steps in order:

1. **VAD Voice Activity Detection** (Silero VAD, ONNX local inference) — real-time determination of speech start and end
2. **ASR Speech Recognition** (FunASR SenseVoiceSmall, local and free) — speech-to-text conversion, with emotion and language detection support
3. **LLM Intent Parsing** (ChatGLM + Function Calling) — for casual conversation, generates a text reply; for tea-making commands, invokes the `ros2_arm_control` tool function
4. **TTS Speech Synthesis** (EdgeTTS, free) — sentence-by-sentence streaming synthesis, Opus-encoded and sent back to the device for playback

Core processing files: [core/handle/receiveAudioHandle.py](core/handle/receiveAudioHandle.py), [core/handle/sendAudioHandle.py](core/handle/sendAudioHandle.py). Module selection and parameters are all configured via `config.yaml`.

## Vision Positioning Pipeline

A depth camera (Nuwa-HP60C3D structured light) captures RGB-D images in real time. The YOLOv8 object detection model is deployed on the RDK X5's BPU hardware acceleration engine, achieving edge-side inference at >= 30 FPS. After detecting targets such as tea caddies and teapots, the system combines a hand-eye calibration matrix with a dual-filtering algorithm (spatial median + temporal EMA) to compute stable 3D grasping coordinates in the robotic arm's base coordinate frame.

## ROS2 Robotic Arm Control (New Code)

Three key additions were made to the original xiaozhi-server to enable voice-to-arm communication:

**[1] ROS2 Environment Loading — [app.py:8-34](app.py#L8-L34)**

At startup, the paths `~/arm_ws` and `/opt/ros/humble` are dynamically injected into `sys.path`, allowing the Python process to directly `import rclpy`.

**[2] Robotic Arm Control Plugin — [plugins_func/functions/ros2_arm_control.py](plugins_func/functions/ros2_arm_control.py) (new file)**

Using xiaozhi-server's `@register_function` plugin mechanism, arm control operations are registered as LLM Function Calling tools. The LLM automatically matches actions based on the user's spoken command:

| User Command | Call Parameter | Arm Action |
|-------------|---------------|------------|
| "make me tea", "brew tea" | `make_tea` | Full workflow: pour tea leaves -> pour water |
| "pour tea leaves", "add leaves" | `pour_tea` | Grasp tea caddy, wrist-roll to pour leaves |
| "pour water", "add water" | `pour_water` | Grasp teapot, tilt to pour water |

After execution, the arm's return status is converted into a spoken reply (e.g., "The tea-making process is complete. Enjoy!") and broadcast to the user via TTS.

**[3] ROS2 Client Library — [core/providers/tools/ros2_arm_client.py](core/providers/tools/ros2_arm_client.py) (new file)**

Encapsulates the `Ros2ArmClient` class: source the arm_ws environment -> preload `libarm_msg__rosidl_typesupport_*.so` shared libraries -> create an rclpy node -> asynchronously call the `/tea_command` service, send a JSON command, and wait for a response. Provides `list_services()` and `is_service_visible()` methods for communication fault diagnosis.

**Corresponding Configuration** (`config.yaml`):

```yaml
# Plugin parameters (lines 156-159)
plugins:
  ros2_arm_control:
    service_name: "/tea_command"
    ros_domain_id: 0
    timeout: 90

# Enable plugin (line 281)
Intent:
  function_call:
    functions:
      - ros2_arm_control
```

## Directory Structure

```
cup_arm/
├── app.py                      # Main entry point; starts WebSocket/HTTP services + loads ROS2 environment
├── config.yaml                 # Global configuration (VAD/ASR/TTS/LLM/plugins/ROS2, etc.)
├── core/
│   ├── connection.py           # Connection handler, session state management
│   ├── handle/                 # Message processing
│   │   ├── receiveAudioHandle.py   # Audio input -> VAD -> ASR -> dialogue
│   │   ├── sendAudioHandle.py      # TTS audio output, precise frame-rate control
│   │   └── intentHandler.py        # Intent analysis
│   ├── providers/
│   │   ├── asr/                # 15 ASR backends
│   │   ├── tts/                # 18 TTS backends
│   │   ├── vad/                # Silero VAD
│   │   ├── llm/                # LLM adapters
│   │   └── tools/              # Tool invocation framework
│   │       ├── ros2_arm_client.py       # ROS2 robotic arm client (NEW)
│   │       ├── ros2_drone_client.py     # ROS2 drone client (NEW)
│   │       └── unified_tool_handler.py  # Unified tool invocation handler
│   └── utils/                  # Opus codec, voiceprint, audio rate control, etc.
├── plugins_func/
│   └── functions/
│       ├── ros2_arm_control.py  # Robotic arm tea-making control plugin (NEW)
│       └── ros2_drone_control.py # Drone control plugin (NEW)
├── tools/                      # Debugging utilities
│   ├── rdkx5_audio_bridge.py   # RDK X5 ALSA audio bridge
│   └── ros2_drone_service_client.py
├── models/                     # Local models (Silero VAD)
├── config/                     # Configuration module & audio assets
└── performance_tester/         # TTS/ASR performance testing
```

## Quick Start

**Environment Requirements**

- Horizon RDK X5 / Ubuntu 22.04 / ROS2 Humble
- Python 3.10+ / STM32 MCU / LeArm 6-DOF robotic arm
- ESP32 voice device / Nuwa-HP60C3D depth camera

**Installation**

```bash
git clone https://github.com/wwwalyson/cup_arm.git
cd cup_arm
pip install -r requirements.txt
```

**Configuration**

```bash
mkdir -p data
cp config.yaml data/.config.yaml
# Edit data/.config.yaml to fill in API keys; the system reads this file first
```

**Startup**

```bash
# Terminal 1: Start the robotic arm ROS2 task node
cd ~/arm_ws && source install/setup.bash
ros2 run arm_task tea_task_node

# Terminal 2: Start the voice service
python app.py
```

Once the service is running, connect the ESP32 via `ws://<RDK_X5_IP>:8000/xiaozhi/v1/` and say "make me a cup of tea" to experience the complete workflow.

## Key Technical Features

- **Dual-threshold VAD**: Silero ONNX local inference, hysteresis mechanism (0.5/0.3) for transient noise immunity
- **Multi-engine switching**: 15 ASR + 18 TTS backends, switchable with a single config line
- **Streaming TTS**: dual-thread queue + AudioRateController for precise 60 ms frame pacing
- **LLM tool invocation**: plugin registration + Function Calling, automatically routing intent to physical actions
- **Non-blocking decoupling**: Python asyncio + ROS2 async Services, separating LLM network requests from real-time control
- **BPU-accelerated inference**: YOLOv8 model deployed on BPU; vision detection at >= 30 FPS
- **Millimeter-level filtering**: spatial median + temporal EMA dual noise reduction; 3D coordinate jitter <= +-2 mm

## License

MIT
