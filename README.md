## Chiese [README_cn.md](./README_cn.md)
# Voice-Controlled Robotic Arm Tea-Making System

This system consists of three sub-projects that together form a fully automated closed loop: from natural language voice commands to physical robotic arm execution. The user speaks a command; the system performs speech recognition and LLM intent parsing, invokes the vision module for 3D object localization, and finally controls the robotic arm to execute grasping and pouring actions.

---

## Project Overview

| Project | Path | Role | Responsibility |
|---------|------|------|----------------|
| xiaozhi-server | `./xiaozhi-server/` | Voice interaction and task orchestration | Voice pipeline (VAD/ASR/LLM/TTS), intent parsing, function calling |
| my_vision | `./my_vision/` | Visual perception | RGB-D object detection, 3D coordinate computation, hand-eye calibration |
| Pour-just-one-cup-arm | `./Pour-just-one-cup-arm/` | Robotic arm control | Serial communication, inverse kinematics, grasp and pour motion sequencing |

---

## System Architecture

```
                              +------------------------+
                              |         User           |
                              |  "Make me a cup of tea"|
                              +-----------+------------+
                                          | WebSocket (Opus)
                                          v
+-----------------------------------------------------------------------+
|  RDK X5 Host Computer                                                  |
|                                                                        |
|  +-------------------------+     +-------------------------+           |
|  |  xiaozhi-server         |     |  my_vision              |           |
|  |                         |     |                         |           |
|  |  VAD -> ASR -> LLM     |     |  RGB-D Camera           |           |
|  |    |                    |     |    |                    |           |
|  |    v                    |     |    v                    |           |
|  |  Function Calling      |<--->|  YOLOv8 (BPU accel.)   |           |
|  |    |                    |     |    |                    |           |
|  |    v                    |     |    v                    |           |
|  |  TTS -> Voice Reply     |     |  3D Coords + Calib.    |           |
|  +-----------+-------------+     +-----------+-------------+           |
|              |                               |                         |
|              |  ROS2 Service                 |  ROS2 Topic             |
|              |  /tea_command                 |  /target_grasp_array    |
|              +-------------+-----------------+                         |
+----------------------------+------------------------------------------+
                             | Serial (USB-TTL)
                             v
+-----------------------------------------------------------------------+
|  STM32 MCU + LeArm 6-DOF Robotic Arm                                  |
|                                                                        |
|  IK -> Servo Angles -> Trajectory -> Grasp -> Pour -> Return          |
+-----------------------------------------------------------------------+
```

---

## Data Flow

```
User Voice
  |
  v
ESP32 Device (Opus encoding, 24kHz, 60ms frames)
  |
  v  WebSocket
xiaozhi-server
  +-- VAD (Silero ONNX, local inference)
  +-- ASR (FunASR SenseVoiceSmall, local/free)
  +-- LLM (ChatGLM + Function Calling)
  |     +-- Casual chat -> TTS (EdgeTTS) -> voice reply
  |     +-- Tea command -> ros2_arm_control tool function
  |                          |
  |                          v  ROS2 Service: /tea_command
  |                 arm_task/tea_task_node.py
  |                    |
  |                    +-- Request target coordinates
  |                    |     |
  |                    |     v  ROS2 Topic: /target_grasp_array
  |                    |  my_vision/yolo_3d.py
  |                    |     +-- RGB-D image acquisition
  |                    |     +-- BPU YOLOv8 object detection
  |                    |     +-- Depth estimation + coordinate transform
  |                    |     +-- Returns JSON: [{class, x, y, z, score}]
  |                    |
  |                    v
  |             Grasp planning -> arm_base serial bridge -> STM32 -> servo execution
  |
  +-- Action complete -> TTS voice announcement
```

---

## xiaozhi-server — Voice Interaction and Task Orchestration

Extended from the open-source [xiaozhi-esp32-server](https://github.com/xinnan-tech/xiaozhi-esp32-server) project, deployed on the Horizon RDK X5 host computer.

### Key Files

| File | Description |
|------|-------------|
| `app.py` | Main entry point; starts WebSocket/HTTP services, loads ROS2 environment |
| `config.yaml` | Global configuration (VAD/ASR/TTS/LLM/plugins/ROS2 parameters) |
| `core/handle/receiveAudioHandle.py` | Voice receive pipeline: VAD -> ASR -> LLM conversation |
| `core/handle/sendAudioHandle.py` | TTS streaming synthesis with frame-rate-controlled transmission |
| `plugins_func/functions/ros2_arm_control.py` | Robotic arm tea-making plugin (LLM Function Calling) |
| `core/providers/tools/ros2_arm_client.py` | ROS2 Service async client wrapper |

### Voice Processing Pipeline

1. **VAD (Voice Activity Detection)**: Silero VAD via ONNX local inference, dual-threshold hysteresis to determine speech boundaries
2. **ASR (Automatic Speech Recognition)**: FunASR SenseVoiceSmall (local and free), supports emotion and language detection
3. **LLM Intent Parsing**: ChatGLM + Function Calling -- casual conversation generates text replies; tea-making commands invoke the `ros2_arm_control` tool function
4. **TTS (Text-to-Speech)**: EdgeTTS (free), sentence-level streaming synthesis, Opus-encoded and sent back to the device

### Technical Features

- Multi-engine support: 15 ASR backends and 18 TTS backends, switchable via configuration
- Plugin registration: decorator-based `@register_function` plugin system with auto-discovery
- Streaming TTS output: dual-thread queue with AudioRateController for precise 60ms frame pacing
- Async decoupling: Python asyncio combined with ROS2 async Services, separating LLM network I/O from real-time control

### Environment Requirements

- Horizon RDK X5 development board
- Ubuntu 22.04 / ROS2 Humble / Python 3.10+

---

## my_vision — Visual Perception

A ROS2 Python node deployed on the RDK X5, performing real-time 3D object detection and localization using BPU hardware acceleration.

### Key Files

| File | Description |
|------|-------------|
| `yolo_3d.py` | Main node: 3D detection, depth estimation, coordinate transform, temporal filtering, result publishing |
| `yolo_bpu_detect_only.py` | 2D-only detection node for debugging and visualization |
| `test.py` | BPU model diagnostic tool; prints input/output tensor information |
| `hand_eye_result.json` | Hand-eye calibration result (4x4 homogeneous transform, 19 sample pairs, RMSE ~3.8 cm) |
| `best_640_480_bayese_640x640_nv12.bin` | Compiled YOLOv8 BPU model (3.7 MB, NV12 input format) |
| `setup.py` | ROS2 package build configuration |
| `start_vision.sh` | One-click launch script |

### Algorithm Pipeline

```
RGB-D Image -> Letterbox 640x640 -> NV12 Conversion -> BPU YOLOv8 Inference
    -> DFL Bounding Box Decoding -> NMS Suppression
    -> ROI Depth Extraction (29x29 px region, 20th percentile)
    -> Pinhole Back-Projection -> Camera-Frame 3D Coordinates
    -> Hand-Eye Transform (4x4 Homogeneous Matrix) -> Robot-Base 3D Coordinates
    -> Three-Stage Temporal Anti-Jitter Filter (Median -> Step Clamp -> EMA)
    -> Dual-Condition Filter (confidence > 0.60, reach < 37 cm)
    -> Publish JSON to /target_grasp_array
```

### Object Detection

| Item | Specification |
|------|---------------|
| Model Architecture | YOLOv8 (Anchor-Free, DFL Head) |
| Input | 640x480 RGB, Letterbox to 640x640, NV12 format |
| Classes | cup, spoon, bottle, caddy |
| Confidence Threshold | 0.45 (detection) / 0.60 (publishing) |
| NMS IoU Threshold | 0.45 |
| Inference Rate | >= 30 FPS (BPU hardware acceleration) |

### Temporal Anti-Jitter Filtering

Each tracked object maintains an independent anti-jitter state machine with three layers:

| Layer | Method | Parameter | Purpose |
|-------|--------|-----------|---------|
| Layer 1 | Sliding-window median filter | Window size 6 frames | Remove sporadic outliers and impulse noise |
| Layer 2 | Step-limit clamping | Max 2.5 cm/frame | Prevent abrupt jumps; physical safety guard |
| Layer 3 | Exponential smoothing (EMA) | Smoothing factor 0.35 | Eliminate random jitter; produce smooth trajectories |

### Hand-Eye Calibration

Uses an Eye-to-Hand configuration: the camera is fixed outside the workspace while an ArUco marker is attached to the gripper/wrist. A 4x4 homogeneous transformation matrix is pre-calibrated using ArUco marker detection combined with SVD, mapping camera-frame coordinates directly to the robot-arm base frame. Calibration is based on 19 sample pairs with an RMSE of approximately 3.8 cm. After the matrix transform, empirical software bias compensation is applied to correct systematic errors such as structural deformation.

### Output Format

The node publishes JSON strings to the `/target_grasp_array` topic:

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

Coordinate unit: centimeters. Reference frame: robot arm base coordinate system. Targets are published only when confidence exceeds 0.60 and the X-axis distance is less than 37.0 cm.

### Usage

```bash
# Model diagnostics
python3 test.py

# 2D detection only (debug mode)
python3 yolo_bpu_detect_only.py

# Full 3D positioning (production mode)
python3 yolo_3d.py
```

---

## Pour-just-one-cup-arm — Robotic Arm Control

A complete ROS2 control stack for the LeArm 6-DOF robotic arm, covering the full spectrum from MCU firmware to host-side task orchestration.

### Module Structure

| Module | Language | Description |
|--------|----------|-------------|
| `arm_base/` | C++ | Serial communication bridge: bidirectional ROS2 topic <-> LeArm protocol conversion |
| `arm_msg/` | — | Custom ROS2 message definitions (Arm / ArmStatus / GripperCmd) |
| `arm_cmd/` | C++ | Coordinate command publisher node for manual target point sending |
| `arm_test/` | Python | Inverse kinematics solver, test sequences, hand-eye calibration collection and computation |
| `arm_task/` | Python | Tea-pouring task orchestration node, state-machine-driven |
| `LeArm/` | C | STM32F103RB firmware (CubeMX HAL / Hiwonder / Keil project) |
| `calibration_data/` | — | Archived hand-eye calibration results |
| `tools/` | Python | Serial debugging scripts |

### ROS2 Nodes and Data Channels

| Node | Subscribes To | Publishes To | Purpose |
|------|--------------|-------------|---------|
| `arm_base` | `/arm_cmd`, `/gripper_cmd`, `/joint_target` | `/arm_status` | Serial bridging and protocol conversion |
| `ik_node.py` | `/arm_cmd` | `/joint_target`, `/arm_status` | Analytical geometric IK solver |
| `tea_task_node` | `/target_grasp_array`, `/task_cmd` | `/arm_cmd`, `/gripper_cmd` | Tea task state machine orchestration |
| `calib_collect.py` | TF, `/arm_status` | — | Hand-eye calibration point collection and SVD solving |

### Inverse Kinematics Control Mode

The system implements an analytical geometric IK solver on the ROS2 (host) side, bypassing the STM32's closed-source IK library. It sends six servo target angles directly, eliminating cumulative coordinate errors on the MCU side and granting full control over joint motion interpolation and trajectory planning.

DH parameters: L1 = 2.89 cm, L2 = 10.43 cm, L3 = 8.90 cm, L4 = 17.70 cm.

### Tea-Making Task Pipeline

`tea_task_node` uses a strict three-phase state machine (MOVING -> ARRIVED/SETTLING -> ACTING) and supports two complete execution pipelines:

- **Tea box / tea leaf task** (10 steps): approach -> grasp -> lift -> move -> wrist-roll pour (-90 deg) -> return -> release
- **Teapot / water pouring task** (13 steps): home -> approach (Y -3 cm offset) -> grasp -> retract -> tilt pour (-60 deg) -> return -> release

The gripper is controlled via an independent `/grip_cmd` topic, isolated from the arm joint channels to prevent interference.

### Hand-Eye Calibration

Uses an Eye-to-Hand configuration with an ArUco marker attached to the gripper/wrist and a fixed external camera. The `calib_collect.py` script reads marker poses from the TF tree and records arm end-effector coordinates; after collecting sufficient point pairs, it automatically solves for the camera-to-base rigid transform via the Kabsch-Umeyama SVD algorithm. Results are saved as `hand_eye_result.json` for consumption by the `my_vision` module.

### Build and Usage

```bash
# Build
colcon build --symlink-install
source install/setup.bash

# Start serial communication bridge (traditional coordinate mode)
ros2 launch arm_base arm_bringup.launch.py \
  usart_port_name:=ttyUSB0 \
  serial_baud_rate:=9600

# Start IK control mode
ros2 launch arm_test ik_bringup.launch.py \
  target_x:=15.0 target_y:=0.0 target_z:=2.0

# Start tea-making task
ros2 launch arm_task tea_task.launch.py move_time_ms:=1500

# Trigger a task
ros2 topic pub -1 /task_cmd std_msgs/msg/String "{data: '1'}"   # tea box task
ros2 topic pub -1 /task_cmd std_msgs/msg/String "{data: '2'}"   # teapot task
```

---

## Full Startup Procedure

### Hardware Checklist

- Horizon RDK X5 development board
- Nuwa-HP60C3D structured-light depth camera (or compatible RGB-D camera)
- STM32F103RB + LeArm 6-DOF robotic arm
- ESP32 voice interaction device

### Startup Steps

**Terminal 1: Robotic Arm Serial Bridge**

```bash
cd ./arm_ws && source install/setup.bash
ros2 launch arm_base arm_bringup.launch.py \
  usart_port_name:=ttyUSB0 \
  serial_baud_rate:=9600
```

**Terminal 2: Vision Detection Node**

```bash
cd ./vision_ws && source install/setup.bash
python3 yolo_3d.py
```

**Terminal 3: Tea-Making Task Node**

```bash
cd ./arm_ws && source install/setup.bash
ros2 run arm_task tea_task_node
```

**Terminal 4: Voice Service**

```bash
cd ./cup_arm
python app.py
```

### Usage

1. Connect the ESP32 device via WebSocket to `ws://<RDK_X5_IP>:8000/xiaozhi/v1/`
2. Say "make me a cup of tea" — the system automatically performs speech recognition, intent parsing, visual localization, robotic arm grasping and pouring
3. Upon completion, a voice announcement is played via TTS

---

## Technology Stack

| Layer | Technology |
|-------|------------|
| Voice Detection | Silero VAD (ONNX local inference) |
| Speech Recognition | FunASR SenseVoiceSmall (local) / Alibaba Cloud / Baidu / Tencent / OpenAI -- 15 backends |
| Large Language Model | ChatGLM + Function Calling, compatible with OpenAI / Coze / Dify / Ollama -- 9 backends |
| Speech Synthesis | EdgeTTS (free) / Alibaba Cloud / ByteDance Doubao -- 18 backends |
| Object Detection | YOLOv8 + BPU hardware acceleration (Horizon RDK X5) |
| 3D Localization | RGB-D depth estimation + hand-eye calibration matrix + three-stage temporal filtering |
| Robotics Framework | ROS2 Humble (rclpy / rclcpp) |
| Kinematics | Analytical geometric inverse kinematics (host-side computation) |
| Microcontroller | STM32F103RB + HAL + LeArm servo protocol |
| Communication | WebSocket / ROS2 Topic / ROS2 Service / Serial (USB-TTL, RS-485) |
| Audio Codec | Opus (24 kHz, 60 ms frames, mono) |

---

## Performance Metrics

| Metric | Value |
|--------|-------|
| YOLOv8 Inference Rate | >= 30 FPS (BPU hardware acceleration) |
| 3D Localization Accuracy | RMSE ~3.8 cm (hand-eye calibration, 19 sample pairs) |
| Coordinate Jitter | <= +/- 2 mm (spatial + temporal filtering) |
| Voice Response Latency | Streaming pipeline; time-to-first-character < 1 second |

---

## License

MIT

---

## References

- [xiaozhi-esp32-server](https://github.com/xinnan-tech/xiaozhi-esp32-server) — Open-source voice conversation service framework
- Horizon RDK X5 — BPU hardware-accelerated inference platform

---

> Each sub-project has its own README.md with more detailed technical documentation. This document is a system-level overview.
