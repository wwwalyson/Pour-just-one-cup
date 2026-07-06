# Omni-Vision — Robot 3D Vision Positioning System Based on Horizon RDK X5

## Project Overview

This project is deployed on the **Horizon RDK X5 development board** and runs in a **ROS 2 Humble** environment. The system perceives desktop objects (cups, spoons, bottles, storage boxes) in real time through an RGB-D depth camera, performs object detection using a BPU hardware-accelerated YOLOv8 model, and combines depth information with a hand-eye calibration matrix to convert pixel coordinates into **real 3D grasp coordinates in the robotic arm base coordinate system**. The results are then published to downstream grasping modules via ROS 2 topics.

## Project Structure

```
my_vision/
├── yolo_3d.py                          # Main node: 3D detection + depth estimation + coordinate transform + temporal filtering
├── yolo_bpu_detect_only.py             # Detection-only node: 2D object detection and visualization (for debugging)
├── test.py                             # Model diagnostic tool: prints BPU model input/output tensor information
├── hand_eye_result.json                # Hand-eye calibration result (camera → robotic arm 4×4 transformation matrix)
├── best_640_480_bayese_640x640_nv12.bin# YOLOv8 BPU compiled model (3.7 MB, NV12 input)
├── setup.py                            # ROS 2 package build configuration
├── start_vision.sh                     # One-click startup script
└── README.md                           # This file
```

## Environment Dependencies

| Dependency | Description |
|---|---|
| **Hardware** | Horizon RDK X5 development board (BPU inference) |
| **Camera** | RGB-D depth camera |
| **OS** | Ubuntu + ROS 2 Humble |
| **SDK** | `hobot_dnn_rdkx5` (Horizon BPU Python SDK) |
| **Python** | `rclpy`, `sensor_msgs`, `cv_bridge`, `opencv-python`, `numpy`, `message_filters` |

## Algorithm Pipeline

```
Camera Driver
  ├── RGB Image (/ascamera/.../rgb0/image)
  ├── Depth Image (/ascamera/.../depth0/image_raw)
  └── Camera Intrinsics (/ascamera/.../camera_info)
          │
          ▼
  Time Synchronization (ApproximateTimeSynchronizer, 200 ms tolerance)
          │
          ▼
  BGR → Letterbox 640×640 → NV12 Conversion
          │
          ▼
  BPU YOLOv8 Inference (Horizon RDK X5 hardware-accelerated)
          │
          ▼
  DFL Bounding Box Decoding → NMS Deduplication
          │
          ▼
  ROI Depth Extraction (29×29 region + 20th percentile)
          │
          ▼
  Pinhole Camera Back-Projection → Camera 3D Coordinates
          │
          ▼
  Hand-Eye Calibration Transform (4×4 homogeneous matrix) → Robotic Arm 3D Coordinates
          │
          ▼
  Three-Stage Temporal Anti-Jitter Filtering (Median → Step Clipping → Exponential Smoothing)
          │
          ▼
  Dual-Condition Filtering: Confidence (>0.60) + Reachability (arm span <37 cm)
          │
          ▼
  Publish JSON to /target_grasp_array
```

## Core Algorithm Details

### 1. YOLOv8 DFL Bounding Box Decoding

YOLOv8 does not directly regress bounding box coordinates; instead, it predicts 16 discrete probability values for each edge (Distribution Focal Loss). During decoding, Softmax normalization is applied to the 16 values, and the expected value is computed to achieve sub-pixel precision bounding box localization.

### 2. Robust Depth Estimation

A **29×29 pixel ROI** is taken around the object center. Invalid depth values (zeros) are filtered out, and the **20th percentile** is used. Compared to single-point reading or mean filtering, this strategy effectively suppresses distant background noise and stably captures the true depth of the target surface.

### 3. Three-Stage Temporal Anti-Jitter Filtering

An independent anti-jitter state machine is maintained for each target, with three layers of protection:

| Stage | Method | Parameters | Purpose |
|---|---|---|---|
| Stage 1 | Sliding-window median filter | Window = 6 frames | Eliminate sporadic outliers and impulse noise |
| Stage 2 | Step clipping | Max = 2.5 cm/frame | Prevent anomalous jumps; physical safety fallback |
| Stage 3 | Exponential Moving Average (EMA) | α = 0.35 | Smooth random jitter for stable trajectories |

### 4. Hand-Eye Calibration and Coordinate Transformation

The system reads a 4×4 homogeneous transformation matrix pre-calibrated via ArUco markers + SVD algorithm from `hand_eye_result.json`, mapping camera-frame coordinates to robotic arm base coordinates in a single step. Calibration is based on 19 sample points with an RMSE of approximately 3.8 cm.

### 5. Software Bias Compensation

After matrix transformation, empirical compensation is applied to correct systematic deviations such as mechanical deformation of the robotic arm structure, with preset grasp heights for different objects (cup/bottle). Final safety validation is performed using the arm span limit (37 cm).

## Model Information

| Item | Details |
|---|---|
| Architecture | YOLOv8 (Anchor-Free, DFL Head) |
| Input | 640×480 RGB, Letterbox to 640×640, NV12 format |
| Classes | `cup(0)`, `spoon(1)`, `bottle(2)`, `caddy(3)` |
| Confidence Threshold | 0.45 (detection) / 0.60 (publishing) |
| NMS IoU Threshold | 0.45 |

## How to Run

### Model Diagnostics

```bash
python3 test.py
```

### 2D Detection Node Only (for debugging)

```bash
python3 yolo_bpu_detect_only.py
```

### Full 3D Positioning Node (for production)

```bash
python3 yolo_3d.py
```

Once started, the node will automatically:
- Subscribe to camera topics
- Load the BPU model and hand-eye calibration matrix
- Display detection results in real time (OpenCV window)
- Publish target JSON coordinates to the `/target_grasp_array` topic

### Output Format

Data format published to the `/target_grasp_array` topic:

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

> Coordinate unit: centimeters. Coordinate system: robotic arm base coordinate system.
> Published only when confidence > 0.60 and X < 37.0 cm.
