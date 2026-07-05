#!/bin/bash

echo "================================================="
echo "🚀 正在启动一键 YOLOv5 视觉检测系统 (深度相机版)"
echo "================================================="

# 核心逻辑：当你按下 Ctrl+C 时，自动杀掉所有相关的后台进程
trap 'echo -e "\n🛑 收到退出指令，正在安全关闭相机和AI节点..."; kill 0; exit 0' SIGINT

echo "▶️ [1/3] 正在启动深度相机 (后台运行)..."
cd ~/ros_ws || exit
./run_ascamera_node.sh > /dev/null 2>&1 &
sleep 5

echo "▶️ [2/3] 正在加载 YOLOv5 神经网络 (后台运行)..."
cd ~  # <--- 【核心修复点】：回到主目录，确保能找到 config 文件夹！
source /opt/tros/humble/setup.bash
ros2 run dnn_node_example example \
  --ros-args -p feed_type:=1 \
  -p ros_img_topic_name:=/ascamera/camera_publisher/rgb0/image \
  -p config_file:=config/yolov5workconfig.json > /dev/null 2>&1 &
sleep 3

echo "▶️ [3/3] 正在打开实时画面预览窗口..."
cd ~/vision_ws || exit
source /opt/tros/humble/setup.bash
source install/setup.bash
ros2 run my_vision_app yolo_viewer
