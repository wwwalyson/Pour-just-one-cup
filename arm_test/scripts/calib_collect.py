#!/usr/bin/env python3
"""手眼标定数据采集 + 求解节点 (配合 easy_handeye2 ArUco 检测器)。

前提:
  1. ArUco 标记贴在腕关节 (机械爪根部)
  2. aruco_detector 节点已运行, 通过 TF 发布标记在相机坐标系下的位姿
  3. arm_base 已运行, 发布 arm_status

工作流:
  1. 移动机械臂到不同位置
  2. 到位后按 Enter, 同时从 TF 和 arm_status 记录点对
  3. 采集 >=15 组后自动 SVD 求解, 保存 hand_eye_result.json

完整启动命令见文件末尾注释。
"""

import json
import os
import sys
import threading
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from arm_msg.msg import ArmStatus
import tf2_ros


# ──────────────────────────────────────────────────────────────────
# SVD 求解
# ──────────────────────────────────────────────────────────────────

def solve_rigid_transform(P_cam, P_base):
    """Kabsch-Umeyama: 相机坐标系 → 基坐标系"""
    centroid_cam = np.mean(P_cam, axis=0)
    centroid_base = np.mean(P_base, axis=0)
    Q_cam = P_cam - centroid_cam
    Q_base = P_base - centroid_base
    H = Q_cam.T @ Q_base
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = centroid_base - R @ centroid_cam
    P_pred = (R @ P_cam.T).T + t
    rmse = np.sqrt(np.mean(np.linalg.norm(P_pred - P_base, axis=1) ** 2))
    return R, t, rmse


def make_4x4(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


# ──────────────────────────────────────────────────────────────────
# 数据采集节点
# ──────────────────────────────────────────────────────────────────

class CalibCollect(Node):

    def __init__(self):
        super().__init__('calib_collect')

        # ---- 参数 ----
        self.declare_parameter('camera_frame', 'ascamera_color_0')
        self.declare_parameter('marker_frame', 'aruco_marker')
        self.declare_parameter('min_samples', 25)
        self.declare_parameter('data_dir', './calibration_data')
        self.declare_parameter('tf_timeout', 0.5)  # TF 数据过期时间 (秒)

        self._camera_frame = self.get_parameter('camera_frame').value
        self._marker_frame = self.get_parameter('marker_frame').value
        self._min_samples = self.get_parameter('min_samples').value
        self._data_dir = self.get_parameter('data_dir').value
        self._tf_timeout = self.get_parameter('tf_timeout').value

        os.makedirs(self._data_dir, exist_ok=True)

        # ---- TF 监听 ----
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- 机械臂位置 ----
        self._latest_pos = None  # (x, y, z) cm
        self._status_received = False

        self._status_sub = self.create_subscription(
            ArmStatus, 'arm_status', self._status_cb,
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))

        # ---- 采集数据 ----
        self._points_cam = []   # list of (3,) np.array in cm
        self._points_base = []  # list of (3,) np.array in cm

        # ---- 键盘输入 (线程设置标志位, 定时器在主线程采集) ----
        self._trigger = False
        self._input_thread = threading.Thread(
            target=self._keyboard_loop, daemon=True)
        self._input_thread.start()
        self._timer = self.create_timer(0.2, self._check_trigger)

        # ---- 等 arm_status ----
        self.get_logger().info('等待 arm_status...')
        self._wait_for_status()

        self.get_logger().info(
            f'标定采集就绪.\n'
            f'  TF: {self._camera_frame} → {self._marker_frame}\n'
            f'  操作: 移动机械臂 → 按 Enter 记录点对\n'
            f'  至少 {self._min_samples} 组后自动求解并保存')

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------

    def _status_cb(self, msg):
        self._latest_pos = np.array([msg.x, msg.y, msg.z], dtype=np.float32)
        self._status_received = True

    # ------------------------------------------------------------------
    # 采集一次
    # ------------------------------------------------------------------

    def _record_sample(self):
        if self._latest_pos is None:
            self.get_logger().warn('尚无 arm_status, 请确认 arm_base 已启动')
            return

        # 1. 从 TF 获取标记在相机坐标系下的位置
        try:
            t = self._tf_buffer.lookup_transform(
                self._camera_frame,
                self._marker_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=self._tf_timeout))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF 查询失败: {e}')
            return

        # TF translation 单位是米, 转为厘米
        P_cam = np.array([
            t.transform.translation.x * 100.0,
            t.transform.translation.y * 100.0,
            t.transform.translation.z * 100.0,
        ], dtype=np.float32)

        P_base = self._latest_pos.copy()

        # 2. 保存
        self._points_cam.append(P_cam)
        self._points_base.append(P_base)
        n = len(self._points_cam)

        self.get_logger().info(
            f'[{n:2d}] cam=({P_cam[0]:6.2f}, {P_cam[1]:6.2f}, {P_cam[2]:6.2f})  '
            f'base=({P_base[0]:5.2f}, {P_base[1]:5.2f}, {P_base[2]:5.2f})')

        # 3. 够数则求解
        if n >= self._min_samples:
            self.get_logger().info(f'已达 {n} 组, 开始求解...')
            self._solve_and_save()

    # ------------------------------------------------------------------
    # SVD 求解 & 保存
    # ------------------------------------------------------------------

    def _solve_and_save(self):
        P_cam = np.array(self._points_cam)
        P_base = np.array(self._points_base)
        R, t, rmse = solve_rigid_transform(P_cam, P_base)

        self.get_logger().info(f'========== 标定结果 ==========')
        self.get_logger().info(f'样本数: {len(P_cam)},  RMSE: {rmse:.3f} cm')
        self.get_logger().info(f'旋转矩阵 R (cam→base):\n{R}')
        self.get_logger().info(f'平移向量 t (cam→base, cm): {t}')

        # 保存
        result = {
            'timestamp': datetime.now().isoformat(),
            'camera_frame': self._camera_frame,
            'marker_frame': self._marker_frame,
            'num_samples': len(P_cam),
            'rmse_cm': float(rmse),
            'rotation': R.tolist(),
            'translation': t.tolist(),
            'transform_4x4': make_4x4(R, t).tolist(),
            'points_cam': [p.tolist() for p in self._points_cam],
            'points_base': [p.tolist() for p in self._points_base],
        }

        path = os.path.join(self._data_dir, 'hand_eye_result.json')
        with open(path, 'w') as f:
            json.dump(result, f, indent=2, ensure_ascii=False)
        self.get_logger().info(f'已保存至 {path}')

    # ------------------------------------------------------------------
    # 键盘 & 等待
    # ------------------------------------------------------------------

    def _keyboard_loop(self):
        while rclpy.ok():
            try:
                line = sys.stdin.readline()
                if not line:
                    break
                self._trigger = True
            except EOFError:
                break

    def _check_trigger(self):
        if self._trigger:
            self._trigger = False
            self._record_sample()

    def _wait_for_status(self, timeout=5.0):
        import time
        start = time.time()
        while rclpy.ok() and not self._status_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().error('超时未收到 arm_status')
                raise RuntimeError('arm_status timeout')


def main():
    rclpy.init()
    node = CalibCollect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# ======================================================================
# 完整启动命令
# ======================================================================
#
# ┌─ 机器A (RDK/树莓派, 运行相机 + ArUco 检测) ─────────────────┐
# │
# │  # 1. 启动深度相机
# │  cd ~/ros_ws && source install/setup.bash
# │  ros2 launch ascamera ascamera.launch.py
# │
# │  # 2. 启动 ArUco 检测器
# │  cd ~/easy_handeye2_ws && source install/setup.bash
# │  ros2 run aruco_detector aruco_detector_node \
# │    --ros-args -p marker_id:=0 -p marker_size:=0.05 \
# │               -p marker_frame:=aruco_marker \3
# │               -p camera_frame:=camera_color_optical_frame
# │
# └────────────────────────────────────────────────────────────┘
#
# ┌─ 机器B (PC, 运行 arm_ws) ───────────────────────────────────┐
# │
# │  # 3. 启动机械臂通信
# │  cd ~/arm_ws && source install/setup.bash
# │  ros2 launch arm_base arm_bringup.launch.py
# │
# │  # 4. PD 控制器 (平滑移动)
# │  ros2 run arm_test pd_controller.py
# │
# │  # 5. 标定采集
# │  ros2 run arm_test calib_collect.py
# │
# │  # 6. 移动机械臂 + 按 Enter 采集
# │  ros2 topic pub /target arm_msg/msg/Arm "{x: 14.0, y: 2.0, z: 8.0}"
# │  # 到位后按 Enter
# │  ros2 topic pub /target arm_msg/msg/Arm "{x: 17.0, y: -3.0, z: 5.0}"
# │  # 到位后按 Enter
# │  # ... 重复 15+ 次
# │
# └────────────────────────────────────────────────────────────┘
