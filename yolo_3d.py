import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String  
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import message_filters
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import json
import time
from collections import deque

# 引入地平线 RDK X5 专属 BPU 推理库
from hobot_dnn_rdkx5 import pyeasy_dnn as dnn


def bgr2nv12(image):
    height, width = image.shape[0], image.shape[1]
    yuv420p = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420)
    u = yuv420p[height:height+height//4, :]
    v = yuv420p[height+height//4:height+height//2, :]
    uv = np.empty((height//2, width), dtype=np.uint8)
    uv[:, 0::2] = u.reshape(-1, width//2)
    uv[:, 1::2] = v.reshape(-1, width//2)
    nv12 = np.vstack((yuv420p[:height, :], uv))
    return nv12


def decode_yolov8(outputs, conf_thres=0.45):
    strides = [8, 16, 32]
    feature_maps = [
        (outputs[0].buffer[0], outputs[1].buffer[0]), 
        (outputs[2].buffer[0], outputs[3].buffer[0]), 
        (outputs[4].buffer[0], outputs[5].buffer[0])  
    ]
    
    boxes, scores, class_ids = [], [], []
    
    for i, (cls_out, reg_out) in enumerate(feature_maps):
        stride = strides[i]
        cls_logits = np.clip(cls_out, -10, 10) 
        score_map = 1 / (1 + np.exp(-cls_logits)) 
        
        max_scores = np.max(score_map, axis=-1)
        max_class_indices = np.argmax(score_map, axis=-1)
        
        y_idx, x_idx = np.where(max_scores > conf_thres)
        
        for y, x in zip(y_idx, x_idx):
            s = max_scores[y, x]
            c_id = max_class_indices[y, x]
            
            reg = reg_out[y, x].reshape(4, 16)
            reg_exp = np.exp(reg)
            reg_softmax = reg_exp / np.sum(reg_exp, axis=1, keepdims=True)
            dist = np.sum(reg_softmax * np.arange(16), axis=1)
            
            cx, cy = (x + 0.5) * stride, (y + 0.5) * stride
            xmin = cx - dist[0] * stride
            ymin = cy - dist[1] * stride
            xmax = cx + dist[2] * stride
            ymax = cy + dist[3] * stride
            
            boxes.append([int(xmin), int(ymin), int(xmax - xmin), int(ymax - ymin)])
            scores.append(float(s))
            class_ids.append(int(c_id))
            
    if len(boxes) == 0: return [], [], []
        
    indices = cv2.dnn.NMSBoxes(boxes, scores, conf_thres, 0.45)
    final_boxes, final_scores, final_class_ids = [], [], []
    if len(indices) > 0:
        for idx in indices:
            if isinstance(idx, (list, np.ndarray)): idx = idx[0]
            final_boxes.append([boxes[idx][0], boxes[idx][1], boxes[idx][0] + boxes[idx][2], boxes[idx][1] + boxes[idx][3]])
            final_scores.append(scores[idx])
            final_class_ids.append(class_ids[idx])
            
    return final_boxes, final_scores, final_class_ids


def load_hand_eye_transform(calib_path):
    fallback_transform = np.array([
        [ 0.15014187, -0.75191271,  0.64193821,  5.65177727],
        [-0.98564070, -0.16458160,  0.03775279,  4.25186729],
        [ 0.07726442, -0.63838881, -0.76582634, 50.89577484],
        [ 0.0,         0.0,         0.0,         1.0       ]
    ], dtype=np.float32)

    if not os.path.exists(calib_path):
        return fallback_transform

    try:
        with open(calib_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        transform = np.array(data.get('transform_4x4', fallback_transform), dtype=np.float32)
        if transform.shape != (4, 4):
            return fallback_transform
        return transform
    except Exception:
        return fallback_transform


class YoloBpu3DNode(Node):
    def __init__(self):
        super().__init__('yolo_bpu_3d_node')
        self.bridge = CvBridge()
        
        # 1. 创建 ROS2 广播电台
        self.target_pub = self.create_publisher(String, '/target_grasp_array', 10)
        
        # 2. 时域抗抖动“记忆池”
        self.pose_history = {}
        self.stable_pose = {}
        self.history_len = 6
        self.alpha = 0.35
        self.max_step_cm = 2.5
        self.max_reach_x_cm = 37.0
        self.min_valid_depth_samples = 20
        self.last_log_time = {}

        # 3. 从手眼标定结果文件加载最新矩阵，避免后续标定更新时改代码
        calib_path = os.path.join(os.path.dirname(__file__), 'hand_eye_result.json')
        self.transform_4x4 = load_hand_eye_transform(calib_path)
        self.get_logger().info(f"✅ 手眼标定矩阵已加载: {calib_path}")

        # 4. 加载 BPU 模型
        model_path = '/home/sunrise/vision_ws/best_640_480_bayese_640x640_nv12.bin' 
        if not os.path.exists(model_path):
            self.get_logger().error(f"❌ 找不到模型文件: {model_path}")
            rclpy.shutdown()
            return
            
        self.models = dnn.load(model_path)
        self.get_logger().info("✅ BPU AI 视觉引擎启动就绪！画面坐标已全部切换为【机械臂基座坐标系】。")

        self.fx = self.fy = self.cx = self.cy = None

        qos_profile = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_rgb = message_filters.Subscriber(self, Image, '/ascamera/camera_publisher/rgb0/image', qos_profile=qos_profile)
        self.sub_depth = message_filters.Subscriber(self, Image, '/ascamera/camera_publisher/depth0/image_raw', qos_profile=qos_profile)
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth], 10, 0.2, allow_headerless=True)
        self.ts.registerCallback(self.image_callback)
        self.create_subscription(CameraInfo, '/ascamera/camera_publisher/rgb0/camera_info', self.info_callback, 1)

    def info_callback(self, msg):
        if self.fx is None:
            self.fx, self.cx, self.fy, self.cy = msg.k[0], msg.k[2], msg.k[4], msg.k[5]

    def stabilize_pose(self, obj_name, pose_cm):
        pose_cm = np.asarray(pose_cm, dtype=np.float32)
        history = self.pose_history.setdefault(obj_name, deque(maxlen=self.history_len))
        history.append(pose_cm)

        history_array = np.stack(history, axis=0)
        median_pose = np.median(history_array, axis=0)

        previous_pose = self.stable_pose.get(obj_name)
        if previous_pose is None:
            stable_pose = median_pose
        else:
            delta = median_pose - previous_pose
            delta = np.clip(delta, -self.max_step_cm, self.max_step_cm)
            stable_pose = previous_pose + self.alpha * delta

        self.stable_pose[obj_name] = stable_pose
        return stable_pose

    def should_log(self, obj_name, interval_sec=0.15):
        now = time.monotonic()
        last_time = self.last_log_time.get(obj_name, 0.0)
        if now - last_time < interval_sec:
            return False
        self.last_log_time[obj_name] = now
        return True

    def image_callback(self, rgb_msg, depth_msg):
        if self.fx is None: return
            
        cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

        # 图像预处理与推理
        img_bpu = np.full((640, 640, 3), 114, dtype=np.uint8)
        top_pad = 80
        img_bpu[top_pad:top_pad+480, 0:640] = cv2.resize(cv_rgb, (640, 480))

        outputs = self.models[0].forward([bgr2nv12(img_bpu)])
        boxes, scores, class_ids = decode_yolov8(outputs, conf_thres=0.45)
        
        class_names = {0: 'cup', 1: 'spoon', 2: 'bottle', 3: 'caddy'}
        colors = {0: (0, 255, 0), 1: (255, 0, 0), 2: (0, 165, 255), 3: (255, 0, 255)}

        if len(boxes) > 0:
            depth_h, depth_w = cv_depth.shape
            frame_targets = []  
            
            for box, score, cid in zip(boxes, scores, class_ids):
                xmin = max(0, box[0])
                xmax = min(639, box[2])
                ymin = max(0, box[1] - top_pad)
                ymax = min(479, box[3] - top_pad)

                u, v = int((xmin + xmax) / 2), int((ymin + ymax) / 2)
                obj_name = class_names.get(cid, f"Unknown_{cid}")
                color = colors.get(cid, (255, 255, 255))

                cv2.rectangle(cv_rgb, (int(xmin), int(ymin)), (int(xmax), int(ymax)), color, 2)
                cv2.circle(cv_rgb, (u, v), 5, (0, 0, 255), -1)

                # 深度提取 (ROI)
                box_size = 14
                depth_roi = cv_depth[max(0, v - box_size):min(depth_h, v + box_size + 1), 
                                     max(0, u - box_size):min(depth_w, u + box_size + 1)]
                valid_depths = depth_roi[depth_roi > 0]

                if len(valid_depths) >= self.min_valid_depth_samples:
                    # 空间滤波
                    z_mm = np.percentile(valid_depths, 20)
                    Z_raw = float(z_mm) / 1000.0  
                    X_raw = (u - self.cx) * Z_raw / self.fx
                    Y_raw = (v - self.cy) * Z_raw / self.fy

                    # 时域滤波
                    stable_cam_m = self.stabilize_pose(obj_name, np.array([X_raw, Y_raw, Z_raw], dtype=np.float32))
                    X_meters, Y_meters, Z_meters = stable_cam_m

                    # 矩阵乘法：计算机械臂坐标
                    P_cam_cm = np.array([X_meters * 100.0, Y_meters * 100.0, Z_meters * 100.0, 1.0])
                    P_base_cm = self.transform_4x4 @ P_cam_cm
                    Robot_X, Robot_Y, Robot_Z = P_base_cm[0], P_base_cm[1], P_base_cm[2]

                    # 软件补偿：修正系统性偏差
                    Robot_X -= 5.5
                    Robot_Y -= 2.8
                    if obj_name in ('cup', 'caddy'):
                        Robot_Z = -2.0
                    elif obj_name == 'bottle':
                        Robot_Z = 6.0

                    reachable = Robot_X < self.max_reach_x_cm
                    reach_text = "reachable" if reachable else f"x limit {self.max_reach_x_cm:.1f}cm"

                    # 画面渲染
                    text_title = f"{obj_name} ({score:.2f})"
                    text_coord = f"Arm: X{Robot_X:.1f} Y{Robot_Y:.1f} Z{Robot_Z:.1f}cm [{reach_text}]"
                    cv2.putText(cv_rgb, text_title, (int(xmin), int(ymin) - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    cv2.putText(cv_rgb, text_coord, (int(xmin), int(ymin) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    
                    if score > 0.60 and reachable:
                        # 打印稳定后的机械臂目标坐标
                        if self.should_log(obj_name):
                            self.get_logger().info(f"🎯 {obj_name} => 机械臂目标: [X={Robot_X:.1f}, Y={Robot_Y:.1f}, Z={Robot_Z:.1f}] cm")
                        
                        target_data = {
                            "class": obj_name,
                            "x": float(Robot_X),
                            "y": float(Robot_Y),
                            "z": float(Robot_Z),
                            "score": float(score)
                        }
                        frame_targets.append(target_data)
                    elif score > 0.60 and not reachable and self.should_log(obj_name):
                        self.get_logger().warn(f"⚠️ {obj_name} 坐标超出可达范围: X={Robot_X:.1f}cm, 已按 x < {self.max_reach_x_cm:.1f}cm 过滤")

            if len(frame_targets) > 0:
                msg = String()
                msg.data = json.dumps(frame_targets)
                self.target_pub.publish(msg)

        cv2.imshow("Omni-Vision Tracker", cv_rgb)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloBpu3DNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()