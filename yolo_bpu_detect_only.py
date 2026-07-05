import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

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
    
    boxes = []
    scores = []
    class_ids = [] # 新增：记录类别 ID
    
    for i, (cls_out, reg_out) in enumerate(feature_maps):
        stride = strides[i]
        
        # 将所有类别的输出转换为 0~1 的概率
        cls_logits = np.clip(cls_out, -10, 10) 
        score_map = 1 / (1 + np.exp(-cls_logits)) 
        
        # 找出每个像素点上概率最大的类别，及其分数
        max_scores = np.max(score_map, axis=-1)
        max_class_indices = np.argmax(score_map, axis=-1)
        
        # 仅保留大于置信度阈值的点
        y_idx, x_idx = np.where(max_scores > conf_thres)
        
        for y, x in zip(y_idx, x_idx):
            s = max_scores[y, x]
            c_id = max_class_indices[y, x]
            
            # 解析边界框 (DFL 解码)
            reg = reg_out[y, x].reshape(4, 16)
            reg_exp = np.exp(reg)
            reg_softmax = reg_exp / np.sum(reg_exp, axis=1, keepdims=True)
            dist = np.sum(reg_softmax * np.arange(16), axis=1)
            
            # 还原到 640x640 画面坐标系
            cx = (x + 0.5) * stride
            cy = (y + 0.5) * stride
            xmin = cx - dist[0] * stride
            ymin = cy - dist[1] * stride
            xmax = cx + dist[2] * stride
            ymax = cy + dist[3] * stride
            
            boxes.append([int(xmin), int(ymin), int(xmax - xmin), int(ymax - ymin)])
            scores.append(float(s))
            class_ids.append(int(c_id))
            
    if len(boxes) == 0:
        return [], [], []
        
    # 非极大值抑制 (NMS)，剔除重叠框
    indices = cv2.dnn.NMSBoxes(boxes, scores, conf_thres, 0.45)
    
    final_boxes, final_scores, final_class_ids = [], [], []
    if len(indices) > 0:
        for idx in indices:
            if isinstance(idx, (list, np.ndarray)):
                idx = idx[0]
            final_boxes.append([boxes[idx][0], boxes[idx][1], boxes[idx][0] + boxes[idx][2], boxes[idx][1] + boxes[idx][3]])
            final_scores.append(scores[idx])
            final_class_ids.append(class_ids[idx])
            
    return final_boxes, final_scores, final_class_ids


class YoloBpuDetectOnlyNode(Node):
    def __init__(self):
        super().__init__('yolo_bpu_detect_only_node')
        self.bridge = CvBridge()
        
        # 挂载真实的专属模型路径
        model_path = '/home/sunrise/vision_ws/best_640_480_bayese_640x640_nv12.bin' 
        if not os.path.exists(model_path):
            self.get_logger().error(f"❌ 未找到模型，请检查路径: {model_path}")
            rclpy.shutdown()
        
        self.models = dnn.load(model_path)
        self.get_logger().info("✅ BPU AI 引擎 (Multi-Class 模式) 点火完毕！")

        # 仅订阅 RGB 图像话题
        self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.rgb_callback,
            10)
        
        self.get_logger().info("🚀 节点就绪，正在等待 RGB 画面流...")

    def rgb_callback(self, rgb_msg):
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"❌ RGB 图像转换失败: {e}")
            return

        # 1. BPU 预处理：尺寸对齐与 Letterbox 等比填充
        img_bpu = np.full((640, 640, 3), 114, dtype=np.uint8)
        top_pad = 80
        cv_rgb_resized = cv2.resize(cv_rgb, (640, 480))
        img_bpu[top_pad:top_pad+480, 0:640] = cv_rgb_resized

        # 2. 格式转换：BGR 转 NV12
        nv12_data = bgr2nv12(img_bpu)
        
        # 3. BPU 推理
        try:
            outputs = self.models[0].forward([nv12_data])
        except Exception as e:
            self.get_logger().error(f"❌ BPU 推理失败: {e}")
            return
        
        # 4. 后处理解码 (返回框、分数、以及类别 ID)
        boxes, scores, class_ids = decode_yolov8(outputs, conf_thres=0.45)
        
        # 定义类别字典和对应的框颜色 (绿, 蓝, 橙, 紫)
        class_names = {0: 'cup', 1: 'spoon', 2: 'bottle', 3: 'caddy'}
        colors = {0: (0, 255, 0), 1: (255, 0, 0), 2: (0, 165, 255), 3: (255, 0, 255)}

        if len(boxes) > 0:
            for box, score, cid in zip(boxes, scores, class_ids):
                xmin_bpu, ymin_bpu, xmax_bpu, ymax_bpu = box
                
                # 5. 逆向还原坐标到原始 640x480 彩色图
                xmin = max(0, xmin_bpu)
                xmax = min(639, xmax_bpu)
                ymin = max(0, ymin_bpu - top_pad)
                ymax = min(479, ymax_bpu - top_pad)

                u = int((xmin + xmax) / 2)
                v = int((ymin + ymax) / 2)

                # 获取目标名字和颜色
                obj_name = class_names.get(cid, f"Unknown_{cid}")
                color = colors.get(cid, (255, 255, 255))

                # 画面渲染
                cv2.rectangle(cv_rgb, (int(xmin), int(ymin)), (int(xmax), int(ymax)), color, 2)
                cv2.circle(cv_rgb, (u, v), 5, (0, 0, 255), -1)
                
                text = f"{obj_name} ({score:.2f})"
                cv2.putText(cv_rgb, text, (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
                # 终端精简打印
                self.get_logger().info(f"🎯 发现: {obj_name} | 置信度: {score:.2f}")

        # 弹出实时检测画面
        cv2.imshow("RDK X5 BPU Vision (Multi-Class)", cv_rgb)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloBpuDetectOnlyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()