#!/usr/bin/env python3
# robot_perception: YOLOv8 人体检测节点
# 订阅相机图像，用 YOLOv8 检测 person，发布带标注的可视化图像

import os
import sys
import tempfile

# Ultralytics / matplotlib 需要写配置文件，默认路径可能不可写，统一指到临时目录
os.environ.setdefault('YOLO_CONFIG_DIR',
                       os.path.join(tempfile.gettempdir(), 'Ultralytics'))
os.environ.setdefault('MPLCONFIGDIR',
                       os.path.join(tempfile.gettempdir(), 'matplotlib'))

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from robot_perception_interfaces.msg import YoloDetection, YoloDetectionArray


class YoloDetector(Node):
    """基于 YOLOv8 的 person 检测节点，与颜色检测并存。"""

    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('model_path', '/tmp/yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.4)
        # COCO 类别: 0=person；可改为任意类别列表，空列表表示全部类别
        self.declare_parameter('classes', [0])
        self.declare_parameter('inference_interval', 0.3)

        model_path = self.get_parameter('model_path').value
        if not os.path.exists(model_path):
            self.get_logger().error(f'模型文件不存在: {model_path}')
            raise FileNotFoundError(f'模型文件不存在: {model_path}')

        self.get_logger().info(f'加载 YOLO 模型: {model_path}')
        self.model = YOLO(model_path)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 2)
        self.vis_pub = self.create_publisher(
            Image, '/yolo_detector/visualization', 2)
        self.det_pub = self.create_publisher(
            YoloDetectionArray, '/yolo_detector/detections', 10)

        self.last_infer_time = 0.0
        self.last_vis_msg = None

    def image_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        interval = self.get_parameter('inference_interval').value
        if now - self.last_infer_time < interval:
            # 未到推理周期，重复发布上一次的可视化结果，保持图像话题连续
            if self.last_vis_msg is not None:
                self.vis_pub.publish(self.last_vis_msg)
            return
        self.last_infer_time = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        classes = self.get_parameter('classes').value
        predict_kwargs = {'verbose': False}
        if classes:
            predict_kwargs['classes'] = classes
        results = self.model.predict(
            source=frame,
            conf=self.get_parameter('conf_threshold').value,
            **predict_kwargs)

        detected = 0
        detections = YoloDetectionArray()
        detections.header = msg.header
        detections.image_width = msg.width
        detections.image_height = msg.height
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                name = self.model.names[cls_id]
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                detected += 1

                det = YoloDetection()
                det.class_name = name
                det.confidence = conf
                det.center_x = float(cx)
                det.center_y = float(cy)
                det.width = float(x2 - x1)
                det.height = float(y2 - y1)
                detections.detections.append(det)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'{name} {conf:.2f}'
                cv2.putText(frame, label, (x1, max(y1 - 8, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                if cls_id == 0:
                    self.get_logger().info(
                        f'person 置信度={conf:.2f} 中心=({cx},{cy})')

        self.det_pub.publish(detections)
        self.get_logger().info(
            f'一帧检测到 {detected} 个目标', throttle_duration_sec=2.0)

        self.last_vis_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.last_vis_msg.header = msg.header
        self.vis_pub.publish(self.last_vis_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = YoloDetector()
    except FileNotFoundError as e:
        # 模型缺失时给出明确提示而不是抛一堆堆栈；launch 未配置 respawn，节点退出后不会重启
        rclpy.logging.get_logger('yolo_detector').error(
            f'YOLO 模型加载失败: {e}\n'
            f'请先下载权重（如 yolov8n.pt）并放在对应路径，'
            f'或通过 launch 参数 model_path:=/path/to/model.pt 指定。')
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
