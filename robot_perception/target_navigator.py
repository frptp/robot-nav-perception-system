import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, LaserScan
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import cv2
import numpy as np
import math

class TargetNavigator(Node):
    def __init__(self):
        super().__init__('target_navigator')
        self.bridge = CvBridge()

        # 摄像头参数（来自waffle系列实际配置）
        self.image_width = 1920
        self.horizontal_fov = 1.085595  # 弧度

        # 目标颜色（可以改成你想找的颜色）
        self.target_color = 'red'
        self.color_ranges = {
            'red': ([0, 100, 100], [10, 255, 255]),
            'green': ([40, 70, 70], [80, 255, 255]),
            'blue': ([100, 100, 100], [130, 255, 255]),
            'yellow': ([20, 100, 100], [35, 255, 255]),
        }

        self.latest_scan = None
        self.goal_sent = False  # 避免重复发送目标点

        # 订阅摄像头和雷达
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # TF2 用于坐标转换
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info(f'目标搜寻节点已启动，正在寻找: {self.target_color}')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def image_callback(self, msg):
        if self.goal_sent or self.latest_scan is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower, upper = self.color_ranges[self.target_color]
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_cnt = None
        best_area = 500
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > best_area:
                best_area = area
                best_cnt = cnt

        if best_cnt is not None:
            x, y, w, h = cv2.boundingRect(best_cnt)
            center_x = x + w / 2.0
            self.get_logger().info(f'发现目标 {self.target_color}，像素中心x={center_x}')
            self.navigate_to_target(center_x)

    def navigate_to_target(self, pixel_center_x):
        # 1. 计算角度偏移(相对相机正前方，左负右正)
        offset_ratio = (pixel_center_x - self.image_width / 2.0) / (self.image_width / 2.0)
        angle = offset_ratio * (self.horizontal_fov / 2.0)

        # 2. 用雷达数据查这个角度方向的距离
        distance = self.get_distance_at_angle(angle)
        if distance is None:
            self.get_logger().warn('该方向雷达数据无效，跳过')
            return

        # 3. 计算目标在base_link坐标系下的相对位置
        # 注意：雷达角度定义与相机角度符号可能相反，需要视情况调整
        local_x = distance * math.cos(-angle)
        local_y = distance * math.sin(-angle)

        # 4. 构造一个base_link坐标系下的PoseStamped
        pose_in_base = PoseStamped()
        pose_in_base.header.frame_id = 'base_link'
        pose_in_base.header.stamp = self.get_clock().now().to_msg()
        pose_in_base.pose.position.x = local_x
        pose_in_base.pose.position.y = local_y
        pose_in_base.pose.orientation.w = 1.0

        # 5. 转换到map坐标系
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            pose_in_map = do_transform_pose(pose_in_base.pose, transform)
        except Exception as e:
            self.get_logger().warn(f'TF转换失败: {e}')
            return

        # 6. 发送Nav2导航目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = pose_in_map

        self.get_logger().info(f'发送导航目标: x={pose_in_map.position.x:.2f}, y={pose_in_map.position.y:.2f}')
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        self.goal_sent = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('导航目标被Nav2拒绝！可能是目标点不可达（比如离障碍物太近）')
            self.goal_sent = False  # 允许重新尝试
            return
        self.get_logger().info('导航目标已被接受，开始执行')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'导航结果: {result}')

    def get_distance_at_angle(self, angle):
        scan = self.latest_scan
        index = int((angle - scan.angle_min) / scan.angle_increment)
        if 0 <= index < len(scan.ranges):
            r = scan.ranges[index]
            if scan.range_min < r < scan.range_max:
                return r
        return None

def main(args=None):
    rclpy.init(args=args)
    node = TargetNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
