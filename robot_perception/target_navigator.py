import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from cv_bridge import CvBridge
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import cv2
import numpy as np
import math
from robot_perception_interfaces.msg import YoloDetectionArray

class TargetNavigator(Node):
    def __init__(self):
        super().__init__('target_navigator')
        self.bridge = CvBridge()
        self.declare_parameter('target_mode', 'color')  # 'color' 或 'person'
        self.target_mode = self.get_parameter('target_mode').value
        self.declare_parameter('camera_height', 0.107)
        self.declare_parameter('min_box_height', 300)
        self.declare_parameter('search_angular_speed', 0.35)
        self.camera_height = self.get_parameter('camera_height').value
        self.min_box_height = self.get_parameter('min_box_height').value
        self.search_angular_speed = self.get_parameter('search_angular_speed').value

        # 摄像头参数（来自waffle系列实际配置）
        self.image_width = 1920
        self.horizontal_fov = 1.085595  # 弧度
        # 人的平均身高（米），追人模式用单目测距估算距离
        self.person_height = 1.7

        # 目标颜色（可以改成你想找的颜色）
        self.target_color = 'red'
        self.color_ranges = {
            'red': ([0, 100, 100], [10, 255, 255]),
            'green': ([40, 70, 70], [80, 255, 255]),
            'blue': ([100, 100, 100], [130, 255, 255]),
            'yellow': ([20, 100, 100], [35, 255, 255]),
        }

        self.latest_scan = None
        self.active_goal_handle = None
        self.nav_goal_pending = False
        self.current_goal_pose = None
        self.last_goal_update_time = 0.0
        self.camera_info = None
        self.last_detection_time = 0.0
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 订阅检测结果和雷达（颜色模式看相机，人形模式看 YOLO 检测话题）
        if self.target_mode == 'person':
            self.create_subscription(YoloDetectionArray,
                                     '/yolo_detector/detections',
                                     self.detection_callback, 10)
            self.create_subscription(CameraInfo,
                                     '/camera/camera_info',
                                     self.camera_info_callback, 10)
            self.search_timer = self.create_timer(0.2, self.search_callback)
        else:
            self.create_subscription(Image, '/camera/image_raw',
                                     self.image_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # TF2 用于坐标转换
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        target_desc = 'person' if self.target_mode == 'person' else self.target_color
        self.get_logger().info(
            f'目标搜寻节点已启动，模式={self.target_mode}，正在寻找: {target_desc}')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def image_callback(self, msg):
        if self.latest_scan is None:
            return

        self.image_width = msg.width
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

    def detection_callback(self, msg):
        # 把同一帧所有 person 框合并成一个整体：
        # 角色可能被 YOLO 拆成上下两个框，单独取一个会导致测距偏大
        x_min = y_min = None
        x_max = y_max = 0.0
        found = False
        for det in msg.detections:
            if det.class_name != 'person':
                continue
            x1 = det.center_x - det.width / 2.0
            y1 = det.center_y - det.height / 2.0
            x2 = det.center_x + det.width / 2.0
            y2 = det.center_y + det.height / 2.0
            x_min = x1 if x_min is None else min(x_min, x1)
            y_min = y1 if y_min is None else min(y_min, y1)
            x_max = max(x_max, x2)
            y_max = max(y_max, y2)
            found = True

        if found:
            self.last_detection_time = self.get_clock().now().nanoseconds / 1e9
            self.image_width = msg.image_width
            center_x = (x_min + x_max) / 2.0
            bottom_y = y_max
            height = y_max - y_min
            self.stop_search()
            self.get_logger().info(
                f'发现目标 person，像素中心x={center_x:.0f}, 框高={height:.0f}, 底边y={bottom_y:.0f}')
            self.navigate_to_target(center_x, height, bottom_y)

    def navigate_to_target(self, pixel_center_x, box_height=None, box_bottom_y=None):
        if self.nav_goal_pending:
            return

        goal_pose = self.compute_target_pose(pixel_center_x, box_height, box_bottom_y)
        if goal_pose is None:
            return

        x = goal_pose.pose.position.x
        y = goal_pose.pose.position.y

        if self.current_goal_pose is not None:
            dx = x - self.current_goal_pose[0]
            dy = y - self.current_goal_pose[1]
            target_moved = math.hypot(dx, dy)
            now = self.get_clock().now().nanoseconds / 1e9
            # 目标基本没动时不要反复向 Nav2 发同一个点
            if target_moved < 0.35 and self.active_goal_handle is not None:
                return
            # 目标移动了，但避免高频重发导致 Nav2 来回取消/重新规划
            if (self.active_goal_handle is not None and
                    now - self.last_goal_update_time < 0.8):
                return

        self.current_goal_pose = (x, y)
        self.last_goal_update_time = self.get_clock().now().nanoseconds / 1e9
        self.nav_goal_pending = True
        self.stop_search()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.get_logger().info(f'发送导航目标: x={x:.2f}, y={y:.2f}')

        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('Nav2 action server 未就绪，跳过本次目标')
            self.nav_goal_pending = False
            return

        try:
            future = self.nav_client.send_goal_async(goal_msg)
        except Exception as e:
            self.get_logger().warn(f'发送导航目标失败: {e}')
            self.nav_goal_pending = False
            return

        future.add_done_callback(self.goal_response_callback)

    def compute_target_pose(self, pixel_center_x, box_height=None, box_bottom_y=None):
        # 1. 计算角度偏移(相对相机正前方，左负右正)
        if self.camera_info is not None:
            fx = self.camera_info.k[0]
            cx = self.camera_info.k[2]
            angle = math.atan2(pixel_center_x - cx, fx)
        else:
            offset_ratio = (pixel_center_x - self.image_width / 2.0) / (self.image_width / 2.0)
            angle = offset_ratio * (self.horizontal_fov / 2.0)

        # 2. 估算距离：追人模式用单目测距（actor 无碰撞体，雷达测不到）；
        #    颜色模式沿用雷达测距
        if box_height is not None and box_height > 0:
            if box_height < self.min_box_height:
                self.get_logger().info(
                    f'person 框高 {box_height:.0f} 过小，可能是局部检测，暂不导航')
                return None

            if self.camera_info is not None and box_bottom_y is not None:
                fy = self.camera_info.k[4]
                cy = self.camera_info.k[5]
                dy = box_bottom_y - cy
                if dy < 20.0:
                    self.get_logger().warn(
                        f'person 底边接近地平线，无法可靠测距: dy={dy:.1f}')
                    return None
                distance = self.camera_height * fy / dy
                self.get_logger().info(f'地面投影估算距离: {distance:.2f} m')
            else:
                focal = (self.image_width / 2.0) / math.tan(self.horizontal_fov / 2.0)
                distance = self.person_height * focal / box_height
                self.get_logger().info(f'单目估算距离: {distance:.2f} m')
        else:
            if self.latest_scan is None:
                self.get_logger().warn('尚未收到雷达数据，跳过')
                return None
            distance = self.get_distance_at_angle(angle)
            if distance is None:
                self.get_logger().warn('该方向雷达数据无效，跳过')
                return None

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
        # 让机器人到达目标时也朝向目标，而不是保持发送目标那一刻的朝向
        target_yaw = math.atan2(local_y, local_x)
        pose_in_base.pose.orientation = self.yaw_to_quaternion(target_yaw)

        # 5. 转换到map坐标系
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            pose_in_map = do_transform_pose(pose_in_base.pose, transform)
        except Exception as e:
            self.get_logger().warn(f'TF转换失败: {e}')
            return None

        # 6. 返回 map 坐标系下的目标位姿
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = pose_in_map
        return goal_pose

    def goal_response_callback(self, future):
        self.nav_goal_pending = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('导航目标被Nav2拒绝！可能是目标点不可达（比如离障碍物太近）')
            self.active_goal_handle = None
            return
        self.active_goal_handle = goal_handle
        self.get_logger().info('导航目标已被接受，开始执行')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.active_goal_handle = None
        try:
            result = future.result()
            self.get_logger().info(f'导航结果: {result}')
        except Exception as e:
            self.get_logger().warn(f'读取导航结果失败: {e}')

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def search_callback(self):
        if self.target_mode != 'person':
            return
        if self.active_goal_handle is not None or self.nav_goal_pending:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_detection_time < 2.0:
            return

        twist = Twist()
        twist.angular.z = self.search_angular_speed
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('未检测到 person，原地旋转搜索...', throttle_duration_sec=2.0)

    def stop_search(self):
        if self.target_mode != 'person':
            return
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

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
