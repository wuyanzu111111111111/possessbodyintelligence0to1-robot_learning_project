# ---------- 搬整个工具箱（通用的、基础的库） ----------
import rclpy
import cv2
import numpy as np
import time
import math                         # ✅ 新增：用于 sin/cos 计算

# ---------- 只拿特定工具（高频使用的类/函数） ----------
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

class PublisherNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(Image, '/synthetic_image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()

        # ✅ 新增：记录节点启动的时间（用于计算时间差，生成动态运动）
        self.start_time = time.time()

        self.get_logger().info('图像发布节点已启动')

    def timer_callback(self):
        # 1. 生成黑色底图
        image = np.zeros((480, 640, 3), dtype=np.uint8)

        # 2. 计算目标位置（使用 math 函数，现在已导入）
        elapsed = time.time() - self.start_time
        center_x = int(320 + 200 * math.sin(elapsed * 0.5))
        center_y = int(240 - 150 * math.cos(elapsed * 0.3))

        # 3. 画黄色圆（目标）
        cv2.circle(image, (center_x, center_y), 30, (0, 255, 255), -1)

        # 4. 画十字参考线
        cv2.line(image, (320, 0), (320, 480), (255, 255, 255), 1)
        cv2.line(image, (0, 240), (640, 240), (255, 255, 255), 1)

        # 5. 转换并发布
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera_link"
        self.publisher_.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode('image_publisher_node')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()