"""Publish deterministic synthetic images for the M3 ROS 2 pipeline."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImagePublisherNode(Node):
    """Generate left, center, right, near, no-target and silence inputs."""

    VALID_SCENARIOS = {
        'left',
        'center',
        'right',
        'near',
        'no_target',
        'silence',
    }

    def __init__(self):
        """Create the image publisher and its periodic callback."""
        super().__init__('image_publisher_node')

        # Periodic scheduling must use monotonic time. ROS/system time can
        # jump when a virtual machine resynchronizes its guest clock.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.bridge = CvBridge()

        # This parameter is read on every timer tick, so it can be changed
        # while the node is running with `ros2 param set`.
        self.declare_parameter('scenario', 'center')

        self.publisher_ = self.create_publisher(
            Image,
            '/camera/image_raw',
            10,
        )

        # Normal publication rate: 10 Hz.
        self.timer = self.create_timer(
            0.1,
            self.timer_callback,
            clock=self.steady_clock,
        )
        self.last_invalid_scenario = None

        self.get_logger().info(
            'Image publisher started on /camera/image_raw'
        )

    def timer_callback(self):
        """Publish one deterministic BGR frame for the selected scenario."""
        scenario = str(self.get_parameter('scenario').value)

        if scenario not in self.VALID_SCENARIOS:
            if scenario != self.last_invalid_scenario:
                self.get_logger().error(
                    f'Unknown scenario: {scenario}; image output is paused'
                )
                self.last_invalid_scenario = scenario
            return

        self.last_invalid_scenario = None

        # Silence is deliberately different from no_target: no Image message
        # is published, so /target_info will eventually stop as well.
        if scenario == 'silence':
            return

        width = 320
        height = 240
        image = np.zeros((height, width, 3), dtype=np.uint8)

        center_y = height // 2
        radius = 18

        if scenario == 'left':
            center_x = width // 4
        elif scenario == 'center':
            center_x = width // 2
        elif scenario == 'right':
            center_x = width * 3 // 4
        elif scenario == 'near':
            center_x = width // 2
            radius = 55
        else:
            # no_target still publishes a valid black image, but it contains
            # no yellow target that the detector can accept.
            center_x = None

        if center_x is not None:
            # OpenCV uses BGR. (0, 255, 255) is yellow.
            cv2.circle(
                image,
                (center_x, center_y),
                radius,
                (0, 255, 255),
                -1,
            )

        image_message = self.bridge.cv2_to_imgmsg(
            image,
            encoding='bgr8',
        )
        image_message.header.stamp = self.get_clock().now().to_msg()
        image_message.header.frame_id = 'camera_link'

        self.publisher_.publish(image_message)


def main(args=None):
    """Run the image publisher node."""
    rclpy.init(args=args)
    node = ImagePublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
