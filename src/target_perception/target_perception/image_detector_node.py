"""Detect the yellow target and publish a structured TargetInfo message."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from target_interfaces.msg import TargetInfo


class ImageDetectorNode(Node):
    """Convert camera images into detection state and normalized geometry."""

    def __init__(self):
        """Create image subscription, parameters and TargetInfo publisher."""
        super().__init__('image_detector_node')

        self.bridge = CvBridge()

        # Yellow is normally close to H=30 in OpenCV's HSV representation.
        self.declare_parameter('h_min', 20)
        self.declare_parameter('h_max', 40)
        self.declare_parameter('s_min', 100)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 100)
        self.declare_parameter('v_max', 255)
        self.declare_parameter('min_contour_area', 500.0)

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10,
        )
        self.publisher_ = self.create_publisher(
            TargetInfo,
            '/target_info',
            10,
        )

        self.get_logger().info(
            'Image detector started: /camera/image_raw -> /target_info'
        )

    def image_callback(self, image_message):
        """Process one input image and publish exactly one TargetInfo."""
        target_message = TargetInfo()

        # The result refers to this source image. The follower uses its own
        # local receipt time for the safety watchdog instead of trusting this.
        target_message.stamp = image_message.header.stamp
        target_message.detected = False
        target_message.lateral = 0.0
        target_message.distance = 0.0

        try:
            # ROS Image -> BGR NumPy array.
            bgr_image = self.bridge.imgmsg_to_cv2(
                image_message,
                desired_encoding='bgr8',
            )

            if bgr_image is None or bgr_image.ndim != 3:
                raise ValueError('input is not a valid three-channel image')

            height, width = bgr_image.shape[:2]
            if width <= 0 or height <= 0:
                raise ValueError('input image has an invalid size')

            # HSV separates hue from brightness and is convenient for color
            # thresholding. inRange returns a binary mask, not a new message.
            hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

            h_min = int(self.get_parameter('h_min').value)
            h_max = int(self.get_parameter('h_max').value)
            s_min = int(self.get_parameter('s_min').value)
            s_max = int(self.get_parameter('s_max').value)
            v_min = int(self.get_parameter('v_min').value)
            v_max = int(self.get_parameter('v_max').value)
            min_contour_area = float(
                self.get_parameter('min_contour_area').value
            )

            if not (
                0 <= h_min <= h_max <= 179
                and 0 <= s_min <= s_max <= 255
                and 0 <= v_min <= v_max <= 255
            ):
                raise ValueError('HSV bounds are invalid')

            if min_contour_area < 0.0:
                raise ValueError('min_contour_area cannot be negative')

            lower_bound = np.array(
                [h_min, s_min, v_min],
                dtype=np.uint8,
            )
            upper_bound = np.array(
                [h_max, s_max, v_max],
                dtype=np.uint8,
            )
            mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                contour_area = float(cv2.contourArea(largest_contour))

                if contour_area >= min_contour_area:
                    moments = cv2.moments(largest_contour)

                    # m00 is required as the denominator for the centroid.
                    if abs(moments['m00']) > 1e-6:
                        center_x = moments['m10'] / moments['m00']

                        # Negative=left, zero=center, positive=right.
                        lateral = (
                            center_x - width / 2.0
                        ) / (width / 2.0)
                        lateral = float(np.clip(lateral, -1.0, 1.0))

                        # This is an area ratio, not a calibrated metric
                        # distance. A larger value means visually closer.
                        area_ratio = contour_area / float(width * height)

                        target_message.detected = True
                        target_message.lateral = lateral
                        target_message.distance = float(area_ratio)

        except (CvBridgeError, cv2.error, ValueError) as error:
            # A malformed image or invalid detector parameter is converted to
            # a safe detected=False output for this received frame.
            self.get_logger().error(f'Image processing failed: {error}')

        # A received frame always produces exactly one TargetInfo. If no frame
        # arrives at all, this callback does not run and nothing is published.
        self.publisher_.publish(target_message)


def main(args=None):
    """Run the image detector node."""
    rclpy.init(args=args)
    node = ImageDetectorNode()

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
