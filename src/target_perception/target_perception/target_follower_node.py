"""Convert TargetInfo into Twist commands with a separate watchdog."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from target_interfaces.msg import TargetInfo


class TargetFollowerNode(Node):
    """Follow a detected target and stop on loss, proximity or timeout."""

    def __init__(self):
        """Create the TargetInfo subscription, command publisher and timer."""
        super().__init__('target_follower_node')

        # Watchdog elapsed time must be monotonic. VM guest system/ROS time
        # may jump during synchronization and would otherwise cause a false
        # timeout immediately followed by "stream recovered".
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('angular_gain', 0.8)
        self.declare_parameter('lateral_deadband', 0.05)
        self.declare_parameter('near_area_ratio', 0.05)
        self.declare_parameter('message_timeout', 1.0)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            TargetInfo,
            '/target_info',
            self.target_callback,
            10,
        )

        # This callback runs independently of target_callback. Therefore it can
        # detect complete /target_info interruption.
        self.watchdog_timer = self.create_timer(
            0.1,
            self.watchdog_callback,
            clock=self.steady_clock,
        )

        self.last_message_time = None
        self.watchdog_active = False

        self.get_logger().info(
            'Target follower started: /target_info -> /cmd_vel'
        )

    def publish_stop(self):
        """Publish a Twist whose linear and angular components are all zero."""
        self.publisher_.publish(Twist())

    def target_callback(self, message):
        """Handle one newly received TargetInfo message."""
        # Any received message, including detected=False, proves the topic is
        # currently alive. Use local receipt time for the watchdog.
        self.last_message_time = self.steady_clock.now()

        if self.watchdog_active:
            self.get_logger().info('TargetInfo stream recovered')
        self.watchdog_active = False

        # Case 1: a new image was processed, but no target was detected.
        if not message.detected:
            self.publish_stop()
            return

        lateral = float(message.lateral)
        area_ratio = float(message.distance)

        if (
            not math.isfinite(lateral)
            or not math.isfinite(area_ratio)
            or area_ratio < 0.0
            or area_ratio > 1.0
        ):
            self.get_logger().error(
                'Invalid TargetInfo values received; stopping'
            )
            self.publish_stop()
            return

        forward_speed = float(
            self.get_parameter('forward_speed').value
        )
        angular_gain = float(
            self.get_parameter('angular_gain').value
        )
        lateral_deadband = float(
            self.get_parameter('lateral_deadband').value
        )
        near_area_ratio = float(
            self.get_parameter('near_area_ratio').value
        )

        if (
            forward_speed < 0.0
            or angular_gain < 0.0
            or lateral_deadband < 0.0
            or near_area_ratio <= 0.0
            or near_area_ratio > 1.0
        ):
            self.get_logger().error('Invalid control parameters; stopping')
            self.publish_stop()
            return

        # Case 2: a target exists, but its area ratio indicates it is too near.
        if area_ratio >= near_area_ratio:
            self.publish_stop()
            return

        lateral = max(-1.0, min(1.0, lateral))
        if abs(lateral) < lateral_deadband:
            lateral = 0.0

        command = Twist()
        command.linear.x = forward_speed

        # lateral<0 means target left. ROS angular.z>0 is a left/CCW turn,
        # hence the minus sign.
        command.angular.z = -angular_gain * lateral
        self.publisher_.publish(command)

    def watchdog_callback(self):
        """Publish zero velocity after /target_info stops for too long."""
        timeout = float(self.get_parameter('message_timeout').value)

        if timeout <= 0.0:
            if not self.watchdog_active:
                self.get_logger().error(
                    'message_timeout must be positive; stopping'
                )
            self.watchdog_active = True
            self.publish_stop()
            return

        # Startup is also a no-command-safe state.
        if self.last_message_time is None:
            if not self.watchdog_active:
                self.get_logger().warning(
                    'No TargetInfo received yet; keeping zero velocity'
                )
            self.watchdog_active = True
            self.publish_stop()
            return

        now = self.steady_clock.now()
        elapsed = (now - self.last_message_time).nanoseconds / 1e9

        # Case 3: /target_info itself has stopped. This is deliberately
        # separate from receiving a new detected=False message.
        if elapsed > timeout:
            if not self.watchdog_active:
                self.get_logger().warning(
                    f'TargetInfo timeout after {elapsed:.3f}s; '
                    'publishing zero velocity'
                )
            self.watchdog_active = True
            self.publish_stop()


def main(args=None):
    """Run the target follower node."""
    rclpy.init(args=args)
    node = TargetFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
