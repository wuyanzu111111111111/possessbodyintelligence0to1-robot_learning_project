"""Launch the complete M3 perception and control pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start the image publisher, detector and follower nodes."""
    # LaunchConfiguration 表示从命令行读取 launch 参数。
    scenario = LaunchConfiguration('scenario')
    min_contour_area = LaunchConfiguration('min_contour_area')
    forward_speed = LaunchConfiguration('forward_speed')
    angular_gain = LaunchConfiguration('angular_gain')
    lateral_deadband = LaunchConfiguration('lateral_deadband')
    near_area_ratio = LaunchConfiguration('near_area_ratio')
    message_timeout = LaunchConfiguration('message_timeout')

    return LaunchDescription([
        # ---------- 声明 launch 参数 ----------

        DeclareLaunchArgument(
            'scenario',
            default_value='center',
            description=(
                'Synthetic scene: left, center, right, near, '
                'no_target or silence'
            ),
        ),

        DeclareLaunchArgument(
            'min_contour_area',
            default_value='500.0',
            description='Minimum accepted contour area in pixels',
        ),

        DeclareLaunchArgument(
            'forward_speed',
            default_value='0.2',
            description='Forward speed when following a valid target',
        ),

        DeclareLaunchArgument(
            'angular_gain',
            default_value='0.8',
            description='Gain from lateral error to angular velocity',
        ),

        DeclareLaunchArgument(
            'lateral_deadband',
            default_value='0.05',
            description='Lateral error treated as centered',
        ),

        DeclareLaunchArgument(
            'near_area_ratio',
            default_value='0.05',
            description='Area ratio at which the robot must stop',
        ),

        DeclareLaunchArgument(
            'message_timeout',
            default_value='1.0',
            description='TargetInfo timeout in seconds',
        ),

        # ---------- 节点一：合成图像发布 ----------

        Node(
            package='target_perception',
            executable='image_publisher_node',
            name='image_publisher_node',
            output='screen',
            parameters=[{
                'scenario': scenario,
            }],
        ),

        # ---------- 节点二：图像检测 ----------

        Node(
            package='target_perception',
            executable='image_detector_node',
            name='image_detector_node',
            output='screen',
            parameters=[{
                # launch 参数最初是字符串，因此显式转换成 float。
                'min_contour_area': ParameterValue(
                    min_contour_area,
                    value_type=float,
                ),
            }],
        ),

        # ---------- 节点三：机器人跟随控制 ----------

        Node(
            package='target_perception',
            executable='target_follower_node',
            name='target_follower_node',
            output='screen',
            parameters=[{
                'forward_speed': ParameterValue(
                    forward_speed,
                    value_type=float,
                ),
                'angular_gain': ParameterValue(
                    angular_gain,
                    value_type=float,
                ),
                'lateral_deadband': ParameterValue(
                    lateral_deadband,
                    value_type=float,
                ),
                'near_area_ratio': ParameterValue(
                    near_area_ratio,
                    value_type=float,
                ),
                'message_timeout': ParameterValue(
                    message_timeout,
                    value_type=float,
                ),
            }],
        ),
    ])
