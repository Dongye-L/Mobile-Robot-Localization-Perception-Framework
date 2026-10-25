from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('uwb_pkg')
    ekf_config = os.path.join(package_dir, 'config', 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='uwb_pkg',
            executable='uwb_node',
            name='uwb_node',
            output='screen',
        ),
        Node(
            package='uwb_pkg',
            executable='imu_node',
            name='wt901_imu_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baudrate': 115200,
                'frame_id': 'imu_link',
            }],
        ),
        Node(
            package='uwb_pkg',
            executable='uwb_pose_adapter',
            name='uwb_pose_adapter',
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_base_link',
            arguments=[
               '0', '0', '0',
               '0', '0', '0',
               'base_link', 'imu_link'
            ],
        ),
        Node(
        package='uwb_pkg',
        executable='filtered_path_node',
        name='filtered_path_node',
        output='screen',
        ),
    ])