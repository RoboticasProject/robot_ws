"""
test_lines.launch.py — Camera + line follower + test_follow_lines only.
navigation_line_node is NOT started so test_follow_lines has full motor control.
Usage:
  ros2 launch navigation test_lines.launch.py
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

CUSPARSELT_LIB = '/home/afro-robotics/.local/lib/python3.10/site-packages/nvidia/cusparselt/lib'


def generate_launch_description():

    camera_device_arg = DeclareLaunchArgument(
        'camera_device', default_value='auto',
        description='Chemin périphérique caméra USB')
    line_dark_arg = DeclareLaunchArgument(
        'line_dark', default_value='True',
        description='True = ruban noir sur sol clair')
    threshold_arg = DeclareLaunchArgument(
        'threshold', default_value='80',
        description='Seuil de binarisation (0–255)')
    roi_top_arg = DeclareLaunchArgument(
        'roi_top_fraction', default_value='0.45',
        description='Début ROI depuis le haut')

    ld_lib_path = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        CUSPARSELT_LIB + ':' + os.environ.get('LD_LIBRARY_PATH', '')
    )

    encoder_node = Node(
        package='navigation',
        executable='encoder_node',
        name='encoder_node',
        output='screen'
    )

    camera_node = Node(
        package='image_acquisition',
        executable='camera_node',
        name='camera_node',
        parameters=[{
            'device': LaunchConfiguration('camera_device'),
            'width': 640, 'height': 480, 'fps': 30,
        }],
        output='screen'
    )

    line_follower_node = Node(
        package='navigation',
        executable='line_follower_node',
        name='line_follower_node',
        parameters=[{
            'line_dark':        LaunchConfiguration('line_dark'),
            'threshold':        LaunchConfiguration('threshold'),
            'roi_top_fraction': LaunchConfiguration('roi_top_fraction'),
            'min_line_area':    800,
            'intersection_width_ratio': 0.65,
        }],
        output='screen'
    )

    test_follow_lines_node = Node(
        package='navigation',
        executable='test_follow_lines',
        name='test_follow_lines',
        output='screen'
    )

    return LaunchDescription([
        ld_lib_path,
        camera_device_arg,
        line_dark_arg,
        threshold_arg,
        roi_top_arg,
        encoder_node,
        camera_node,
        line_follower_node,
        test_follow_lines_node,
    ])
