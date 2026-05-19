import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

CUSPARSELT_LIB = '/home/afro-robotics/.local/lib/python3.10/site-packages/nvidia/cusparselt/lib'


def generate_launch_description():

    # ── Arguments (can be overridden from command line) ───────────────────────
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/afro-robotics/robot_ws/models/best.engine',
        description='Path to YOLO TensorRT engine file'
    )
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Inference device: cuda or cpu'
    )
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='USB camera device path'
    )
    conf_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.6',
        description='YOLO minimum confidence to react'
    )
    speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='25',
        description='Cruise speed in % (0-100)'
    )

    # ── GPU library path (required for TensorRT on Jetson) ────────────────────
    ld_lib_path = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        CUSPARSELT_LIB + ':' + os.environ.get('LD_LIBRARY_PATH', '')
    )

    # ── Camera node ───────────────────────────────────────────────────────────
    camera_node = Node(
        package='image_acquisition',
        executable='camera_node',
        name='camera_node',
        parameters=[{
            'device': LaunchConfiguration('camera_device'),
            'width':  640,
            'height': 480,
            'fps':    30,
        }],
        output='screen'
    )

    # ── YOLO detection node ───────────────────────────────────────────────────
    yolo_node = Node(
        package='image_acquisition',
        executable='yolo_detection_node',
        name='yolo_detection_node',
        parameters=[{
            'model_path':           LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'device':               LaunchConfiguration('device'),
            'img_size':             640,
        }],
        additional_env={'QT_QPA_PLATFORM': 'offscreen'},
        output='screen'
    )

    # ── Motor node (fuzzy speed control) ─────────────────────────────────────
    motor_node = Node(
        package='navigation',
        executable='motor_node',
        name='motor_node',
        parameters=[{
            'speed': LaunchConfiguration('speed'),
        }],
        output='screen'
    )

    return LaunchDescription([
        ld_lib_path,
        model_path_arg,
        device_arg,
        camera_device_arg,
        conf_arg,
        speed_arg,
        camera_node,
        yolo_node,
        motor_node,
    ])
