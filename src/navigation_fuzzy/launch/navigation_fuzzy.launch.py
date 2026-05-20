import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

CUSPARSELT_LIB = '/home/afro-robotics/.local/lib/python3.10/site-packages/nvidia/cusparselt/lib'


def generate_launch_description():

    # ── Arguments ─────────────────────────────────────────────────────────────
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
        default_value='/dev/video1',
        description='USB camera device path'
    )
    conf_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.6',
        description='YOLO minimum confidence to publish a detection'
    )

    # ── GPU library path (TensorRT on Jetson) ─────────────────────────────────
    ld_lib_path = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        CUSPARSELT_LIB + ':' + os.environ.get('LD_LIBRARY_PATH', '')
    )

    # ── Encoder node — owns GPIO, publishes /wheel_encoders at 50 Hz ──────────
    encoder_node = Node(
        package='navigation_fuzzy',
        executable='encoder_node',
        name='encoder_node',
        output='screen'
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

    # ── Navigation fuzzy node — straight line + both fuzzy layers ─────────────
    navigation_fuzzy_node = Node(
        package='navigation_fuzzy',
        executable='navigation_fuzzy_node',
        name='navigation_fuzzy_node',
        output='screen'
    )

    return LaunchDescription([
        ld_lib_path,
        model_path_arg,
        device_arg,
        camera_device_arg,
        conf_arg,
        encoder_node,
        camera_node,
        yolo_node,
        navigation_fuzzy_node,
    ])
