"""
robot_line.launch.py — Lancement complet : suivi de ligne caméra + tri de déchets.

Ordre de démarrage :
  1. encoder_node        — propriétaire GPIO ; publie /wheel_encoders @ 50 Hz
  2. camera_node         — capture USB ; publie /camera/image_raw @ 30 fps
  3. yolo_detection_node — inférence TensorRT ; publie /detections
  4. line_follower_node  — détection ligne OpenCV ; publie /line_error (CPU)
  5. navigation_line_node — machine à états ; contrôle PCA9685

Streams de debug disponibles une fois lancé :
  http://<IP_ROBOT>:8080  — MJPEG YOLO (bboxes déchets annotées)
  http://<IP_ROBOT>:8081  — MJPEG ligne (ROI + centroïde annotés)

Arguments :
  camera_device          /dev/video0  (ou /dev/video1 selon USB)
  model_path             chemin vers best.engine
  confidence_threshold   0.6
  device                 cuda
  line_dark              True   (ruban noir sur sol clair)
  threshold              80     (seuil de binarisation)
  roi_top_fraction       0.60   (bas 40 % du frame)

Usage :
  ros2 launch navigation robot_line.launch.py
  ros2 launch navigation robot_line.launch.py camera_device:=/dev/video1
  ros2 launch navigation robot_line.launch.py threshold:=60 line_dark:=True
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

CUSPARSELT_LIB = '/home/afro-robotics/.local/lib/python3.10/site-packages/nvidia/cusparselt/lib'


def generate_launch_description():

    # ── Arguments ─────────────────────────────────────────────────────────────
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='auto',
        description='Chemin périphérique caméra USB (auto = détection automatique)'
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/afro-robotics/robot_ws/models/best.engine',
        description='Chemin modèle YOLO (TensorRT .engine recommandé)'
    )
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Dispositif inférence : cuda ou cpu'
    )
    conf_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.45',
        description='Confiance minimale YOLO pour publier une détection'
    )
    line_dark_arg = DeclareLaunchArgument(
        'line_dark',
        default_value='True',
        description='True = ruban noir sur sol clair  /  False = blanc sur sombre'
    )
    threshold_arg = DeclareLaunchArgument(
        'threshold',
        default_value='80',
        description='Seuil de binarisation (0–255)'
    )
    roi_top_arg = DeclareLaunchArgument(
        'roi_top_fraction',
        default_value='0.45',
        description='Début ROI depuis le haut (0.45 = utiliser 55 % bas du frame)'
    )

    # ── Variable d'environnement GPU (TensorRT sur Jetson) ────────────────────
    ld_lib_path = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        CUSPARSELT_LIB + ':' + os.environ.get('LD_LIBRARY_PATH', '')
    )

    # ── 1. encoder_node — propriétaire GPIO ───────────────────────────────────
    encoder_node = Node(
        package='navigation',
        executable='encoder_node',
        name='encoder_node',
        output='screen'
    )

    # ── 2. camera_node ────────────────────────────────────────────────────────
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

    # ── 3. yolo_detection_node — GPU TensorRT ─────────────────────────────────
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

    # ── 4. line_follower_node — OpenCV CPU ────────────────────────────────────
    line_follower_node = Node(
        package='navigation',
        executable='line_follower_node',
        name='line_follower_node',
        parameters=[{
            'line_dark':               LaunchConfiguration('line_dark'),
            'threshold':               LaunchConfiguration('threshold'),
            'roi_top_fraction':        LaunchConfiguration('roi_top_fraction'),
            'min_line_area':           800,
            'intersection_width_ratio': 0.65,
        }],
        output='screen'
    )

    # ── 5. navigation_line_node — machine à états ─────────────────────────────
    navigation_line_node = Node(
        package='navigation',
        executable='navigation_line_node',
        name='navigation_line_node',
        output='screen'
    )

    return LaunchDescription([
        ld_lib_path,
        camera_device_arg,
        model_path_arg,
        device_arg,
        conf_arg,
        line_dark_arg,
        threshold_arg,
        roi_top_arg,
        encoder_node,
        camera_node,
        yolo_node,
        line_follower_node,
        navigation_line_node,
    ])
