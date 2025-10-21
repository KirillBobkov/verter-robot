#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch файл для запуска TOF камеры.
    """
    
    # Аргументы запуска
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='0',
        description='Номер устройства камеры (обычно 0 для /dev/video0)'
    )
        
    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='640',
        description='Ширина изображения камеры'
    )
    
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='480',
        description='Высота изображения камеры'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Частота кадров'
    )
    
    use_gst_arg = DeclareLaunchArgument(
        'use_gstreamer',
        default_value='true',
        description='Использовать GStreamer/libcamera для CSI камеры'
    )
    
    gst_pipeline_arg = DeclareLaunchArgument(
        'gst_pipeline',
        default_value='',
        description='Пользовательский GStreamer pipeline (если пусто — сгенерируется)'
    )
    
    depth_min_arg = DeclareLaunchArgument(
        'depth_min',
        default_value='0.1',
        description='Минимальная глубина в метрах'
    )
    
    depth_max_arg = DeclareLaunchArgument(
        'depth_max',
        default_value='5.0',
        description='Максимальная глубина в метрах'
    )
    
    return LaunchDescription([
        camera_device_arg,
        camera_width_arg,
        camera_height_arg,
        fps_arg,
        depth_min_arg,
        depth_max_arg,
        use_gst_arg,
        gst_pipeline_arg,
        
        # Узел TOF камеры
        Node(
            package='verter_admin',
            executable='tof_camera_node',
            name='tof_camera_node',
            output='screen',
            parameters=[{
                'camera_device': LaunchConfiguration('camera_device'),
                'camera_width': LaunchConfiguration('camera_width'),
                'camera_height': LaunchConfiguration('camera_height'),
                'fps': LaunchConfiguration('fps'),
                'depth_min': LaunchConfiguration('depth_min'),
                'depth_max': LaunchConfiguration('depth_max'),
                'use_gstreamer': LaunchConfiguration('use_gstreamer'),
                'gst_pipeline': LaunchConfiguration('gst_pipeline'),
            }],
            remappings=[
                ('tof_camera/depth_image', 'camera/depth_image'),
                ('tof_camera/point_cloud', 'camera/point_cloud'),
                ('tof_camera/stats', 'camera/stats'),
            ]
        ),
    ])
