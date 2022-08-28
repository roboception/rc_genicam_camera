import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('rc_genicam_camera'),
        'config',
        'rc_genicam_camera_params.yaml'
        )

    return LaunchDescription([
        Node(
            package='rc_genicam_camera',
            # namespace='rc_genicam',
            executable='rc_genicam_camera_node',
            name='rc_genicam_camera_node',
            # remappings=[
            #     ('/image', '/image/raw'),
            # ]
            parameters = [config]
        )
    ])