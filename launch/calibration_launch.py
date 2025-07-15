from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='foxglove_lidar_calib',
            executable='calibrator_node',
            name='calibrator_node',
            parameters=[
                {"camera_matrix": [306.01, 0.0, 317.20, 0.0, 306.00, 183.83, 0.0, 0.0, 1.0]},
                {"roll": 0.0},
                {"pitch": 0.0},
                {"yaw": 0.0}
            ],
            output='screen'
        )
    ])
