from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': False},  # 시뮬레이션 시간을 사용하지 않을 경우 False로 설정
                {'slam_toolbox_params': '/path/to/slam_params.yaml'}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/path/to/slam_config.rviz']
        )
    ])
