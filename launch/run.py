import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_prototyping_project2 = get_package_share_directory('prototyping-project2')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'simple_mobile.urdf'
    urdf = os.path.join(pkg_prototyping_project2, 'urdf', urdf_file_name)

    world_file_name = 'test_world.world'  # 사용할 월드 파일 이름
    world_path = os.path.join(pkg_prototyping_project2, 'description', world_file_name)

    rviz_config_dir = os.path.join(
        pkg_prototyping_project2,
        'rviz',
        'view_simple_mobile.rviz')
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}
    
    xpose = LaunchConfiguration("x_pose", default="-6.4")
    ypose = LaunchConfiguration("y_pose", default="-2.1")

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Set x position'),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Set y position'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world_path}.items()  # 월드 파일 경로 추가
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}],
            arguments=[urdf]),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),
        TimerAction(
            period=20.0,  # Wait for 5 seconds before proceeding
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_entity',
                    arguments=[
                        '-entity', "simple_mobile",
                        "-file", urdf,
                        "-x", "-6.4",
                        "-y", "-2.1",
                        "-z", "1",
                        "-package_to_model"
                    ],
                    output='screen'
                )
            ]
        ),
        
        Node(
            package='prototyping-project2',
            executable='PotentialFieldVisualizer.py',
            name='PotentialFieldVisualizer',
            output='screen',
            parameters=[],
        ),
        Node(
            package='prototyping-project2',
            executable='MakeEnvNode.py',
            name='MakeEnvNode',
            output='screen',
            parameters=[],
        ),
        Node(
            package='prototyping-project2',
            executable='SpawnObjectNode.py',
            name='SpawnObjectNode',
            output='screen',
            parameters=[],
        )
    ])
