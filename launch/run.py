from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='prototyping-project2',  # 여기에 패키지 이름을 입력하세요
            executable='PotentialFieldVisualizer.py',  # 실행 파일 이름 (setup.py에 등록된 이름과 동일해야 함)
            name='PotentialFieldVisualizer',  # 노드 이름
            output='screen',  # 로그 출력 옵션
            parameters=[
                # 여기에 필요하다면 노드 파라미터를 추가할 수 있습니다.
            ],
        ),
    ])
