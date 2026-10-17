from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bridge_imu',             # 패키지 이름
            executable='imu_bridge',          # setup.py의 entry_points에서 지정한 실행 이름
            name='imu_bridge_node',           # ROS 그래프 상에서의 노드 이름 (원하면 수정 가능)
            output='screen'                   # 출력 콘솔에 노드 로그 표시
        )
    ])

