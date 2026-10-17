from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([

        # 1. PX4 SITL (즉시 실행)
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'cd ~/PX4-Autopilot && make px4_sitl gazebo-classic_iris_depth_camera__smartfactory'
            ],
            output='screen'
        ),

        # 2. QGroundControl (즉시 실행)
        ExecuteProcess(
            cmd=['bash', '-c', './QGroundControl.AppImage'],
            output='screen'
        ),

        # 3. Micro XRCE Agent (즉시 실행)
        ExecuteProcess(
            cmd=['bash', '-c', 'MicroXRCEAgent udp4 -p 8888'],
            output='screen'
        ),

        # 4. rtabmap interface (즉시 실행)
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'cd ~/workspace/ws_rtabmap && source ~/workspace/ws_rtabmap/install/setup.bash && ros2 launch rtabmap_examples gazebo-classic_interface.launch.py'
            ],
            output='screen'
        ),
        
        
        # 5. canny edge detection (즉시 실행)
        ExecuteProcess(
            cmd=['bash', '-c', 'cd ~/offboard_control/add_offboard_control && source install/setup.bash && ros2 run cannyedge_test1 canny_edge_detector'],
            output='screen'
        ),

        # 6. rtabmap1 (1초 후)
        TimerAction(
            period=0.5,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash', '-c',
                        'cd ~/workspace/ws_rtabmap && source ~/workspace/ws_rtabmap/install/setup.bash && ros2 launch rtabmap_examples gazebo-classic_rtabmap1.launch.py'
                    ],
                    output='screen'
                )
            ]
        ),

        # 7. RViz2 (2초 후)
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', 'rviz2'],
                    output='screen'
                )
            ]
        ),

        # 8. filtering_map (3초 후)
        TimerAction(
            period=1.5,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash', '-c',
                        'cd ~/workspace/ws_voxel/voxel_mapper && source install/setup.bash && ros2 run voxel_mapper filtering_map_node'
                    ],
                    output='screen'
                )
            ]
        ),
    ])
