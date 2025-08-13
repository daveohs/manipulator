from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # LaunchConfiguration을 사용하여 시뮬레이션 시간 사용 여부 결정
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # XACRO 파일 경로 설정
    xacro_file_name = 'manipulator.urdf.xacro'
    xacro_file_path = os.path.join(
        get_package_share_directory('manipulator'),
        'urdf',
        xacro_file_name)
    
    # xacro 명령어를 사용하여 URDF 생성
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file_path,
        ]
    )
    robot_description = {"robot_description": robot_description_content}


    # RViz 설정 파일 경로
    rviz_config_file = os.path.join(
        get_package_share_directory('manipulator'), 
        'rviz', 
        'display.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        # robot_state_publisher 노드: URDF를 기반으로 로봇의 상태를 TF로 발행
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {"use_sim_time": use_sim_time}]),
        
        # joint_state_publisher_gui 노드: GUI를 통해 조인트 상태를 조절
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'),
        
        # RViz2 노드
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file])
    ])
