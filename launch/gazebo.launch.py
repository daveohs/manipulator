import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get the package paths
    pkg_manipulator_path = FindPackageShare('manipulator')
    pkg_gazebo_ros_path = FindPackageShare('gazebo_ros')

    # --- FIX: Gazebo가 모델 파일을 찾을 수 있도록 경로를 환경 변수에 추가 ---
    # manipulator 패키지의 share 디렉토리 경로를 올바르게 설정합니다.
    manipulator_share_path = get_package_share_directory('manipulator')
    gazebo_model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(os.path.dirname(manipulator_share_path), '') # Get parent directory of package's share
    )

    # XACRO 파일 경로 설정
    xacro_file = PathJoinSubstitution(
        [pkg_manipulator_path, "urdf", "manipulator.urdf.xacro"]
    )
    
    # xacro 명령어를 사용하여 URDF 생성
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_gazebo_ros_path, 'launch', 'gazebo.launch.py']
            )
        )
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'manipulator'],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Joint trajectory controller spawner
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(declared_arguments + [
        gazebo_model_path, # 추가된 환경 변수 설정을 리스트 맨 앞에 포함
        gazebo,
        spawn_entity,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
    ])
