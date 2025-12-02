import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_controller = get_package_share_directory('yahboomcar_mecanum_controller')

    xacro_file = os.path.join(pkg_controller, 'urdf', 'yahboomcar_mecanum.xacro')
    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        # 启动 Gazebo（会自动加载 ros2_control 插件）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            )
        ),

        # 发布 robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # 在 Gazebo 中生成机器人（会触发插件加载）
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'mecanum_yahboomcar', '-topic', '/robot_description'],
            output='screen'
        ),

        # 等待 controller_manager 启动后加载控制器
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['mecanum_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
    ])