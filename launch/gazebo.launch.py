from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command

def generate_launch_description():
    return LaunchDescription([
        # Gazebo starten mit "gz sim"
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),
        Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-topic', '/robot_description',  # Beschreibung des Roboters
            '-name', 'my_robot'  # Name des Roboters in Gazebo
            ],
            output='screen',
        ),
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ', '/home/noi/batmobil/scr/serial_motor_demo/description/urdf/robot.urdf.xacro'
                ])
            }]
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_broad'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont'],
            output='screen',
        ),
    ])
