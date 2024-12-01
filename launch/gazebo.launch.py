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
                    'xacro ', '/home/noi/serial_motor/scr/serial_motor_demo/description/urdf/robot.urdf.xacro'
                ])
            }]
        ),
    ])
