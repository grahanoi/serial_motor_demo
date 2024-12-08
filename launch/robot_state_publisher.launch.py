from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Path to the URDF/Xacro file
    urdf_file_path = '/home/noi/batmobil/scr/serial_motor_demo/description/urdf/update.urdf.xacro'

    return LaunchDescription([
        # Node for robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file_path])
            }]
        ),

        # Node for RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/noi/batmobil/scr/serial_motor_demo/rviz/robot.rviz']  # Path to your RViz config file
        ),

        # Node for Static Transform Publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'odom']  # Update these values as needed
        ),
        # Node for MotorGui
        Node(
            package='serial_motor_demo',
            executable='gui',
            name='motor_gui',
            output='screen'
        ),
    ])
