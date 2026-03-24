import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
def generate_launch_description():
    pkg = get_package_share_directory('robot_description')
    urdf_file = os.path.join(pkg, 'urdf', 'drift_asg.urdf')
    with open(urdf_file, 'r') as urdf:
        robot_desc = urdf.read()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2'
            #arguments=['-d', os.path.join(pkg, 'rviz', 'drift_asg.rviz')]
        )
    ])