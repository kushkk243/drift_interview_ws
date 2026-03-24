import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
def generate_launch_description():
    pkg = get_package_share_directory('robot_description')
    urdf_file = os.path.join(pkg, 'urdf', 'drift_asg.urdf')
    #with open(urdf_file, 'r') as urdf:
    #    robot_desc = urdf.read()
    world_file = os.path.join(pkg, "worlds", "apartment_des.sdf")
    robot_desc = ParameterValue(Command(['xacro ',urdf_file]), value_type=str)
    print(os.path.dirname(pkg))
    gz_resources = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH ', value=os.path.dirname(pkg))
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(),
    )

    rob_desc = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name','drift_asg_cleaner',
            '-topic','robot_description',
            '-z','0.1'
        ]
    )
    cam = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments= ['/main_cam@sensor_msgs/msg/Image@ignition.msgs.Image']
    )
    imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments= ['/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU']
    )
    lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments= ['/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan']
    )
    depthim = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments= ['/depthcam@sensor_msgs/msg/Image@ignition.msgs.Image']
    )
    depthcl = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments= ['/depthcam/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked']
    )
    odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments= ['/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry']
    )
    cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments= ['/cmd@geometry_msgs/msg/Twist@ignition.msgs.Twist']
    )
    tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments= ['/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V']
    )
    ### Actuators

    load_joint_broadcast = ExecuteProcess(cmd=['ros2','control','load_controller','--set-state','active','joint_state_broadcaster'],output='screen')
    left_arm_control = ExecuteProcess(cmd=['ros2','control','load_controller','--set-state','active','arm_left_controller'],output='screen')
    right_arm_control = ExecuteProcess(cmd=['ros2','control','load_controller','--set-state','active','arm_right_controller'],output='screen')
    right_gripper_control = ExecuteProcess(cmd=['ros2','control','load_controller','--set-state','active','gripper_right_controller'],output='screen')
    left_gripper_control = ExecuteProcess(cmd=['ros2','control','load_controller','--set-state','active','gripper_left_controller'],output='screen')

    return LaunchDescription([
        gz_resources,
        gz,
        rob_desc,
        spawn,
        load_joint_broadcast,
        left_arm_control,
        right_arm_control,
        right_gripper_control,
        left_gripper_control,
        cam,
        imu,
        lidar,
        depthim,
        depthcl,
        odom,
        cmd_vel,
        tf
    ])