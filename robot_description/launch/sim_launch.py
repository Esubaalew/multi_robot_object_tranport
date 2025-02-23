from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    example_pkg_path = FindPackageShare('robot_description')
    gz_launch_path = PathJoinSubstitution(
        [ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    truck_sdf = PathJoinSubstitution(
        [example_pkg_path, 'sdf', 'car_robot.sdf'])
    excavator_sdf = PathJoinSubstitution(
        [example_pkg_path, 'sdf', 'excavator.sdf'])

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', PathJoinSubstitution(
            [example_pkg_path, 'models'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': PathJoinSubstitution([example_pkg_path, 'worlds', 'multi_robot_world.world']),
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/car_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/excavator/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/excavator/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/excavator/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
            ],
            output='screen',
            parameters=[{'verbose': True,
                         'qos_overrides': {
                             '/car_robot/cmd_vel': {'reliability': 'reliable'},
                             '/excavator/cmd_vel': {'reliability': 'reliable'}
                         }}]
        ),
        # Position the car robot at (-2, -2) facing positive x (yaw=0)
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'car_robot', '-file', truck_sdf,
                       '-x', '-2', '-y', '-2', '-z', '0.0', '-Y', '0.0'],
            output='screen'
        ),
        # Position the excavator at (-2, 0) facing positive x (yaw=0)
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'excavator', '-file', excavator_sdf,
                       '-x', '-2', '-y', '0', '-z', '0.0', '-Y', '0.0'],
            output='screen'
        ),
        Node(
            package='robot_description',
            executable='object_detector',
            output='screen'
        )
    ])
