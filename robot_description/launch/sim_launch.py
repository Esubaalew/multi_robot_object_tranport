from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    # Replace 'robot_description' with your actual package name if different
    example_pkg_path = FindPackageShare('robot_description')
    gz_launch_path = PathJoinSubstitution(
        [ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    # Paths to your SDF files
    car_sdf = PathJoinSubstitution([example_pkg_path, 'sdf', 'car_robot.sdf'])
    manipulator_sdf = PathJoinSubstitution(
        [example_pkg_path, 'sdf', 'excavator_model.sdf'])
    
    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([example_pkg_path, 'models'])
        ),
        SetEnvironmentVariable(
            'GZ_SIM_PLUGIN_PATH',
            PathJoinSubstitution([example_pkg_path, 'plugins'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': PathJoinSubstitution([example_pkg_path, 'worlds', 'multi_robot_world.world']),
                'on_exit_shutdown': 'True'
            }.items(),
        ),

        # Bridging and remapping Gazebo topics to ROS 2 (replace with your own topics)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/example_imu_topic@sensor_msgs/msg/Imu@gz.msgs.IMU'],
            remappings=[('/example_imu_topic', '/remapped_imu_topic')],
            output='screen'
        ),

        # Spawn the car robot at a specific location
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'car_robot',
                '-file', car_sdf,
                '-x', '-2', '-y', '0', '-z', '0.0',
                '-Y', '0.0'  
            ],
            output='screen'
        ),

        # Spawn the manipulator robot at a specific location
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'manipulator_robot',
                '-file', manipulator_sdf,
                '-x', '-5', '-y', '0', '-z', '0.0',
                # Facing towards the positive Y-axis (approximately 90 degrees)
                '-Y', '1.57'
            ],
            output='screen'
        ),
    ])
