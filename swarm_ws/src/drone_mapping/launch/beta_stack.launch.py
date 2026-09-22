import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_drone_mapping = get_package_share_directory('drone_mapping')
    urdf_file = os.path.join(pkg_drone_mapping, 'urdf', 'drone.urdf')

    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Get Domain ID to figure out which Gazebo topics to bridge
    domain_id = os.environ.get('ROS_DOMAIN_ID', '1')
    drone_prefix = '/drone1' if domain_id == '1' else '/drone2'

    return LaunchDescription([
        # ROS-Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                f'{drone_prefix}/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'{drone_prefix}/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'{drone_prefix}/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'{drone_prefix}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            ],
            remappings=[
                (f'{drone_prefix}/lidar/scan', '/scan'),
                (f'{drone_prefix}/camera/image', '/camera/image_raw'),
                (f'{drone_prefix}/camera/camera_info', '/camera/camera_info')
            ],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True
            }]
        ),

        # RF2O Laser Odometry
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom',
                'publish_tf': True, # Needs to publish TF in beta_v2 since EKF is removed
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 20.0,
                'use_sim_time': True
            }]
        ),

        # VIO to Ardupilot
        Node(
            package='drone_mapping',
            executable='vio_to_ardupilot',
            name='vio_to_ardupilot',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'sysid': int(domain_id),
                'initial_yaw': 0.0 if domain_id == '1' else 180.0
            }]
        ),

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'slam_params_file': os.path.join(pkg_drone_mapping, 'config', 'mapper_params_online_async.yaml'),
                'use_sim_time': 'true'
            }.items()
        ),

        # Frontier Exploration
        Node(
            package='drone_control',
            executable='frontier_exploration',
            name='frontier_exploration',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'target_survivor_count': 4
            }]
        ),

        # Person Detector
        Node(
            package='ssd_lite_ros',
            executable='person_detector',
            name='person_detector',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
