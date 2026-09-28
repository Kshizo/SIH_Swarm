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
                # person_detector subscribes to /camera/depth_image
                (f'{drone_prefix}/camera/depth_image', '/camera/depth_image'),
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
                # The bridge already publishes the lidar on /scan for SLAM, so the
                # view-coverage layer reads that instead of a separate raw topic.
                'coverage_scan_topic': '/scan',

                # The maze is 26 m across; the old 15 m cap put two of the four
                # survivors out of reach of either drone and starved the frontier
                # list into an early return.
                'exploration_radius_m': 40.0,
                'max_speed_mps': 0.6,

                # A moving multirotor sits well past 10 degrees, which kept the
                # view-coverage layer suspended for most of a flight.
                'coverage_max_tilt_deg': 25.0,
                'coverage_scan_timeout_s': 3.0,

                # Split the building in half about its centre; each drone claims
                # the wedge it launched into. No message passes between them.
                #
                # Each drone's map frame is anchored at its own take-off point,
                # so the shared reference has to be handed to it. Both drones
                # start 9.6 m along each axis from the maze centre, and drone 2
                # enters facing the other way, so in each drone's own map frame
                # the centre sits at (9.6, 9.6) - the launch heading is what
                # tells them apart.
                'sector_count': 2,
                'sector_index': -1,
                'sector_origin_x': 9.6,
                'sector_origin_y': 9.6,
                'sector_yaw_offset_deg': 0.0 if domain_id == '1' else 180.0
            }],
            # The node re-publishes pitch-gated scans; keep them off /scan so they
            # do not duplicate what the bridge is already feeding SLAM.
            remappings=[('scan', 'scan_gated')]
        ),

        # Hazard Mapper - reads the same map the explorer uses
        Node(
            package='drone_control',
            executable='hazard_mapper',
            name='hazard_mapper',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                # Corridors here are 2.4 m (1.2 m clearance); debris leaves
                # 1.8 m (0.9 m), so anything under 1.0 m clearance is debris.
                'min_clearance_m': 1.0
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
