import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_drone_mapping = get_package_share_directory('drone_mapping')
    urdf_file = os.path.join(pkg_drone_mapping, 'urdf', 'drone.urdf')

    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # RF2O Laser Odometry
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom',
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 20.0,
                'use_sim_time': True
            }]
        ),

        # Robot State Publisher for URDF
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

        # ROS-Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/downward_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/downward_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/world/maze_survivors/model/iris_with_sensors/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
            ],
            remappings=[
                ('/lidar/scan', '/scan_raw'),
                ('/camera/image', '/camera/image_raw'),
                ('/camera/camera_info', '/camera_info'),
                ('/world/maze_survivors/model/iris_with_sensors/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu', '/imu')
            ],
            output='screen'
        ),

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'slam_params_file': os.path.join(get_package_share_directory('drone_mapping'), 'config', 'mapper_params_online_async.yaml'),
                'use_sim_time': 'true'
            }.items()
        ),

        # Optical Flow Node
        Node(
            package='drone_mapping',
            executable='optical_flow',
            name='optical_flow',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # EKF Node for Odometry Fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(pkg_drone_mapping, 'config', 'ekf.yaml'),
                {'use_sim_time': True}
            ]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),

        # VIO to ArduPilot Feedback Loop
        Node(
            package='drone_mapping',
            executable='vio_to_ardupilot',
            name='vio_to_ardupilot',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
