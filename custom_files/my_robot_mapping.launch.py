# Custom RTAB-Map launch file for your ASCamera HP60C + LD LiDAR setup
#
# Usage:
#   SLAM mode:
#     $ ros2 launch rtabmap_demos my_robot_mapping.launch.py rviz:=true rtabmap_viz:=true
#
#   Localization mode (after creating a map):
#     $ ros2 launch rtabmap_demos my_robot_mapping.launch.py localization:=true rviz:=true
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    localization = LaunchConfiguration('localization')

    # RTAB-Map parameters optimized for your hardware
    parameters={
          # Frame configuration - you'll need to verify these
          'frame_id':'ldlidar_base',  # Change this to your robot's base frame
          'odom_frame_id':'odom',     # You may need to create this frame
          'odom_tf_linear_variance':0.001,
          'odom_tf_angular_variance':0.001,
          
          # Sensor subscriptions
          'subscribe_rgbd':True,      # Use your RGB-D camera
          'subscribe_scan':True,      # Use your LiDAR
          'approx_sync':True,         # Allow approximate time sync
          'sync_queue_size': 10,
          
          # RTAB-Map algorithm parameters (optimized for real-time)
          'RGBD/NeighborLinkRefining': 'true',    # Improve odometry with laser
          'RGBD/ProximityBySpace':     'true',    # Spatial loop closure
          'RGBD/ProximityByTime':      'false',   # Disable time-based loops
          'RGBD/ProximityPathMaxNeighbors': '10', 
          'Reg/Strategy':              '1',       # ICP registration
          'Vis/MinInliers':            '15',      # Visual loop closure threshold
          'RGBD/OptimizeFromGraphEnd': 'false',   # Generate /map -> /odom transform
          'RGBD/OptimizeMaxError':     '3',       # Reject bad loop closures
          'Reg/Force3DoF':             'true',    # 2D SLAM for your setup
          'Grid/FromDepth':            'false',   # Use laser for occupancy grid
          'Mem/STMSize':               '30',      # Short-term memory size
          'RGBD/LocalRadius':          '5',       # Local proximity radius
          
          # ICP parameters tuned for your LiDAR
          'Icp/CorrespondenceRatio':   '0.3',     # Scan overlap threshold
          'Icp/PM':                    'false',   # Don't use libpointmatcher
          'Icp/PointToPlane':          'false',   # Point-to-point ICP
          'Icp/MaxCorrespondenceDistance': '0.1', # Max correspondence distance
          'Icp/VoxelSize':             '0.02',    # Voxel grid for downsampling
          
          # Memory and processing optimization
          'Rtabmap/TimeThr':           '0',       # No time limit for processing
          'Rtabmap/DetectionRate':     '1',       # Process every frame
          'Mem/RehearsalSimilarity':   '0.6',     # Memory management
          'RGBD/AngularUpdate':        '0.1',     # Minimum angular change (rad)
          'RGBD/LinearUpdate':         '0.1',     # Minimum linear change (m)
    }
    
    # Topic remappings for your specific hardware
    remappings=[
         ('rgb/image',       '/ascamera_hp60c/camera_publisher/rgb0/image'),
         ('depth/image',     '/ascamera_hp60c/camera_publisher/depth0/image_raw'),
         ('rgb/camera_info', '/ascamera_hp60c/camera_publisher/rgb0/camera_info'),
         ('depth/camera_info', '/ascamera_hp60c/camera_publisher/depth0/camera_info'),
         ('scan',            '/ldlidar_node/scan')]
    
    # RViz configuration path
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
    )

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument('rtabmap_viz',  default_value='true',  description='Launch RTAB-Map UI.'),
        DeclareLaunchArgument('rviz',         default_value='true',  description='Launch RViz.'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('rviz_cfg',     default_value=config_rviz, description='RViz config file.'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db', description='Database path.'),

        # Use real-time (not simulation time)
        SetParameter(name='use_sim_time', value=False),

        # Visual odometry node (using RTAB-Map's own odometry)
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[{
                'frame_id': 'ldlidar_base',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'guess_frame_id': '',
                'wait_imu_to_init': False,
                # Visual odometry parameters
                'Odom/Strategy': '0',  # Frame-to-Map = 0, Frame-to-Frame = 1
                'Vis/EstimationType': '1',  # 0=3D->3D, 1=3D->2D (PnP)
                'Vis/MaxDepth': '4.0',
                'Vis/MinInliers': '15',
                'Odom/ResetCountdown': '15',
                'Vis/InlierDistance': '0.1',
            }],
            remappings=[
                ('rgb/image', '/ascamera_hp60c/camera_publisher/rgb0/image'),
                ('depth/image', '/ascamera_hp60c/camera_publisher/depth0/image_raw'),
                ('rgb/camera_info', '/ascamera_hp60c/camera_publisher/rgb0/camera_info')
            ]),

        # RGB-D synchronization node
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[parameters,
              {'rgb_image_transport':'raw',          # Your camera likely uses raw transport
               'depth_image_transport':'raw',        # Your camera likely uses raw transport
               'approx_sync_max_interval': 0.05}],  # 50ms sync window
            remappings=remappings),
        
        # SLAM mode node
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']), # Delete previous database for fresh start
            
        # Localization mode node
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',      # Don't add new data to map
               'Mem/InitWMWithAllNodes':'True'}],    # Load entire existing map
            remappings=remappings),

        # RTAB-Map visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[parameters],
            remappings=remappings),
            
        # RViz visualization  
        Node(
            package='rviz2', executable='rviz2', name="rviz2", output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]]),
    ])
