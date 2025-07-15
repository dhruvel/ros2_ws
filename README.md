# RTABMap Integration with ASCAM HP60C RGBD Camera & LDROBOT D500 LiDAR Kit

A complete guide for integrating the ASCAM HP60C RGBD camera with RTABMap for real-time SLAM (Simultaneous Localization and Mapping) in ROS 2.

## 📋 Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Camera Integration Setup](#camera-integration-setup)
- [RTABMap Configuration](#rtabmap-configuration)
- [Launch Files](#launch-files)
- [Usage](#usage)
- [Visualization](#visualization)
- [Troubleshooting](#troubleshooting)
- [Performance Tuning](#performance-tuning)
- [File Structure](#file-structure)

## 🎯 Overview

This integration allows the ASCAM HP60C RGBD camera to work seamlessly with RTABMap for real-time SLAM applications. The setup includes:

- Custom RTABMap launch configuration
- Optimized SLAM parameters
- Real-time visualization
- Automated setup scripts
- Comprehensive troubleshooting tools

### System Requirements

- **ROS 2 Distribution**: Jazzy (or compatible)
- **Camera**: ASCAM HP60C with working ROS 2 driver
- **RTABMap**: Full installation with visualization components
- **Hardware**: Sufficient CPU for real-time processing

## 📦 Prerequisites

### 1. ROS 2 Installation

```bash
# Install ROS 2 Jazzy (Ubuntu 24.04)
sudo apt update && sudo apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-desktop -y
```

### 2. ASCAM Camera Driver

Ensure your ASCAM HP60C camera driver is installed and working:
```bash
# Example workspace structure
~/ros2_ws/
└── src/
    └── ascamera/
```

### 3. LDLiDAR Driver

Ensure your LDROBOT D500 LiDAR driver is installed and working:
```bash
# Example workspace structure
~/ros2_ws/
└── src/
    └── ldrobot-liadr-ros2/
```


## 🔧 Installation

### Step 1: Clone RTABMap ROS 2 Workspace

```bash
# Create RTABMap workspace
mkdir -p ~/rtabmap_ros2_ws/src
cd ~/rtabmap_ros2_ws

Cleanup binaries
sudo apt remove ros-$ROS_DISTRO-rtabmap* -y

# RTABMap ROS packages
git clone https://github.com/introlab/rtabmap.git src/rtabmap
git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
```

### Step 2: Import Custom Files

```bash
# Clone custom files for integration
cd ~
git clone https://github.com/dhruvel/rtabmap_ros2_ws.git dhruvel_ros2_ws
mkdir ~/rtabmap_ros2_ws/src/rtabmap_ros/rtabmap_demos/scripts
mv dhruvel_ros2_ws/custom_files/my_robot_mapping.launch.py ~/rtabmap_ros2_ws/src/rtabmap_ros/rtabmap_demos/launch && mv dhruvel_ros2_ws/custom_files/test_sensors.py ~/rtabmap_ros2_ws/src/rtabmap_ros/rtabmap_demos/scripts
# Cleanup temp directory
rm -fr ~/dhruvel_ros2_ws
```


### Step 3: Build Workspace

```bash
cd ~/rtabmap_ros2_ws
rosdep update && rosdep install --from-paths src --ignore-src -r -y
export MAKEFLAGS="-j6" # Can be ignored if you have a lot of RAM (>16GB)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Step 3: Create Integration Files

Create the following files in your RTABMap workspace:

## 🎬 Camera Integration Setup

### 1. RTABMap Launch File

Create: `~/rtabmap_ros2_ws/src/rtabmap_ros/rtabmap_examples/launch/ascam_hp60c.launch.py`

```python
# Requirements:
#   ASCAM HP60C camera running with ascamera ROS2 package
#   Make sure the ascamera node is running before launching this file
# Example:
#   Terminal 1: $ cd ~/ascam_ros2_ws && source install/setup.bash && ros2 launch ascamera hp60c.launch.py
#   Terminal 2: $ cd ~/rtabmap_ros2_ws && source install/setup.bash && ros2 launch rtabmap_examples ascam_hp60c.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    
    # Get the path to the configuration file
    config_file = os.path.join(
        get_package_share_directory('rtabmap_examples'),
        'config',
        'ascam_hp60c.yaml'
    )
    
    # Parameters for RTABMap nodes
    parameters=[{
          'frame_id':'ascamera_hp60c_color_0',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':False,
          'wait_for_transform':0.2,
          'use_sim_time':False}]

    # Topic remappings to match ASCAM HP60C output
    remappings=[
          ('rgb/image', '/ascamera_hp60c/camera_publisher/rgb0/image'),
          ('rgb/camera_info', '/ascamera_hp60c/camera_publisher/rgb0/camera_info'),
          ('depth/image', '/ascamera_hp60c/camera_publisher/depth0/image_raw'),
          ('depth/camera_info', '/ascamera_hp60c/camera_publisher/depth0/camera_info')]

    return LaunchDescription([
        
        # Declare launch arguments
        DeclareLaunchArgument(
            'rtabmap_args',
            default_value='',
            description='Additional arguments for rtabmap node'
        ),
        
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RTABMap visualization'
        ),

        # RGB-D Odometry node
        Node(
            package='rtabmap_odom', 
            executable='rgbd_odometry', 
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['--ros-args', '--log-level', 'info']),

        # RTABMap SLAM node
        Node(
            package='rtabmap_slam', 
            executable='rtabmap', 
            output='screen',
            parameters=[parameters[0], config_file],
            remappings=remappings,
            arguments=['-d', LaunchConfiguration('rtabmap_args')]),

        # RTABMap Visualization (optional)
        Node(
            package='rtabmap_viz', 
            executable='rtabmap_viz', 
            output='screen',
            parameters=parameters,
            remappings=remappings,
            condition=IfCondition(LaunchConfiguration('launch_rviz'))),
    ])
```

### 2. RTABMap Configuration File

Create: `~/rtabmap_ros2_ws/src/rtabmap_ros/rtabmap_examples/config/ascam_hp60c.yaml`

```yaml
# RTABMap configuration parameters optimized for ASCAM HP60C
# These parameters can be loaded using the 'load_parameters_from_file' launch argument

rtabmap:
  ros__parameters:
    # Database and memory management
    Mem/RehearsalSimilarity: "0.6"
    Mem/BadSignaturesIgnored: "false"
    Mem/MapLabelsAdded: "true"
    Mem/UseOdomGravity: "false"
    
    # RGB-D specific parameters
    RGBD/ProximityBySpace: "true"
    RGBD/ProximityByTime: "false"
    RGBD/ProximityPathMaxNeighbors: "10"
    RGBD/AngularUpdate: "0.1"
    RGBD/LinearUpdate: "0.1"
    
    # Visual odometry optimization
    Vis/EstimationType: "1"  # 0=3D->3D, 1=3D->2D (PnP)
    Vis/MaxDepth: "4.0"     # Adjust based on your camera's effective range
    Vis/MinInliers: "20"
    Vis/InlierDistance: "0.02"
    
    # Loop closure detection
    Kp/MaxDepth: "4.0"
    Kp/MaxFeatures: "400"
    Kp/DetectorStrategy: "0"  # SURF
    
    # Graph optimization
    Optimizer/Strategy: "0"   # TORO
    Optimizer/Iterations: "20"
    
    # Grid mapping
    Grid/FromDepth: "true"
    Grid/MaxObstacleHeight: "2.0"
    Grid/MaxGroundHeight: "0.0"
```

### 3. Convenience Launcher Script

Create: `~/rtabmap_ascam_launcher.sh`

```bash
#!/bin/bash

# RTABMap ASCAM HP60C Integration Script
# This script helps launch both the ASCAM camera and RTABMap SLAM

set -e

echo "=== RTABMap ASCAM HP60C Integration Script ==="
echo ""

# Function to check if a ROS package is available
check_package() {
    local package_name=$1
    local workspace_path=$2
    
    if ! ros2 pkg list | grep -q "^${package_name}$"; then
        echo "ERROR: Package '${package_name}' not found!"
        echo "Make sure you've built the workspace at: ${workspace_path}"
        exit 1
    fi
}

# Function to launch camera
launch_camera() {
    echo "Launching ASCAM HP60C camera..."
    cd ~/ascam_ros2_ws
    source install/setup.bash
    check_package "ascamera" "~/ascam_ros2_ws"
    ros2 launch ascamera hp60c.launch.py &
    CAMERA_PID=$!
    echo "Camera launched with PID: $CAMERA_PID"
    sleep 3
}

# Function to launch RTABMap
launch_rtabmap() {
    echo "Launching RTABMap SLAM..."
    cd ~/rtabmap_ros2_ws
    source install/setup.bash
    check_package "rtabmap_examples" "~/rtabmap_ros2_ws"
    ros2 launch rtabmap_examples ascam_hp60c.launch.py
}

# Function to cleanup
cleanup() {
    echo ""
    echo "Cleaning up..."
    if [ ! -z "$CAMERA_PID" ]; then
        kill $CAMERA_PID 2>/dev/null || true
    fi
    pkill -f "ascamera_node" 2>/dev/null || true
    pkill -f "rtabmap" 2>/dev/null || true
    pkill -f "rgbd_odometry" 2>/dev/null || true
    pkill -f "rtabmap_viz" 2>/dev/null || true
    echo "Cleanup complete."
}

# Set up signal handlers
trap cleanup EXIT INT TERM

# Main execution
case "${1:-full}" in
    "camera")
        launch_camera
        echo "Camera is running. Press Ctrl+C to stop."
        wait $CAMERA_PID
        ;;
    "rtabmap")
        launch_rtabmap
        ;;
    "full"|"")
        launch_camera
        echo "Waiting for camera to initialize..."
        sleep 5
        
        # Check if camera topics are available
        echo "Checking camera topics..."
        if ! ros2 topic list | grep -q "ascamera_hp60c"; then
            echo "ERROR: Camera topics not found! Check camera launch."
            exit 1
        fi
        
        echo "Camera topics detected. Launching RTABMap..."
        launch_rtabmap
        ;;
    "topics")
        echo "Available camera topics:"
        cd ~/ascam_ros2_ws
        source install/setup.bash
        ros2 topic list | grep ascamera || echo "No ascamera topics found"
        ;;
    "help")
        echo "Usage: $0 [option]"
        echo ""
        echo "Options:"
        echo "  full     - Launch both camera and RTABMap (default)"
        echo "  camera   - Launch only the camera"
        echo "  rtabmap  - Launch only RTABMap (assumes camera is running)"
        echo "  topics   - List available camera topics"
        echo "  help     - Show this help message"
        ;;
    *)
        echo "Unknown option: $1"
        echo "Use '$0 help' for usage information."
        exit 1
        ;;
esac
```

### 4. Diagnostic Test Script

Create: `~/rtabmap_test.sh`

```bash
#!/bin/bash

# RTABMap ASCAM Integration Test Script
# This script helps diagnose issues with the camera-RTABMap integration

echo "=== RTABMap ASCAM Integration Test ==="
echo ""

# Source the workspaces
echo "Setting up environment..."
source /opt/ros/jazzy/setup.bash
cd ~/ascam_ros2_ws && source install/setup.bash
cd ~/rtabmap_ros2_ws && source install/setup.bash

echo "1. Checking if camera is running..."
if ros2 topic list | grep -q "ascamera_hp60c"; then
    echo "✓ Camera topics detected"
    
    echo "2. Camera topics:"
    ros2 topic list | grep ascamera_hp60c
    
    echo ""
    echo "3. Checking message rates..."
    echo "RGB image rate:"
    timeout 5 ros2 topic hz /ascamera_hp60c/camera_publisher/rgb0/image 2>/dev/null || echo "No messages received"
    
    echo "Depth image rate:"
    timeout 5 ros2 topic hz /ascamera_hp60c/camera_publisher/depth0/image_raw 2>/dev/null || echo "No messages received"
    
    echo ""
    echo "4. Checking frame IDs..."
    echo "RGB frame_id:"
    timeout 2 ros2 topic echo /ascamera_hp60c/camera_publisher/rgb0/camera_info --once 2>/dev/null | grep frame_id || echo "Could not get frame_id"
    
    echo ""
    echo "5. Checking camera intrinsics..."
    echo "RGB camera intrinsics (K matrix):"
    timeout 2 ros2 topic echo /ascamera_hp60c/camera_publisher/rgb0/camera_info --once 2>/dev/null | grep -A 9 "k:" || echo "Could not get intrinsics"
    
    echo ""
    echo "6. Testing RTABMap launch file..."
    echo "Launch file location: ~/rtabmap_ros2_ws/src/rtabmap_ros/rtabmap_examples/launch/ascam_hp60c.launch.py"
    if [ -f "~/rtabmap_ros2_ws/src/rtabmap_ros/rtabmap_examples/launch/ascam_hp60c.launch.py" ]; then
        echo "✓ Launch file exists"
    else
        echo "✗ Launch file missing"
    fi
    
    echo ""
    echo "7. Testing TF tree..."
    echo "Available transforms:"
    timeout 3 ros2 run tf2_tools view_frames.py 2>/dev/null || echo "TF tree check failed"
    
else
    echo "✗ Camera not running or topics not found"
    echo ""
    echo "Please start the camera first:"
    echo "cd ~/ascam_ros2_ws && source install/setup.bash && ros2 launch ascamera hp60c.launch.py"
fi

echo ""
echo "=== Test Complete ==="
echo ""
echo "To run RTABMap with your camera:"
echo "1. Start camera: cd ~/ascam_ros2_ws && source install/setup.bash && ros2 launch ascamera hp60c.launch.py"
echo "2. Start RTABMap: cd ~/rtabmap_ros2_ws && source install/setup.bash && ros2 launch rtabmap_examples ascam_hp60c.launch.py"
echo "3. Or use the convenience script: ~/rtabmap_ascam_launcher.sh"
```

### 5. Build the Integration

```bash
# Make scripts executable
chmod +x ~/rtabmap_ascam_launcher.sh
chmod +x ~/rtabmap_test.sh

# Build RTABMap with new files
cd ~/rtabmap_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rtabmap_examples --allow-overriding rtabmap_examples
```

## 🚀 Usage

### Manual Launch (Five Terminals)

**Terminal 1 - Camera:**
```bash
ros2 launch ascamera hp60c.launch.py
```
**Terminal 2 - LiDAR:**
```bash
ros2 launch ldlidar_node ldlidar_bringup.launch.py
```

**Terminal 3 - Configure LiDAR:**
```bash
ros2 lifecycle set /ldlidar_node configure
ros2 lifecycle set /ldlidar_node activate
```

**Terminal 4 - RTABMap:**
```bash
cd /home/dhruvel/rtabmap_ros2_ws && source install/setup.bash && ros2 launch rtabmap_demos my_robot_mapping.launch.py rviz:=true rtabmap_viz:=true
```

**Terminal 5 - Configure RTABMap:**
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 ldlidar_base ascamera_hp60c_camera_link_0 &
```

### Launch with Custom Parameters

```bash
# Delete previous database and start fresh
ros2 launch rtabmap_examples ascam_hp60c.launch.py rtabmap_args:="--delete_db_on_start"

# Launch without visualization
ros2 launch rtabmap_examples ascam_hp60c.launch.py launch_rviz:=false

# Enable debug logging
ros2 launch rtabmap_examples ascam_hp60c.launch.py rtabmap_args:="--ros-args --log-level debug"
```

## 👁️ Visualization

### RTABMap Visualization

RTABMap includes a built-in 3D visualization tool that shows:
- Real-time point cloud
- Loop closures
- Graph structure
- Odometry path

### RViz2 Configuration

Launch RViz2 separately and add these topics:

```bash
rviz2
```

**Essential Topics to Add:**
- **Map**: `/map` (nav_msgs/OccupancyGrid) - 2D occupancy grid
- **Point Cloud**: `/cloud_map` (sensor_msgs/PointCloud2) - 3D map
- **Path**: `/mapPath` (nav_msgs/Path) - Camera trajectory
- **Odometry**: `/odom` (nav_msgs/Odometry) - Current position
- **TF**: Add TF display for coordinate frames

**Frame Settings:**
- **Fixed Frame**: `map`
- **Target Frame**: `ascamera_hp60c_color_0`

## 🔧 Troubleshooting

### Common Issues

#### 1. Camera Not Detected

**Symptoms**: No camera topics, RTABMap can't find input
```bash
# Check camera topics
ros2 topic list | grep ascamera

# Test camera data
ros2 topic hz /ascamera_hp60c/camera_publisher/rgb0/image
```

**Solutions**:
- Verify camera USB connection
- Check camera driver launch file
- Ensure correct USB permissions

#### 2. Poor SLAM Performance

**Symptoms**: Tracking loss, poor map quality, drift

**Solutions**:
```bash
# Check lighting conditions (avoid direct sunlight/darkness)
# Move slowly (< 0.5 m/s)
# Point at textured surfaces
# Adjust max depth in config file
```

#### 3. Frame ID Mismatches

**Symptoms**: TF transform errors
```bash
# Check current frame IDs
ros2 topic echo /ascamera_hp60c/camera_publisher/rgb0/camera_info --once | grep frame_id

# View TF tree
ros2 run tf2_tools view_frames.py
```

#### 4. High CPU Usage

**Solutions**:
- Reduce image resolution in camera launch file
- Lower frame rate
- Adjust feature detection parameters
- Use hardware acceleration if available

#### 5. Memory Issues

**Symptoms**: RTABMap crashes, out of memory errors

**Solutions**:
```bash
# Limit memory usage in config
Mem/UseOdomGravity: "true"
Mem/RehearsalSimilarity: "0.8"  # Higher = more selective

# Clear database periodically
ros2 service call /rtabmap/reset std_srvs/srv/Empty
```

### Diagnostic Commands

```bash
# Check all RTABMap topics
ros2 topic list | grep -E "(odom|map|rtabmap)"

# Monitor processing rates
ros2 topic hz /odom
ros2 topic hz /map

# Check RTABMap status
ros2 service list | grep rtabmap

# View current parameters
ros2 param list /rtabmap

# Save current map
ros2 service call /rtabmap/save_map rtabmap_msgs/srv/SaveMap "{map_path: '/path/to/save/map.db'}"
```

## ⚡ Performance Tuning

### Camera Parameters

Optimize your camera settings in the ascamera launch file:

```python
parameters=[
    {"depth_width": 640},    # Reduce for better performance
    {"depth_height": 480},   # Reduce for better performance
    {"rgb_width": 640},      # Reduce for better performance
    {"rgb_height": 480},     # Reduce for better performance
    {"fps": 15},             # Lower FPS for slower systems
],
```

### RTABMap Parameters

Edit `ascam_hp60c.yaml` for your environment:

```yaml
# For outdoor/large spaces
Vis/MaxDepth: "10.0"
Kp/MaxDepth: "10.0"
Grid/MaxObstacleHeight: "3.0"

# For indoor/small spaces
Vis/MaxDepth: "3.0"
Kp/MaxDepth: "3.0"
Grid/MaxObstacleHeight: "1.5"

# For better performance (lower quality)
Kp/MaxFeatures: "200"
Vis/MinInliers: "10"

# For better quality (slower)
Kp/MaxFeatures: "600"
Vis/MinInliers: "30"
```

## 📁 File Structure

After setup, your file structure should look like:

```
~/rtabmap_ros2_ws/
├── src/
│   └── rtabmap_ros/
│       └── rtabmap_examples/
│           ├── launch/
│           │   └── ascam_hp60c.launch.py
│           └── config/
│               └── ascam_hp60c.yaml
├── build/
├── install/
└── log/

~/ascam_ros2_ws/
├── src/
│   └── ascamera/
├── build/
├── install/
└── log/

~/ (home directory)
├── rtabmap_ascam_launcher.sh
└── rtabmap_test.sh
```

## 🎯 Camera Specifications

**ASCAM HP60C Configuration Detected:**
- **Resolution**: 640x480 (RGB and Depth)
- **Frame Rate**: ~20 Hz
- **Intrinsics**: fx=565.26, fy=564.45, cx=336.35, cy=232.01
- **Frame ID**: `ascamera_hp60c_color_0`
- **Topics**:
  - RGB: `/ascamera_hp60c/camera_publisher/rgb0/image`
  - RGB Info: `/ascamera_hp60c/camera_publisher/rgb0/camera_info`
  - Depth: `/ascamera_hp60c/camera_publisher/depth0/image_raw`
  - Depth Info: `/ascamera_hp60c/camera_publisher/depth0/camera_info`
  - Point Cloud: `/ascamera_hp60c/camera_publisher/depth0/points`

## 📚 Additional Resources

- [RTABMap Documentation](http://wiki.ros.org/rtabmap_ros)
- [RTABMap Parameters](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h)
- [ROS 2 Launch System](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)

## 🤝 Contributing

To adapt this integration for other RGBD cameras:

1. **Identify Camera Topics**: Use `ros2 topic list` to find your camera's topic names
2. **Check Frame IDs**: Use `ros2 topic echo /your_camera/camera_info --once`
3. **Update Remappings**: Modify the `remappings` section in the launch file
4. **Adjust Parameters**: Tune the YAML config file for your camera's specifications
5. **Test Integration**: Use the diagnostic script to verify functionality

## 📄 License

This integration guide is provided under the same license as RTABMap ROS 2 packages.

---

**Happy SLAM Mapping! 🗺️**

For issues or improvements, please open an issue in this repository.
