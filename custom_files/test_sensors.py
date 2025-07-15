#!/usr/bin/env python3

"""
Sensor test script for RTAB-Map setup
This script checks if all required sensors are publishing data correctly.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from threading import Lock
import time

class SensorTester(Node):
    def __init__(self):
        super().__init__('sensor_tester')
        
        self.lock = Lock()
        self.sensors_status = {
            'rgb_image': {'received': False, 'count': 0, 'last_time': None},
            'depth_image': {'received': False, 'count': 0, 'last_time': None},
            'rgb_camera_info': {'received': False, 'count': 0, 'last_time': None},
            'depth_camera_info': {'received': False, 'count': 0, 'last_time': None},
            'laser_scan': {'received': False, 'count': 0, 'last_time': None}
        }
        
        # Subscribers for your specific topics
        self.rgb_sub = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image', 
            self.rgb_callback, 10)
        
        self.depth_sub = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw', 
            self.depth_callback, 10)
            
        self.rgb_info_sub = self.create_subscription(
            CameraInfo, '/ascamera_hp60c/camera_publisher/rgb0/camera_info', 
            self.rgb_info_callback, 10)
            
        self.depth_info_sub = self.create_subscription(
            CameraInfo, '/ascamera_hp60c/camera_publisher/depth0/camera_info', 
            self.depth_info_callback, 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/ldlidar_node/scan', 
            self.scan_callback, 10)
        
        # Status reporting timer
        self.timer = self.create_timer(2.0, self.report_status)
        
        self.get_logger().info('Sensor tester started. Monitoring sensors...')
        
    def rgb_callback(self, msg):
        with self.lock:
            self.sensors_status['rgb_image']['received'] = True
            self.sensors_status['rgb_image']['count'] += 1
            self.sensors_status['rgb_image']['last_time'] = time.time()
            
    def depth_callback(self, msg):
        with self.lock:
            self.sensors_status['depth_image']['received'] = True
            self.sensors_status['depth_image']['count'] += 1
            self.sensors_status['depth_image']['last_time'] = time.time()
            
    def rgb_info_callback(self, msg):
        with self.lock:
            self.sensors_status['rgb_camera_info']['received'] = True
            self.sensors_status['rgb_camera_info']['count'] += 1
            self.sensors_status['rgb_camera_info']['last_time'] = time.time()
            
    def depth_info_callback(self, msg):
        with self.lock:
            self.sensors_status['depth_camera_info']['received'] = True
            self.sensors_status['depth_camera_info']['count'] += 1
            self.sensors_status['depth_camera_info']['last_time'] = time.time()
            
    def scan_callback(self, msg):
        with self.lock:
            self.sensors_status['laser_scan']['received'] = True
            self.sensors_status['laser_scan']['count'] += 1
            self.sensors_status['laser_scan']['last_time'] = time.time()
    
    def report_status(self):
        current_time = time.time()
        self.get_logger().info("=== Sensor Status Report ===")
        
        all_good = True
        for sensor, status in self.sensors_status.items():
            if status['received']:
                age = current_time - status['last_time'] if status['last_time'] else 999
                rate = status['count'] / 2.0  # Messages per 2 seconds
                status_str = f"✅ {sensor}: {status['count']} msgs, {rate:.1f} Hz, {age:.1f}s ago"
                if age > 5.0:  # No data for 5 seconds
                    status_str += " (STALE)"
                    all_good = False
            else:
                status_str = f"❌ {sensor}: No data received"
                all_good = False
                
            self.get_logger().info(status_str)
        
        if all_good:
            self.get_logger().info("🎉 All sensors are working! Ready for RTAB-Map!")
        else:
            self.get_logger().info("⚠️  Some sensors need attention before running RTAB-Map")
            
        # Reset counters
        with self.lock:
            for status in self.sensors_status.values():
                status['count'] = 0

def main(args=None):
    rclpy.init(args=args)
    node = SensorTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
