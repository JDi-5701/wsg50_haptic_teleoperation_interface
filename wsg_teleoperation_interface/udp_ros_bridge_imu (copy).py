#!/usr/bin/env python3

import socket
import time
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import numpy as np
from collections import deque

class UdpImuBridge(Node):
    def __init__(self):
        super().__init__('esp32_imu_node')
        
        # Declare parameters with default values
        self.declare_parameter('local_port', 5000)
        self.declare_parameter('esp32_address', '192.168.2.119')  # ESP32 IP address
        self.declare_parameter('esp32_port', 5000)
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('rate_window_size', 10)  # Number of samples for rate calculation
        
        # Get parameter values
        self.local_port = self.get_parameter('local_port').value
        self.esp32_address = (self.get_parameter('esp32_address').value, 
                            self.get_parameter('esp32_port').value)
        self.publish_rate = self.get_parameter('publish_rate').value
        self.rate_window_size = self.get_parameter('rate_window_size').value
        
        # Calculate minimum time between publishes
        self.min_publish_interval = 1.0 / self.publish_rate
        self.last_publish_time = time.time()
        
        # Rate monitoring
        self.message_times = deque(maxlen=self.rate_window_size)
        self.last_rate_print_time = time.time()
        self.rate_print_interval = 1.0  # Print rate every second
        
        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        try:
            # Bind to local port
            self.sock.bind(('0.0.0.0', self.local_port))
            self.get_logger().info(f"Bound to local port {self.local_port}")
            self.get_logger().info(f'Will communicate with ESP32 at {self.esp32_address[0]}:{self.esp32_address[1]}')
            self.get_logger().info(f'Publish rate set to {self.publish_rate} Hz')
        except Exception as e:
            self.get_logger().error(f"Failed to bind socket: {e}")
            raise
        
        # ROS Publisher for IMU data
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        
        # Create IMU message
        self.imu_msg = Imu()
        
        # Set socket timeout
        self.sock.settimeout(0.1)

        # Create a timer to check for UDP messages
        self.timer = self.create_timer(0.01, self.check_udp_messages)  # 100Hz

    def calculate_rate(self):
        if len(self.message_times) < 2:
            return 0.0
        time_diff = self.message_times[-1] - self.message_times[0]
        if time_diff == 0:
            return 0.0
        return (len(self.message_times) - 1) / time_diff

    def check_udp_messages(self):
        try:
            # Receive data from the ESP32
            data, addr = self.sock.recvfrom(1024)  # Buffer size is 1024 bytes
            
            # Only process messages from the specified ESP32
            if addr[0] == self.esp32_address[0]:
                current_time = time.time()
                self.message_times.append(current_time)
                
                # Print rate every rate_print_interval seconds
                if current_time - self.last_rate_print_time >= self.rate_print_interval:
                    rate = self.calculate_rate()
                    self.get_logger().info(f'Current message rate: {rate:.2f} Hz')
                    self.last_rate_print_time = current_time
                
                # Unpack the received data (6 float values)
                imu_data = struct.unpack('ffffff', data)  # 6 float values
                
                # Create header
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = 'imu_link'
                
                # Fill IMU message
                self.imu_msg.header = header
                
                # Set linear acceleration (in m/s^2)
                self.imu_msg.linear_acceleration.x = imu_data[0]
                self.imu_msg.linear_acceleration.y = imu_data[1]
                self.imu_msg.linear_acceleration.z = imu_data[2]
                
                # Set angular velocity (in rad/s)
                self.imu_msg.angular_velocity.x = imu_data[3]
                self.imu_msg.angular_velocity.y = imu_data[4]
                self.imu_msg.angular_velocity.z = imu_data[5]
                
                # Set covariance matrices (unknown)
                self.imu_msg.linear_acceleration_covariance = [0.0] * 9
                self.imu_msg.angular_velocity_covariance = [0.0] * 9
                self.imu_msg.orientation_covariance = [0.0] * 9
                
                # Only publish if rate limit
                if current_time - self.last_publish_time >= self.min_publish_interval:
                    self.imu_pub.publish(self.imu_msg)
                    self.last_publish_time = current_time
                    self.get_logger().debug(
                        f'IMU Data - Accel: [{imu_data[0]:.2f}, {imu_data[1]:.2f}, {imu_data[2]:.2f}], '
                        f'Gyro: [{imu_data[3]:.2f}, {imu_data[4]:.2f}, {imu_data[5]:.2f}]'
                    )
            else:
                self.get_logger().debug(f"Ignoring message from {addr[0]}")
        except socket.timeout:
            # Timeout is expected, just continue
            pass
        except Exception as e:
            self.get_logger().error(f"Error receiving message: {e}")

    def shutdown(self):
        if hasattr(self, 'sock'):
            self.sock.close()
        self.get_logger().info("Socket closed")

def main(args=None):
    rclpy.init(args=args)
    try:
        bridge = UdpImuBridge()
        bridge.get_logger().info("Starting UDP IMU bridge node...")
        
        # Use the default executor
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(bridge)
        
        try:
            executor.spin()
        finally:
            bridge.shutdown()
            executor.remove_node(bridge)
            
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 

