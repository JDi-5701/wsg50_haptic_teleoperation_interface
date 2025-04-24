#!/usr/bin/env python3

import socket
import time
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import numpy as np
from collections import deque

class UdpForceSensorBridge(Node):
    def __init__(self):
        super().__init__('esp32_force_sensor_node')
        
        # Declare parameters with default values
        self.declare_parameter('local_port', 5000)
        self.declare_parameter('esp32_address', '10.200.2.149')  # Force sensor ESP32
        self.declare_parameter('esp32_port', 5000)
        self.declare_parameter('filter_window_size', 5)
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('rate_window_size', 10)  # Number of samples for rate calculation
        
        # Get parameter values
        self.local_port = self.get_parameter('local_port').value
        self.esp32_address = (self.get_parameter('esp32_address').value, 
                            self.get_parameter('esp32_port').value)
        self.filter_window_size = self.get_parameter('filter_window_size').value
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
        
        # ROS Publishers
        self.esp32_force_raw_pub = self.create_publisher(Int32, 'gripper_finger_force_raw', 10)
        self.esp32_force_pub = self.create_publisher(Float32, 'gripper_finger_force', 10)
        
        # Create messages
        self.esp32_force_raw_msg = Int32()
        self.esp32_force_msg = Float32()
        
        # Force filtering parameters
        self.force_history = deque(maxlen=self.filter_window_size)
        self.last_filtered_force = 0.0

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
    
    def calibrate_force_in_N(self, esp32_raw_force):
        return 0.0036 * esp32_raw_force - 3.4051
    
    def filter_force(self, force):
        """
        Apply a low-latency moving average filter to the force data.
        Uses a small window size and exponential weighting for minimal delay.
        """
        self.force_history.append(force)
        
        # If we don't have enough samples yet, return the raw value
        if len(self.force_history) < self.filter_window_size:
            return force
        
        # Calculate weighted moving average
        weights = np.exp(np.linspace(0, 1, self.filter_window_size))
        weights = weights / np.sum(weights)
        
        filtered_force = np.average(list(self.force_history), weights=weights)
        
        # Apply a small deadzone to reduce noise
        if abs(filtered_force) < 0.1:
            filtered_force = 0.0
            
        return filtered_force

    def check_udp_messages(self):
        try:
            # Receive data from the ESP32
            data, addr = self.sock.recvfrom(1024)  # Buffer size is 1024 bytes
            
            # Debug log for received messages
            self.get_logger().debug(f"Received message from {addr[0]}:{addr[1]}, expected ESP32 at {self.esp32_address[0]}:{self.esp32_address[1]}")
            
            # Only process messages from the specified ESP32
            if addr[0] == self.esp32_address[0]:
                current_time = time.time()
                self.message_times.append(current_time)
                
                # Print rate every rate_print_interval seconds
                if current_time - self.last_rate_print_time >= self.rate_print_interval:
                    rate = self.calculate_rate()
                    self.get_logger().info(f'Current message rate: {rate:.2f} Hz')
                    self.last_rate_print_time = current_time
                
                # Unpack the received data (force as int32_t)
                raw_force = struct.unpack('i', data)[0]  # 'i' for int32
                
                # Publish raw force
                self.esp32_force_raw_msg.data = raw_force
                self.esp32_force_raw_pub.publish(self.esp32_force_raw_msg)
                
                # Calibrate and filter the force
                force_in_N = self.calibrate_force_in_N(raw_force)
                filtered_force = self.filter_force(force_in_N)
                
                # Only publish if rate limit
                if current_time - self.last_publish_time >= self.min_publish_interval:
                    self.esp32_force_msg.data = float(filtered_force)  # Ensure float type
                    self.esp32_force_pub.publish(self.esp32_force_msg)
                    self.last_filtered_force = filtered_force
                    self.last_publish_time = current_time
                    self.get_logger().debug(f'Raw force: {raw_force}, Filtered force in N: {filtered_force:.2f}')
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
        bridge = UdpForceSensorBridge()
        bridge.get_logger().info("Starting UDP force sensor bridge node...")
        
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
