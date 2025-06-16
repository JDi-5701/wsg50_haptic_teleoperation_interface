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
        self.declare_parameter('esp32_address', '192.168.2.119')
        self.declare_parameter('esp32_port', 5000)
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('rate_window_size', 10)
        self.declare_parameter('filter_window_size', 5)

        # Get parameter values
        self.local_port = self.get_parameter('local_port').value
        self.esp32_address = (self.get_parameter('esp32_address').value, 
                            self.get_parameter('esp32_port').value)
        self.publish_rate = self.get_parameter('publish_rate').value
        self.rate_window_size = self.get_parameter('rate_window_size').value
        self.filter_window_size = self.get_parameter('filter_window_size').value

        self.min_publish_interval = 1.0 / self.publish_rate
        self.last_publish_time = time.time()

        self.message_times = deque(maxlen=self.rate_window_size)
        self.last_rate_print_time = time.time()
        self.rate_print_interval = 1.0

        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        try:
            self.sock.bind(('0.0.0.0', self.local_port))
            self.get_logger().info(f"Bound to local port {self.local_port}")
            self.get_logger().info(f'Will communicate with ESP32 at {self.esp32_address[0]}:{self.esp32_address[1]}')
            self.get_logger().info(f'Publish rate set to {self.publish_rate} Hz')
        except Exception as e:
            self.get_logger().error(f"Failed to bind socket: {e}")
            raise

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.imu_msg = Imu()
        self.sock.settimeout(0.1)

        # Buffers for filtering
        self.accel_buffer = [deque(maxlen=self.filter_window_size) for _ in range(3)]
        self.gyro_buffer = [deque(maxlen=self.filter_window_size) for _ in range(3)]

        self.timer = self.create_timer(0.01, self.check_udp_messages)

    def calculate_rate(self):
        if len(self.message_times) < 2:
            return 0.0
        time_diff = self.message_times[-1] - self.message_times[0]
        if time_diff == 0:
            return 0.0
        return (len(self.message_times) - 1) / time_diff

    def filter_data(self, buffer_list):
        return [float(np.mean(buf)) if len(buf) > 0 else 0.0 for buf in buffer_list]

    def check_udp_messages(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            if addr[0] == self.esp32_address[0]:
                current_time = time.time()
                self.message_times.append(current_time)

                if current_time - self.last_rate_print_time >= self.rate_print_interval:
                    rate = self.calculate_rate()
                    self.get_logger().info(f'Current message rate: {rate:.2f} Hz')
                    self.last_rate_print_time = current_time

                imu_data = struct.unpack('ffffff', data)

                # Update buffers
                for i in range(3):
                    self.accel_buffer[i].append(imu_data[i])
                    self.gyro_buffer[i].append(imu_data[i+3])

                filtered_accel = self.filter_data(self.accel_buffer)
                filtered_gyro = self.filter_data(self.gyro_buffer)

                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = 'imu_link'

                self.imu_msg.header = header
                self.imu_msg.linear_acceleration.x = filtered_accel[0]
                self.imu_msg.linear_acceleration.y = filtered_accel[1]
                self.imu_msg.linear_acceleration.z = filtered_accel[2]
                self.imu_msg.angular_velocity.x = filtered_gyro[0]
                self.imu_msg.angular_velocity.y = filtered_gyro[1]
                self.imu_msg.angular_velocity.z = filtered_gyro[2]

                self.imu_msg.linear_acceleration_covariance = [0.0] * 9
                self.imu_msg.angular_velocity_covariance = [0.0] * 9
                self.imu_msg.orientation_covariance = [0.0] * 9

                if current_time - self.last_publish_time >= self.min_publish_interval:
                    self.imu_pub.publish(self.imu_msg)
                    self.last_publish_time = current_time
        
        except socket.timeout:
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
