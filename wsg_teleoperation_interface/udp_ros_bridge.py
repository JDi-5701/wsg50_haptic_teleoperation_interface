#!/usr/bin/env python3

import socket
import time
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import threading
from collections import deque

class UdpKnobBridge(Node):
    def __init__(self):
        super().__init__('esp32_knob_node')
        
        # Declare parameters with default values
        self.declare_parameter('local_port', 5000)
        self.declare_parameter('esp32_address', '10.200.2.148')  # Default to knob ESP32
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
        
        # ROS Publishers and Subscribers
        self.knob_state_pub = self.create_publisher(Int32, 'knob_state', 10)
        self.knob_torque_pub = self.create_publisher(Float32, 'knob_torque', 10)
        self.get_logger().info("Created publisher for topic 'knob_state'")
        
        # Create subscription with absolute topic name to match the publisher
        self.force_sub = self.create_subscription(
            Float32,
            '/knob_force_command',  # Use absolute topic name
            self.force_callback,
            10
        )
        self.get_logger().info("Created subscription to topic '/knob_force_command'")
        
        # Create message
        self.knob_state_msg = Int32()
        self.knob_state_msg.data = 0

        self.knob_torque_msg = Float32()
        self.knob_torque_msg.data = 0.0

        # Flag to control the UDP thread
        self.running = True

        # Start UDP thread
        self.udp_thread = threading.Thread(target=self.udp_receive_loop)
        self.udp_thread.start()

    def force_callback(self, msg):
        try:
            #self.get_logger().info(f'Received Force Command: {msg.data:.2f}')
            # Create a message with force value from ROS topic
            message = struct.pack('f', msg.data)  # 'f' for float
            self.sock.sendto(message, self.esp32_address)
            #time.sleep(0.1)
            #self.get_logger().info(f'Sent Force to ESP32: {msg.data:.2f}')
        except Exception as e:
            self.get_logger().error(f"Error sending message: {e}")

    def calculate_rate(self):
        if len(self.message_times) < 2:
            return 0.0
        time_diff = self.message_times[-1] - self.message_times[0]
        if time_diff == 0:
            return 0.0
        return (len(self.message_times) - 1) / time_diff

    def udp_receive_loop(self):
        self.get_logger().info("UDP receive thread started...")
        while self.running and rclpy.ok():
            try:
                # Set timeout for recvfrom to allow checking running flag
                self.sock.settimeout(0.1)
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
                            #self.get_logger().info(f'Current message rate: {rate:.2f} Hz')
                            self.last_rate_print_time = current_time
                        
                        # Unpack the received data (position as int32)
                        #position = struct.unpack('i', data)[0]  # 'i' for int32
                        position = struct.unpack('<i', data[0:4])[0]  # 'i' for int32
                        torque = struct.unpack('<f', data[4:8])[0]
                        #self.get_logger().info(f'Received Knob Position: {position}')

                        # Rate limit publishing
                        if current_time - self.last_publish_time >= self.min_publish_interval:
                            self.knob_state_msg.data = position
                            self.knob_state_pub.publish(self.knob_state_msg)

                            self.knob_torque_msg.data = torque
                            self.knob_torque_pub.publish(self.knob_torque_msg)

                            self.last_publish_time = current_time
                    else:
                        self.get_logger().debug(f"Ignoring message from {addr[0]}")
                except socket.timeout:
                    # Timeout is expected, just continue the loop
                    continue
                
            except Exception as e:
                self.get_logger().error(f"Error receiving message: {e}")
                time.sleep(1)  # Wait before retrying

    def shutdown(self):
        self.running = False
        if hasattr(self, 'udp_thread'):
            self.udp_thread.join()
        if hasattr(self, 'sock'):
            self.sock.close()
        self.get_logger().info("Socket closed")

def main(args=None):
    rclpy.init(args=args)
    try:
        bridge = UdpKnobBridge()
        bridge.get_logger().info("Starting UDP bridge node...")
        
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
