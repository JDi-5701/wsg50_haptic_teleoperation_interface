#!/usr/bin/env python3

import socket
import time
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import threading
from collections import deque
from builtin_interfaces.msg import Time
from rclpy.time import Time as RclTime

class UdpKnobBridge(Node):
    def __init__(self):
        super().__init__('esp32_knob_node')
        
        # --- 参数设置 ---
        self.declare_parameter('local_port', 5000)
        self.declare_parameter('esp32_address', '10.200.2.148')
        self.declare_parameter('esp32_port', 5000)
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('rate_window_size', 10)
        
        self.local_port = self.get_parameter('local_port').value
        self.esp32_address = (
            self.get_parameter('esp32_address').value,
            self.get_parameter('esp32_port').value
        )
        self.publish_rate = self.get_parameter('publish_rate').value
        self.rate_window_size = self.get_parameter('rate_window_size').value

        self.min_publish_interval = 1.0 / self.publish_rate
        self.last_publish_time = time.time()
        self.message_times = deque(maxlen=self.rate_window_size)
        self.last_rate_print_time = time.time()
        self.rate_print_interval = 1.0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(('0.0.0.0', self.local_port))
            self.get_logger().info(f"Bound to local port {self.local_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to bind socket: {e}")
            raise

        # --- ROS Publishers ---
        self.knob_state_pub = self.create_publisher(Int32, 'knob_state', 10)
        self.knob_torque_pub = self.create_publisher(Float32, 'knob_torque', 10)
        self.esp32_force_pub = self.create_publisher(Float32, 'gripper_finger_force', 10)

        self.knob_state_msg = Int32()
        self.knob_torque_msg = Float32()
        self.esp32_force_msg = Float32()


        # --- 启动 UDP 接收线程 ---
        self.running = True
        self.udp_thread = threading.Thread(target=self.udp_receive_loop)
        self.udp_thread.start()

        self.get_logger().info("=== UdpKnobBridge node started ===")

    def calculate_rate(self):
        if len(self.message_times) < 2:
            return 0.0
        time_diff = self.message_times[-1] - self.message_times[0]
        return (len(self.message_times) - 1) / time_diff if time_diff > 0 else 0.0

    def udp_receive_loop(self):
        self.get_logger().info("UDP receive thread started...")
        while self.running and rclpy.ok():
            try:
                self.sock.settimeout(0.1)
                data, addr = self.sock.recvfrom(1024)
                if addr[0] != self.esp32_address[0]:
                    continue

                current_time = time.time()
                self.message_times.append(current_time)

                if current_time - self.last_rate_print_time >= self.rate_print_interval:
                    rate = self.calculate_rate()
                    self.get_logger().info(f'Current message rate: {rate:.2f} Hz')
                    self.last_rate_print_time = current_time

                # --- 解包 MotorMsg ---
                if len(data) >= 36:
                    unpacked = struct.unpack('<ii f Q Q f i', data[:36])
                    (
                        msg_id,
                        fsr_value,
                        force_filtered,
                        force_ts,
                        motor_ts,
                        motor_torque,
                        knob_state
                    ) = unpacked

                    # --- 打印简要信息 ---
                    # self.get_logger().info(
                    #     f"[UDP] id={msg_id}, FSR={fsr_value}, Force={force_filtered:.2f} N, "
                    #     f"force_ts={force_ts} µs, motor_ts={motor_ts} µs, τ={motor_torque:.2f} V"
                    # )


                    print("[UDP Raw Message]",
                          f"id: {msg_id}, fsr: {fsr_value}, force: {force_filtered:.2f},",
                          f"force_ts: {force_ts}, motor_ts: {motor_ts},",
                          f"torque: {motor_torque:.2f}, knob: {knob_state}")


                    # --- 限频发布 ---
                    if current_time - self.last_publish_time >= self.min_publish_interval:
                        ros_now = self.get_clock().now()
                        latency_us = motor_ts - force_ts
                        latency_sec = latency_us / 1e6

                        #force_ros_time = ros_now - rclpy.duration.Duration(seconds=latency_sec)

                        #self.knob_torque_msg.header.stamp = ros_now.to_msg()
                        #self.esp32_force_msg.header.stamp = force_ros_time.to_msg()

                        self.knob_state_msg.data = knob_state
                        self.knob_torque_msg.data = motor_torque
                        self.esp32_force_msg.data = force_filtered

                        self.knob_state_pub.publish(self.knob_state_msg)
                        self.knob_torque_pub.publish(self.knob_torque_msg)
                        self.esp32_force_pub.publish(self.esp32_force_msg)

                        self.last_publish_time = current_time

                else:
                    self.get_logger().warn(f"Invalid UDP packet size: {len(data)} bytes")

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP error: {e}")
                time.sleep(1)

    def shutdown(self):
        self.running = False
        if hasattr(self, 'udp_thread'):
            self.udp_thread.join()
        self.sock.close()
        self.get_logger().info("Socket closed")

def main(args=None):

    rclpy.init(args=args)
    try:
        node = UdpKnobBridge()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except Exception as e:
        print(f"Exception in main: {e}")
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
