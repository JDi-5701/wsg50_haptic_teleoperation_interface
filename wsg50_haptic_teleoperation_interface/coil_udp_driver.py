#!/usr/bin/env python3
"""SUPERSEDED by omni_hgi_bridge.py - kept for reference only.

Written against an earlier firmware and no longer matches wifi_task.h:

  * TX packs '<ff' (8 B); receiveUdpForce() dispatches on size and only
    accepts ForceMsg3D at 12 B, so every command is dropped with
    "Received packet of unexpected size 8" and the coils never actuate.
  * RX expects '<i' (4 B); the board sends MotorMsg at 36 B, so the
    `len(data) == UDP_RX_SIZE` test never passes and /knob_state is silent.
  * Binds :5001 and publishes /knob_state, colliding with udp_ros_bridge.py,
    which talks to the same board.

omni_hgi_bridge.py replaces both nodes with one socket and the current
protocol. Run that instead.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import WrenchStamped
import socket
import struct

# --- 网络配置 ---
PC_IP = "0.0.0.0"
PC_PORT = 5001
ESP32_IP = "10.200.2.148"
ESP32_PORT = 5000

# --- 结构体定义 (ESP32 侧 ForceMsg 为 2个 float) ---
UDP_TX_FORMAT = "<ff" 
UDP_RX_FORMAT = "<i"
UDP_RX_SIZE = struct.calcsize(UDP_RX_FORMAT)

class Esp32ForceInterface(Node):
    def __init__(self):
        super().__init__('esp32_force_interface')
        
        # 1. ROS 2 话题
        self.publisher_ = self.create_publisher(Int32, 'knob_state', 10)
        self.subscription = self.create_subscription(
            WrenchStamped, 
            '/tactile_resultant_wrench', 
            self.wrench_callback, 
            10)
        
        # 2. UDP 套接字
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((PC_IP, PC_PORT))
        self.sock.setblocking(False)
        
        # 3. 状态变量
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.log_counter = 0 # 计数器

        # 4. 循环定时器 (200Hz: 0.005s)
        self.timer = self.create_timer(0.005, self.udp_loop)
        self.get_logger().info(f'UDP Bridge Active. Sending at 200Hz, Logging at 10Hz.')

    def wrench_callback(self, msg):
        """手动定义比例转换逻辑"""
        RATIO = 0.08 
        MAX_DUTY = 0.8 

        raw_x = msg.wrench.force.x
        raw_y = msg.wrench.force.y
        
        duty_x = raw_x * RATIO
        duty_y = raw_y * RATIO
        
        self.cmd_x = max(min(duty_x, MAX_DUTY), -MAX_DUTY)
        self.cmd_y = max(min(duty_y, MAX_DUTY), -MAX_DUTY)

    def udp_loop(self):
        # A. 发送力指令
        try:
            tx_data = struct.pack(UDP_TX_FORMAT, float(self.cmd_x), float(self.cmd_y))
            self.sock.sendto(tx_data, (ESP32_IP, ESP32_PORT))
            
            # 降低日志频率至 10Hz (200Hz / 20 = 10Hz)
            self.log_counter += 1
            if self.log_counter >= 20:
                self.get_logger().info(f'[UDP Send] X: {self.cmd_x:.3f}, Y: {self.cmd_y:.3f}')
                self.log_counter = 0
                
        except Exception as e:
            self.get_logger().error(f'UDP Send Error: {e}')

        # B. 接收反馈
        try:
            data, addr = self.sock.recvfrom(1024)
            if len(data) == UDP_RX_SIZE:
                raw_state = struct.unpack(UDP_RX_FORMAT, data)[0]
                msg_out = Int32()
                msg_out.data = raw_state
                self.publisher_.publish(msg_out)
        except BlockingIOError:
            pass
        except Exception as e:
            self.get_logger().error(f'UDP Recv Error: {e}')

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Esp32ForceInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
