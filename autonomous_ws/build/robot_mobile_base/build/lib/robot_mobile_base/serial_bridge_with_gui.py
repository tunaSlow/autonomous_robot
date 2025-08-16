#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial, struct, threading, time, tkinter as tk

HEADER = b'\xAA\x55'
PROTO_VER = 1

def crc16_ccitt(data: bytes, poly=0x1021, init=0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ poly) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.port = '/dev/ttyACM0'
        self.baud = 115200
        self.scale = 1000.0
        self.endianness = '<'  # little endian
        self.rate_hz = 50.0
        self.min_interval = 1.0 / self.rate_hz
        self.last_send_time = 0.0
        self.seq = 0
        self.mode = 1
        self.flags = 0
        self.current_topic = '/cmd_vel'

        self.ser = None
        self.serial_lock = threading.Lock()
        self.connect_serial()

        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 1)
        self.sub_cmd_nav = self.create_subscription(Twist, '/cmd_vel_nav', self.cb_cmd_vel_nav, 1)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.cb_odom, 5)

        self.reconnect_timer = self.create_timer(1.0, self.try_reconnect)

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.get_logger().info(f"Connected to {self.port}")
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f"Serial connection failed: {e}")

    def try_reconnect(self):
        if not self.ser or not self.ser.is_open:
            self.connect_serial()

    def safe_write(self, data: bytes):
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(data)
                    return True
                except Exception as e:
                    self.get_logger().warn(f"Write failed: {e}")
                    self.ser.close()
                    self.ser = None
        return False

    def build_frame(self, vx_f, vy_f, vw_f):
        s = self.scale
        vx, vy, vw = [int(max(min(v * s, 32767), -32768)) for v in (vx_f, vy_f, vw_f)]

        payload = struct.pack(f'{self.endianness}BBBBhhhBB',
                              PROTO_VER, self.mode, self.flags, 0,
                              vx, vy, vw,
                              self.seq & 0xFF, 0)

        payload_len = len(payload)
        payload = payload[:-1] + struct.pack('B', payload_len)
        crc = struct.pack('<H', crc16_ccitt(payload))
        return HEADER + payload + crc

    def send_triplet(self, vx_f, vy_f, vw_f):
        if time.time() - self.last_send_time < self.min_interval:
            return
        self.last_send_time = time.time()

        frame = self.build_frame(vx_f, vy_f, vw_f)
        if self.safe_write(frame):
            self.seq = (self.seq + 1) & 0xFF

    def cb_cmd_vel(self, msg):
        if self.current_topic == '/cmd_vel':
            self.send_triplet(msg.linear.x, msg.linear.y, msg.angular.z)

    def cb_cmd_vel_nav(self, msg):
        if self.current_topic == '/cmd_vel_nav':
            self.send_triplet(msg.linear.x, msg.linear.y, msg.angular.z)

    def cb_odom(self, msg):
        if self.current_topic == '/odom':
            twist = msg.twist.twist
            self.send_triplet(twist.linear.x, twist.linear.y, twist.angular.z)

    def switch_topic(self, topic):
        self.current_topic = topic
        self.get_logger().info(f"Switched to {topic}")

    def cleanup(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

def run_gui(node: SerialBridge):
    win = tk.Tk()
    win.title("ROS2 Serial Bridge")
    win.geometry("300x180")

    tk.Label(win, text="Current Topic").pack()
    topic_label = tk.Label(win, text=node.current_topic, font=("Arial", 12))
    topic_label.pack(pady=5)

    def switch(t):
        node.switch_topic(t)
        topic_label.config(text=t)

    for t in ['/cmd_vel', '/cmd_vel_nav', '/odom']:
        tk.Button(win, text=f"Switch to {t}", command=lambda t=t: switch(t)).pack(pady=2)

    tk.Button(win, text="Reconnect Serial", command=node.connect_serial).pack(pady=10)
    win.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    try:
        run_gui(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
