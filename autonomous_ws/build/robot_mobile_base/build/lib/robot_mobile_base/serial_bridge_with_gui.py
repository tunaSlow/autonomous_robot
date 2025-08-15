#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import serial.tools.list_ports
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk

HEADER = b'\xAA\x55'
PROTO_VER = 1

def crc16_ccitt(data: bytes, poly=0x1021, init=0xFFFF) -> int:
    """
    CRC-16/CCITT-FALSE (init=0xFFFF, poly=0x1021, refin=false, refout=false, xorout=0x0000)
    """
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if (crc & 0x8000) != 0:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # ---- Parameters ----
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('endianness', 'little')  # little|big
        self.declare_parameter('scale', 1000.0)         # m/s -> int16 scale
        self.declare_parameter('rate_hz', 50.0)         # max send rate
        self.declare_parameter('topic_cmd', '/cmd_vel')
        self.declare_parameter('topic_cmd_nav', '/cmd_vel_nav')
        self.declare_parameter('topic_odom', '/odom')   # optional

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.endianness = self.get_parameter('endianness').get_parameter_value().string_value
        self.scale = float(self.get_parameter('scale').get_parameter_value().double_value or
                           self.get_parameter('scale').value)
        self.rate_hz = float(self.get_parameter('rate_hz').get_parameter_value().double_value or
                             self.get_parameter('rate_hz').value)
        self.topic_cmd = self.get_parameter('topic_cmd').get_parameter_value().string_value
        self.topic_cmd_nav = self.get_parameter('topic_cmd_nav').get_parameter_value().string_value
        self.topic_odom = self.get_parameter('topic_odom').get_parameter_value().string_value

        # ---- Serial ----
        self.ser = None
        self.serial_lock = threading.Lock()
        self.write_timeout = 0.05
        self.connect_serial()  # initial attempt

        # ---- State / mode ----
        self.current_topic = self.topic_cmd
        self.mode = 1          # your custom 'mode' byte
        self.flags = 0         # could map to 'odom_cal' or runtime flags
        self.seq = 0
        self.last_send_time = 0.0
        self.min_interval = 1.0 / max(self.rate_hz, 1.0)

        # ---- QoS ----
        qos_cmd = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
        )
        qos_odom = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # ---- Subscriptions (keep both, switch via flag) ----
        self.sub_cmd = self.create_subscription(Twist, self.topic_cmd, self.cb_cmd_vel, qos_cmd)
        self.sub_cmd_nav = self.create_subscription(Twist, self.topic_cmd_nav, self.cb_cmd_vel_nav, qos_cmd)
        self.sub_odom = self.create_subscription(Odometry, self.topic_odom, self.cb_odom, qos_odom)

        self.get_logger().info(f"Subscribed to {self.topic_cmd}, {self.topic_cmd_nav}, {self.topic_odom}")

        # ---- Timer to try reconnect if serial down ----
        self.reconnect_timer = self.create_timer(1.0, self.try_reconnect)

        # ---- Mutex for topic switching ----
        self.topic_lock = threading.Lock()

    # ------------- Serial -------------
    def connect_serial(self):
        try:
            if self.ser and self.ser.is_open:
                return
            self.ser = serial.Serial(
                self.port, self.baud, timeout=0.01, write_timeout=self.write_timeout
            )
            self.get_logger().info(f"Serial opened: {self.port} @ {self.baud}")
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f"Serial open failed on {self.port}: {e}")

    def try_reconnect(self):
        if not self.ser or not self.ser.is_open:
            self.connect_serial()

    def safe_write(self, data: bytes):
        with self.serial_lock:
            if not self.ser or not self.ser.is_open:
                return False
            try:
                self.ser.write(data)
                return True
            except Exception as e:
                self.get_logger().warn(f"Serial write failed: {e}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                return False

    # ------------- Callbacks -------------
    def cb_cmd_vel(self, msg: Twist):
        with self.topic_lock:
            if self.current_topic != self.topic_cmd:
                return
        self.publish_packet_from_twist(msg)

    def cb_cmd_vel_nav(self, msg: Twist):
        with self.topic_lock:
            if self.current_topic != self.topic_cmd_nav:
                return
        self.publish_packet_from_twist(msg)

    def cb_odom(self, msg: Odometry):
        with self.topic_lock:
            if self.current_topic != self.topic_odom:
                return
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vw = msg.twist.twist.angular.z
        self.send_triplet(vx, vy, vw)

    # ------------- Build & send packet -------------
    def publish_packet_from_twist(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        vw = msg.angular.z
        self.send_triplet(vx, vy, vw)

    def send_triplet(self, vx_f: float, vy_f: float, vw_f: float):
        now = time.time()
        if (now - self.last_send_time) < self.min_interval:
            return  # rate limit
        self.last_send_time = now

        s = self.scale
        vx = int(max(min(vx_f * s, 32767), -32768))
        vy = int(max(min(vy_f * s, 32767), -32768))
        vw = int(max(min(vw_f * s, 32767), -32768))

        # endianness
        endian = '<' if self.endianness.lower().startswith('l') else '>'
        # Frame: [AA 55][ver][mode][flags][vx][vy][vw][seq][len][crc16_le]
        # Pack payload (ver..len) first for CRC
        payload = struct.pack(
            f'{endian}BBBBhhhBB',
            PROTO_VER, self.mode, self.flags, 0,  # the '0' can be reserved
            vx, vy, vw,
            self.seq & 0xFF,
            0  # placeholder len; we'll set correct length below
        )
        # length excludes header (2 bytes) and CRC (2 bytes), include payload bytes
        payload_len = len(payload)
        # replace length byte (last one) with actual len
        payload = payload[:-1] + struct.pack('B', payload_len)

        # Compute CRC over payload only (common choice). You can also CRC header+payload.
        crc = crc16_ccitt(payload)
        # CRC little-endian on the wire for simplicity
        crc_bytes = struct.pack('<H', crc)

        frame = HEADER + payload + crc_bytes

        ok = self.safe_write(frame)
        if ok:
            self.seq = (self.seq + 1) & 0xFF

    # ------------- Topic switching -------------
    def switch_topic(self, new_topic: str):
        with self.topic_lock:
            if new_topic != self.current_topic:
                self.get_logger().info(f"Switching to {new_topic}")
                self.current_topic = new_topic

    def set_flag(self, flags: int):
        self.flags = flags & 0xFF

    # ------------- Cleanup -------------
    def cleanup(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

# ---------------- GUI ----------------
def list_serial_ports() -> list[str]:
    return [p.device for p in serial.tools.list_ports.comports()]

def run_gui(node: SerialBridge):
    window = tk.Tk()
    window.title("ROS 2 → STM32 Serial Bridge")
    window.geometry("360x240")

    current_label = tk.Label(window, text=f"Current topic: {node.current_topic}", font=("Arial", 12))
    current_label.pack(pady=8)

    # Topic buttons
    frm_topics = tk.Frame(window)
    frm_topics.pack(pady=5)

    def set_topic(t):
        node.switch_topic(t)
        current_label.config(text=f"Current topic: {t}")

    tk.Button(frm_topics, text=f"Use {node.topic_cmd}", width=22, command=lambda: set_topic(node.topic_cmd)).grid(row=0, column=0, padx=4, pady=2)
    tk.Button(frm_topics, text=f"Use {node.topic_cmd_nav}", width=22, command=lambda: set_topic(node.topic_cmd_nav)).grid(row=0, column=1, padx=4, pady=2)
    tk.Button(frm_topics, text=f"Use {node.topic_odom}", width=22, command=lambda: set_topic(node.topic_odom)).grid(row=1, column=0, columnspan=2, padx=4, pady=2)

    # Flags toggle (e.g., odom_cal)
    flags_var = tk.IntVar(value=node.flags)
    def on_flags_change():
        node.set_flag(flags_var.get())
    tk.Checkbutton(window, text="Odom Cal (flag bit0)", variable=flags_var, command=on_flags_change).pack(pady=4)

    # Serial controls
    frm_serial = tk.Frame(window)
    frm_serial.pack(pady=6)
    tk.Label(frm_serial, text="Serial Port:").grid(row=0, column=0, sticky="e", padx=4)
    port_combo = ttk.Combobox(frm_serial, values=list_serial_ports(), width=20)
    port_combo.set(node.port)
    port_combo.grid(row=0, column=1, padx=4)

    def refresh_ports():
        port_combo['values'] = list_serial_ports()
    tk.Button(frm_serial, text="Refresh", command=refresh_ports).grid(row=0, column=2, padx=4)

    def connect_selected():
        node.port = port_combo.get()
        if node.ser and node.ser.is_open:
            try:
                node.ser.close()
            except Exception:
                pass
        node.connect_serial()
    tk.Button(frm_serial, text="Connect", command=connect_selected).grid(row=1, column=1, pady=4)

    window.mainloop()

# ---------------- Main ----------------
def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()

    # Spin ROS in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        run_gui(node)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
