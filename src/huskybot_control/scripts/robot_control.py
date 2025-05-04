#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys                                           # Import sys untuk exit/error
import rclpy                                         # Import utama ROS2 Python
from rclpy.node import Node                          # Base class Node ROS2
from geometry_msgs.msg import Twist                  # Import pesan Twist (untuk cmd_vel)
from sensor_msgs.msg import Joy                      # Import pesan Joy (untuk joystick)
import threading                                     # Import threading untuk multi-thread executor dan lock

class Commander(Node):                               # Node publisher utama ke /huskybot/cmd_vel
    def __init__(self):
        super().__init__('commander')                # Inisialisasi node dengan nama 'commander'
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/huskybot/cmd_vel').value  # Parameter topic cmd_vel (bisa di-remap via launch/YAML)
        self.safe_distance = self.declare_parameter('safe_distance', 0.5).value                 # Parameter jarak aman (bisa di-set via launch/YAML)
        self.max_speed = self.declare_parameter('max_speed', 1.0).value                         # Parameter kecepatan maksimum (bisa di-set via launch/YAML)
        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic, 10)  # Publisher ke topic cmd_vel
        self.vel_msg = Twist()                       # State Twist OOP, bukan global
        self.vel_lock = threading.Lock()             # Lock OOP, bukan global
        timer_period = 0.02                          # Timer period 20ms (50Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)  # Timer untuk publish kecepatan
        self.add_on_set_parameters_callback(self.parameter_event_handler)   # Dynamic reconfigure: handler parameter event
        self.get_logger().info(f"Commander node started, publishing to {self.cmd_vel_topic} with safe_distance={self.safe_distance} and max_speed={self.max_speed}")

    def timer_callback(self):
        with self.vel_lock:                          # Pastikan akses vel_msg aman dari thread lain
            # Batasi kecepatan maksimum sebelum publish
            self.vel_msg.linear.x = max(min(self.vel_msg.linear.x, self.max_speed), -self.max_speed)
            self.publisher_.publish(self.vel_msg)    # Publish Twist ke topic cmd_vel
        self.get_logger().debug(f"Published Twist: linear.x={self.vel_msg.linear.x}, angular.z={self.vel_msg.angular.z}")

    def set_twist(self, twist):                      # Method untuk update Twist dari luar (Joy/Planner)
        with self.vel_lock:
            # Batasi kecepatan maksimum juga di sini
            twist.linear.x = max(min(twist.linear.x, self.max_speed), -self.max_speed)
            self.vel_msg = twist

    def stop(self):                                  # Method untuk emergency stop
        with self.vel_lock:
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0

    def parameter_event_handler(self, params):        # Handler untuk dynamic reconfigure parameter
        for param in params:
            if param.name == 'max_speed' and param.type_ == param.Type.DOUBLE:
                self.max_speed = param.value         # Update max_speed secara dinamis
                self.get_logger().info(f"max_speed updated dynamically: {self.max_speed}")
            if param.name == 'safe_distance' and param.type_ == param.Type.DOUBLE:
                self.safe_distance = param.value     # Update safe_distance secara dinamis
                self.get_logger().info(f"safe_distance updated dynamically: {self.safe_distance}")
            if param.name == 'cmd_vel_topic' and param.type_ == param.Type.STRING:
                # Tidak bisa update publisher topic secara dinamis tanpa re-create publisher
                self.get_logger().warn("cmd_vel_topic dynamic update not supported at runtime.")
        return rclpy.parameter.ParameterEventHandlerResult(successful=True)  # Wajib return result

class JoySubscriber(Node):                           # Node subscriber joystick
    def __init__(self, commander: Commander):
        super().__init__('joy_subscriber')           # Inisialisasi node dengan nama 'joy_subscriber'
        self.commander = commander                   # Simpan referensi ke Commander untuk update Twist
        self.joy_topic = self.declare_parameter('joy_topic', 'joy').value  # Parameter topic joy (multi-sensor support)
        self.subscription = self.create_subscription(
            Joy,
            self.joy_topic,                          # Subscribe ke topic joy (bisa diubah via parameter)
            self.listener_callback,
            10)
        self.subscription                            # Simpan subscription agar tidak di-GC
        self.get_logger().info(f"Joy_subscriber node started, listening to {self.joy_topic}")

    def listener_callback(self, data):
        try:
            if not hasattr(data, 'axes') or not hasattr(data, 'buttons'):  # Validasi data Joy
                self.get_logger().warn("Joy message tidak valid, skip log.")
                return
            if len(data.axes) < 2:                   # Validasi minimal 2 axes untuk mapping
                self.get_logger().warn("Joy axes kurang dari 2, skip log.")
                return
            twist = Twist()
            twist.linear.x = float(data.axes[1])     # Maju/mundur (stick kiri atas/bawah)
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(data.axes[0])    # Belok kiri/kanan (stick kiri kiri/kanan)
            self.commander.set_twist(twist)          # Update Twist di Commander
            self.get_logger().debug(f"Joystick: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
        except Exception as e:
            self.get_logger().error(f"Error in Joy callback: {e}")

def planner_callback(data, commander: Commander):
    # Placeholder: Tambahkan subscriber untuk autonomous planner jika diperlukan
    # Contoh: mapping data ke vel_msg jika autonomous mode aktif
    # twist = Twist()
    # twist.linear.x = data.linear.x
    # twist.angular.z = data.angular.z
    # commander.set_twist(twist)
    pass

if __name__ == '__main__':
    rclpy.init(args=None)                           # Inisialisasi ROS2

    commander = Commander()                         # Node publisher cmd_vel (OOP state)
    joy_subscriber = JoySubscriber(commander)       # Node subscriber joystick, referensi ke commander

    # Contoh: Tambahkan subscriber planner jika ingin autonomous mode
    # planner_node = Node('planner_sub')
    # planner_node.create_subscription(Twist, '/planner/cmd_vel', lambda msg: planner_callback(msg, commander), 10)
    # executor.add_node(planner_node)

    executor = rclpy.executors.MultiThreadedExecutor()  # Executor multi-thread agar node bisa jalan paralel
    executor.add_node(commander)                        # Tambahkan commander ke executor
    executor.add_node(joy_subscriber)                   # Tambahkan joy_subscriber ke executor
    # executor.add_node(planner_node)                   # Jika ada node planner

    executor_thread = threading.Thread(target=executor.spin, daemon=True)  # Jalankan executor di thread terpisah
    executor_thread.start()
    rate = commander.create_rate(2)                    # Rate loop utama (2Hz)
    try:
        while rclpy.ok():
            # Tidak perlu set vel_msg di sini, sudah di-handle oleh Commander
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()                                  # Shutdown ROS2
    executor_thread.join()                            # Tunggu thread executor selesai

# Saran lanjutan:
# - Untuk unit test, buat file test_robot_control.py di folder test/ dan gunakan pytest/launch_testing.
# - Pastikan semua script Python sudah executable: chmod +x scripts/robot_control.py
# - Jika ingin publish ke topic selain /huskybot/cmd_vel, pastikan parameter di-launch file sudah sesuai.
# - Jika ingin parameter YAML, pastikan formatnya sesuai contoh di README.
# - Untuk dynamic reconfigure, gunakan parameter event handler atau rclpy.parameter API.
# - Dokumentasikan semua parameter di README agar user mudah mengatur.