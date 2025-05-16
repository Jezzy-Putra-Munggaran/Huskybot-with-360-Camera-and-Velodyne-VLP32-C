#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

import sys                                           # Import sys untuk exit/error
import rclpy                                         # Import utama ROS2 Python
from rclpy.node import Node                          # Base class Node ROS2
from geometry_msgs.msg import Twist                  # Import pesan Twist (untuk cmd_vel)
from sensor_msgs.msg import Joy                      # Import pesan Joy (untuk joystick)
import threading                                     # Import threading untuk multi-thread executor dan lock
import os                                            # Import os untuk path log
import time                                          # Import time untuk validasi topic
import logging                                       # Import logging untuk log ke file

# ===================== LOGGING TO FILE (OPSIONAL) =====================
def setup_file_logger(log_path="~/huskybot_robot_control.log"):
    log_path = os.path.expanduser(log_path)
    logger = logging.getLogger("robot_control_file_logger")
    logger.setLevel(logging.INFO)
    if not logger.hasHandlers():
        fh = logging.FileHandler(log_path)
        fh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s: %(message)s'))
        logger.addHandler(fh)
    return logger

file_logger = setup_file_logger()

def log_to_file(msg, level='info'):
    if file_logger:
        if level == 'error':
            file_logger.error(msg)
        elif level == 'warn':
            file_logger.warning(msg)
        else:
            file_logger.info(msg)

# ===================== COMMANDER NODE =====================
class Commander(Node):                               # Node publisher utama ke /huskybot/cmd_vel
    def __init__(self):
        super().__init__('commander')                # Inisialisasi node dengan nama 'commander'
        try:
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
            log_to_file(f"Commander node started, publishing to {self.cmd_vel_topic} with safe_distance={self.safe_distance} and max_speed={self.max_speed}")
        except Exception as e:
            self.get_logger().error(f"Error initializing Commander: {e}")
            log_to_file(f"Error initializing Commander: {e}", level='error')
            sys.exit(10)

    def timer_callback(self):
        try:
            with self.vel_lock:                          # Pastikan akses vel_msg aman dari thread lain
                # Batasi kecepatan maksimum sebelum publish
                self.vel_msg.linear.x = max(min(self.vel_msg.linear.x, self.max_speed), -self.max_speed)
                self.publisher_.publish(self.vel_msg)    # Publish Twist ke topic cmd_vel
            self.get_logger().debug(f"Published Twist: linear.x={self.vel_msg.linear.x}, angular.z={self.vel_msg.angular.z}")
            log_to_file(f"Published Twist: linear.x={self.vel_msg.linear.x}, angular.z={self.vel_msg.angular.z}", level='debug')
        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {e}")
            log_to_file(f"Error in timer_callback: {e}", level='error')

    def set_twist(self, twist):                      # Method untuk update Twist dari luar (Joy/Planner)
        try:
            with self.vel_lock:
                # Batasi kecepatan maksimum juga di sini
                twist.linear.x = max(min(twist.linear.x, self.max_speed), -self.max_speed)
                self.vel_msg = twist
            self.get_logger().debug(f"Set Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
            log_to_file(f"Set Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}", level='debug')
        except Exception as e:
            self.get_logger().error(f"Error in set_twist: {e}")
            log_to_file(f"Error in set_twist: {e}", level='error')

    def stop(self):                                  # Method untuk emergency stop
        try:
            with self.vel_lock:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = 0.0
            self.get_logger().warn("Emergency stop triggered!")
            log_to_file("Emergency stop triggered!", level='warn')
        except Exception as e:
            self.get_logger().error(f"Error in stop: {e}")
            log_to_file(f"Error in stop: {e}", level='error')

    def parameter_event_handler(self, params):        # Handler untuk dynamic reconfigure parameter
        try:
            for param in params:
                if param.name == 'max_speed' and param.type_ == param.Type.DOUBLE:
                    self.max_speed = param.value         # Update max_speed secara dinamis
                    self.get_logger().info(f"max_speed updated dynamically: {self.max_speed}")
                    log_to_file(f"max_speed updated dynamically: {self.max_speed}")
                if param.name == 'safe_distance' and param.type_ == param.Type.DOUBLE:
                    self.safe_distance = param.value     # Update safe_distance secara dinamis
                    self.get_logger().info(f"safe_distance updated dynamically: {self.safe_distance}")
                    log_to_file(f"safe_distance updated dynamically: {self.safe_distance}")
                if param.name == 'cmd_vel_topic' and param.type_ == param.Type.STRING:
                    # Tidak bisa update publisher topic secara dinamis tanpa re-create publisher
                    self.get_logger().warn("cmd_vel_topic dynamic update not supported at runtime.")
                    log_to_file("cmd_vel_topic dynamic update not supported at runtime.", level='warn')
            return rclpy.parameter.ParameterEventHandlerResult(successful=True)  # Wajib return result
        except Exception as e:
            self.get_logger().error(f"Error in parameter_event_handler: {e}")
            log_to_file(f"Error in parameter_event_handler: {e}", level='error')
            return rclpy.parameter.ParameterEventHandlerResult(successful=False)

# ===================== JOY SUBSCRIBER NODE =====================
class JoySubscriber(Node):                           # Node subscriber joystick
    def __init__(self, commander: Commander):
        super().__init__('joy_subscriber')           # Inisialisasi node dengan nama 'joy_subscriber'
        try:
            self.commander = commander                   # Simpan referensi ke Commander untuk update Twist
            self.joy_topic = self.declare_parameter('joy_topic', 'joy').value  # Parameter topic joy (multi-sensor support)
            self.subscription = self.create_subscription(
                Joy,
                self.joy_topic,                          # Subscribe ke topic joy (bisa diubah via parameter)
                self.listener_callback,
                10)
            self.subscription                            # Simpan subscription agar tidak di-GC
            self.get_logger().info(f"Joy_subscriber node started, listening to {self.joy_topic}")
            log_to_file(f"Joy_subscriber node started, listening to {self.joy_topic}")
        except Exception as e:
            self.get_logger().error(f"Error initializing JoySubscriber: {e}")
            log_to_file(f"Error initializing JoySubscriber: {e}", level='error')
            sys.exit(11)

    def listener_callback(self, data):
        try:
            if not hasattr(data, 'axes') or not hasattr(data, 'buttons'):  # Validasi data Joy
                self.get_logger().warn("Joy message tidak valid, skip log.")
                log_to_file("Joy message tidak valid, skip log.", level='warn')
                return
            if len(data.axes) < 2:                   # Validasi minimal 2 axes untuk mapping
                self.get_logger().warn("Joy axes kurang dari 2, skip log.")
                log_to_file("Joy axes kurang dari 2, skip log.", level='warn')
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
            log_to_file(f"Joystick: linear.x={twist.linear.x}, angular.z={twist.angular.z}", level='debug')
        except Exception as e:
            self.get_logger().error(f"Error in Joy callback: {e}")
            log_to_file(f"Error in Joy callback: {e}", level='error')

# ===================== PLANNER CALLBACK (OPSIONAL) =====================
def planner_callback(data, commander: Commander):
    # Placeholder: Tambahkan subscriber untuk autonomous planner jika diperlukan
    pass

# ===================== VALIDASI DEPENDENCY & TOPIC (OPSIONAL) =====================
def validate_dependencies():
    # Cek dependency ROS2
    try:
        import rclpy
        import geometry_msgs.msg
        import sensor_msgs.msg
    except ImportError as e:
        print(f"[ERROR] Dependency ROS2 tidak ditemukan: {e}", file=sys.stderr)
        log_to_file(f"Dependency ROS2 tidak ditemukan: {e}", level='error')
        sys.exit(2)

def validate_topic_active(node, topic, timeout=5.0):
    # Cek apakah topic aktif (ada publisher)
    start = time.time()
    while time.time() - start < timeout:
        info = node.get_topic_names_and_types()
        if any(t[0] == topic for t in info):
            node.get_logger().info(f"Topic aktif: {topic}")
            log_to_file(f"Topic aktif: {topic}")
            return True
        time.sleep(0.5)
    node.get_logger().warn(f"Topic {topic} tidak aktif setelah {timeout} detik.")
    log_to_file(f"Topic {topic} tidak aktif setelah {timeout} detik.", level='warn')
    return False

# ===================== MAIN =====================
if __name__ == '__main__':
    validate_dependencies()
    rclpy.init(args=None)                           # Inisialisasi ROS2

    commander = Commander()                         # Node publisher cmd_vel (OOP state)
    joy_subscriber = JoySubscriber(commander)       # Node subscriber joystick, referensi ke commander

    # Validasi topic Joy aktif (opsional, bisa dihapus jika tidak perlu)
    validate_topic_active(joy_subscriber, joy_subscriber.joy_topic)

    executor = rclpy.executors.MultiThreadedExecutor()  # Executor multi-thread agar node bisa jalan paralel
    executor.add_node(commander)                        # Tambahkan commander ke executor
    executor.add_node(joy_subscriber)                   # Tambahkan joy_subscriber ke executor

    executor_thread = threading.Thread(target=executor.spin, daemon=True)  # Jalankan executor di thread terpisah
    executor_thread.start()
    rate = commander.create_rate(2)                    # Rate loop utama (2Hz)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        commander.get_logger().info("KeyboardInterrupt, shutting down robot_control node.")
        log_to_file("KeyboardInterrupt, shutting down robot_control node.", level='warn')
    except Exception as e:
        commander.get_logger().error(f"Exception utama: {e}")
        log_to_file(f"Exception utama: {e}", level='error')
    finally:
        rclpy.shutdown()                                  # Shutdown ROS2
        executor_thread.join()                            # Tunggu thread executor selesai
        commander.get_logger().info("robot_control node shutdown complete.")
        log_to_file("robot_control node shutdown complete.")

# Saran lanjutan:
# - Untuk unit test, buat file test_robot_control.py di folder test/ dan gunakan pytest/launch_testing.
# - Pastikan semua script Python sudah executable: chmod +x scripts/robot_control.py
# - Jika ingin publish ke topic selain /huskybot/cmd_vel, pastikan parameter di-launch file sudah sesuai.
# - Jika ingin parameter YAML, pastikan formatnya sesuai contoh di README.
# - Untuk dynamic reconfigure, gunakan parameter event handler atau rclpy.parameter API.
# - Dokumentasikan semua parameter di README agar user mudah mengatur.