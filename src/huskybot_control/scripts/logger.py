#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

import rclpy  # Import utama ROS2 Python
from rclpy.node import Node  # Base class Node ROS2
from geometry_msgs.msg import Twist  # Import pesan Twist (untuk log perintah kecepatan)
from sensor_msgs.msg import LaserScan  # Import pesan LaserScan (untuk log data sensor)
from sensor_msgs.msg import Joy  # Import pesan Joy (untuk log joystick)
from nav_msgs.msg import Odometry  # Import pesan Odometry (untuk log odom)
from sensor_msgs.msg import Imu  # Import pesan IMU (untuk log imu)
import datetime  # Untuk timestamp log
import os  # Untuk operasi file (rotasi log)
import csv  # Untuk log ke format CSV

class LoggerNode(Node):  # Definisi class LoggerNode, turunan dari Node (OOP, sudah benar)
    def __init__(self):
        super().__init__('huskybot_logger')  # Inisialisasi node dengan nama 'huskybot_logger'
        self._cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/huskybot/cmd_vel').value  # Parameter topic cmd_vel
        self._scan_topic = self.declare_parameter('scan_topic', '/scan').value  # Parameter topic scan
        self._joy_topic = self.declare_parameter('joy_topic', '/joy').value  # Parameter topic joy (opsional)
        self._odom_topic = self.declare_parameter('odom_topic', '/odom').value  # Parameter topic odometry (opsional)
        self._imu_topic = self.declare_parameter('imu_topic', '/imu/data').value  # Parameter topic imu (opsional)
        self._log_file_path = self.declare_parameter('log_file', '').value  # Parameter path file log (opsional)
        self._log_csv = self.declare_parameter('log_csv', False).value  # Parameter: log ke CSV (True/False)
        self._log_level = self.declare_parameter('log_level', 'info').value.lower()  # Parameter log level (info/debug/warn)
        self._max_log_size = self.declare_parameter('max_log_size', 5*1024*1024).value  # Max log size (default 5MB)
        self._file_handle = None  # Handle file log
        self._csv_writer = None  # CSV writer jika log_csv True

        if self._log_file_path:
            self._open_log_file()  # Buka file log (dengan rotasi jika perlu)

        self._cmd_vel_sub = self.create_subscription(
            Twist, self._cmd_vel_topic, self._cmd_vel_callback, 10  # Subscribe ke topic cmd_vel
        )
        self._scan_sub = self.create_subscription(
            LaserScan, self._scan_topic, self._scan_callback, 10  # Subscribe ke topic scan
        )
        self._joy_sub = self.create_subscription(
            Joy, self._joy_topic, self._joy_callback, 10  # Subscribe ke topic joy (opsional)
        )
        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._odom_callback, 10  # Subscribe ke topic odometry (opsional)
        )
        self._imu_sub = self.create_subscription(
            Imu, self._imu_topic, self._imu_callback, 10  # Subscribe ke topic imu (opsional)
        )

        self.get_logger().info(
            f"Logger node started. Logging {self._cmd_vel_topic}, {self._scan_topic}, {self._joy_topic}, {self._odom_topic}, {self._imu_topic} | Level: {self._log_level} | CSV: {self._log_csv}"  # Info saat node mulai
        )

    def _open_log_file(self):
        # Rotasi file log jika sudah terlalu besar
        if os.path.exists(self._log_file_path) and os.path.getsize(self._log_file_path) > self._max_log_size:
            base, ext = os.path.splitext(self._log_file_path)
            rotated = f"{base}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}{ext}"
            os.rename(self._log_file_path, rotated)
            self.get_logger().info(f"Log file rotated: {rotated}")
        self._file_handle = open(self._log_file_path, 'a', newline='' if self._log_csv else None)
        if self._log_csv:
            self._csv_writer = csv.writer(self._file_handle)
            # Tulis header CSV
            self._csv_writer.writerow(['timestamp', 'type', 'data'])

    def _cmd_vel_callback(self, msg):  # Callback untuk pesan Twist (cmd_vel)
        try:
            log_msg = f"[{datetime.datetime.now()}] CMD_VEL: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}"
            csv_row = [datetime.datetime.now(), 'cmd_vel', f"{msg.linear.x:.2f},{msg.angular.z:.2f}"]
            self._log(log_msg, csv_row, level='info')
        except Exception as e:
            self.get_logger().error(f"Error logging cmd_vel: {e}")

    def _scan_callback(self, msg):  # Callback untuk pesan LaserScan (scan)
        try:
            if not msg.ranges or len(msg.ranges) == 0:  # Validasi: cek data LaserScan tidak kosong
                self.get_logger().warn("LaserScan data kosong, skip log.")  # Log warning jika data kosong
                return
            min_range = min(msg.ranges)
            log_msg = f"[{datetime.datetime.now()}] SCAN: min_range={min_range:.2f}"
            csv_row = [datetime.datetime.now(), 'scan', f"{min_range:.2f}"]
            self._log(log_msg, csv_row, level='debug')
        except Exception as e:
            self.get_logger().error(f"Error logging scan: {e}")

    def _joy_callback(self, msg):  # Callback untuk pesan Joy (joystick)
        try:
            if not hasattr(msg, 'axes') or not hasattr(msg, 'buttons'):  # Validasi: cek field axes dan buttons ada
                self.get_logger().warn("Joy message tidak valid, skip log.")  # Log warning jika data tidak valid
                return
            if len(msg.axes) == 0 and len(msg.buttons) == 0:  # Validasi: data Joy kosong
                self.get_logger().warn("Joy data kosong, skip log.")  # Log warning jika data kosong
                return
            log_msg = f"[{datetime.datetime.now()}] JOY: axes={list(msg.axes)}, buttons={list(msg.buttons)}"
            csv_row = [datetime.datetime.now(), 'joy', f"axes={list(msg.axes)},buttons={list(msg.buttons)}"]
            self._log(log_msg, csv_row, level='debug')
        except Exception as e:
            self.get_logger().error(f"Error logging joy: {e}")

    def _odom_callback(self, msg):  # Callback untuk pesan Odometry
        try:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            log_msg = (f"[{datetime.datetime.now()}] ODOM: pos=({pos.x:.2f},{pos.y:.2f},{pos.z:.2f}), "
                       f"ori=({ori.x:.2f},{ori.y:.2f},{ori.z:.2f},{ori.w:.2f})")
            csv_row = [datetime.datetime.now(), 'odom', f"{pos.x:.2f},{pos.y:.2f},{pos.z:.2f},{ori.x:.2f},{ori.y:.2f},{ori.z:.2f},{ori.w:.2f}"]
            self._log(log_msg, csv_row, level='debug')
        except Exception as e:
            self.get_logger().error(f"Error logging odometry: {e}")

    def _imu_callback(self, msg):  # Callback untuk pesan IMU
        try:
            ori = msg.orientation
            ang = msg.angular_velocity
            lin = msg.linear_acceleration
            log_msg = (f"[{datetime.datetime.now()}] IMU: ori=({ori.x:.2f},{ori.y:.2f},{ori.z:.2f},{ori.w:.2f}), "
                       f"ang=({ang.x:.2f},{ang.y:.2f},{ang.z:.2f}), lin=({lin.x:.2f},{lin.y:.2f},{lin.z:.2f})")
            csv_row = [datetime.datetime.now(), 'imu', f"{ori.x:.2f},{ori.y:.2f},{ori.z:.2f},{ori.w:.2f},{ang.x:.2f},{ang.y:.2f},{ang.z:.2f},{lin.x:.2f},{lin.y:.2f},{lin.z:.2f}"]
            self._log(log_msg, csv_row, level='debug')
        except Exception as e:
            self.get_logger().error(f"Error logging imu: {e}")

    def _log(self, log_msg, csv_row=None, level='info'):
        # Log ke terminal sesuai level
        try:
            if self._should_log(level):
                if level == 'debug':
                    self.get_logger().debug(log_msg)
                elif level == 'warn':
                    self.get_logger().warn(log_msg)
                else:
                    self.get_logger().info(log_msg)
            # Log ke file jika ada
            if self._file_handle:
                if self._log_csv and csv_row:
                    self._csv_writer.writerow(csv_row)
                else:
                    self._file_handle.write(log_msg + '\n')
                self._file_handle.flush()  # Flush agar data tidak hilang jika node crash
        except Exception as e:
            self.get_logger().error(f"Error writing log: {e}")

    def _should_log(self, level):
        # Cek apakah level log ini boleh ditampilkan sesuai setting user
        levels = {'debug': 0, 'info': 1, 'warn': 2}
        return levels.get(level, 1) >= levels.get(self._log_level, 1)

    def destroy_node(self):  # Override destroy_node untuk cleanup file handle
        if self._file_handle:
            self._file_handle.close()  # Tutup file log jika ada
        super().destroy_node()  # Panggil destroy_node parent

def main(args=None):  # Fungsi utama ROS2 Python
    rclpy.init(args=args)  # Inisialisasi ROS2
    node = LoggerNode()  # Buat instance LoggerNode
    try:
        rclpy.spin(node)  # Spin node sampai shutdown
    except KeyboardInterrupt:
        pass
    node.destroy_node()  # Cleanup node
    rclpy.shutdown()  # Shutdown ROS2

if __name__ == '__main__':  # Jika dijalankan langsung
    main()  # Panggil main