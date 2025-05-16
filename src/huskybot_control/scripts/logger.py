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
import sys

class LoggerNode(Node):  # Definisi class LoggerNode, turunan dari Node (OOP, sudah benar)
    def __init__(self):
        super().__init__('huskybot_logger')
        try:
            self._cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/huskybot/cmd_vel').value
            self._scan_topic = self.declare_parameter('scan_topic', '/scan').value
            self._joy_topic = self.declare_parameter('joy_topic', '/joy').value
            self._odom_topic = self.declare_parameter('odom_topic', '/odom').value
            self._imu_topic = self.declare_parameter('imu_topic', '/imu/data').value
            self._log_file_path = self.declare_parameter('log_file', '').value
            self._log_csv = self.declare_parameter('log_csv', False).value
            self._log_level = self.declare_parameter('log_level', 'info').value.lower()
            self._max_log_size = self.declare_parameter('max_log_size', 5*1024*1024).value
            self._file_handle = None
            self._csv_writer = None

            # Validasi file log jika diaktifkan
            if self._log_file_path:
                try:
                    self._open_log_file()
                except Exception as e:
                    self.get_logger().error(f"Error opening log file: {e}")
                    print(f"[ERROR] Error opening log file: {e}", file=sys.stderr)
                    sys.exit(2)

            # Validasi dependency topic
            self._cmd_vel_sub = self.create_subscription(
                Twist, self._cmd_vel_topic, self._cmd_vel_callback, 10
            )
            self._scan_sub = self.create_subscription(
                LaserScan, self._scan_topic, self._scan_callback, 10
            )
            self._joy_sub = self.create_subscription(
                Joy, self._joy_topic, self._joy_callback, 10
            )
            self._odom_sub = self.create_subscription(
                Odometry, self._odom_topic, self._odom_callback, 10
            )
            self._imu_sub = self.create_subscription(
                Imu, self._imu_topic, self._imu_callback, 10
            )

            self.get_logger().info(
                f"Logger node started. Logging {self._cmd_vel_topic}, {self._scan_topic}, {self._joy_topic}, {self._odom_topic}, {self._imu_topic} | Level: {self._log_level} | CSV: {self._log_csv}"
            )
            self._validate_topic_active(self._cmd_vel_topic)
            self._validate_topic_active(self._scan_topic)
        except Exception as e:
            self.get_logger().error(f"Error initializing LoggerNode: {e}")
            print(f"[ERROR] Error initializing LoggerNode: {e}", file=sys.stderr)
            sys.exit(10)

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
            # Tulis header CSV jika file baru
            if os.stat(self._log_file_path).st_size == 0:
                self._csv_writer.writerow(['timestamp', 'type', 'data'])

    def _cmd_vel_callback(self, msg):
        try:
            log_msg = f"[{datetime.datetime.now()}] CMD_VEL: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}"
            csv_row = [datetime.datetime.now(), 'cmd_vel', f"{msg.linear.x:.2f},{msg.angular.z:.2f}"]
            self._log(log_msg, csv_row, level='info')
        except Exception as e:
            self.get_logger().error(f"Error logging cmd_vel: {e}")

    def _scan_callback(self, msg):
        try:
            if not msg.ranges or len(msg.ranges) == 0:
                self.get_logger().warn("LaserScan data kosong, skip log.")
                return
            min_range = min(msg.ranges)
            log_msg = f"[{datetime.datetime.now()}] SCAN: min_range={min_range:.2f}"
            csv_row = [datetime.datetime.now(), 'scan', f"{min_range:.2f}"]
            self._log(log_msg, csv_row, level='debug')
        except Exception as e:
            self.get_logger().error(f"Error logging scan: {e}")

    def _joy_callback(self, msg):
        try:
            if not hasattr(msg, 'axes') or not hasattr(msg, 'buttons'):
                self.get_logger().warn("Joy message tidak valid, skip log.")
                return
            if len(msg.axes) == 0 and len(msg.buttons) == 0:
                self.get_logger().warn("Joy data kosong, skip log.")
                return
            log_msg = f"[{datetime.datetime.now()}] JOY: axes={list(msg.axes)}, buttons={list(msg.buttons)}"
            csv_row = [datetime.datetime.now(), 'joy', f"axes={list(msg.axes)},buttons={list(msg.buttons)}"]
            self._log(log_msg, csv_row, level='debug')
        except Exception as e:
            self.get_logger().error(f"Error logging joy: {e}")

    def _odom_callback(self, msg):
        try:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            log_msg = (f"[{datetime.datetime.now()}] ODOM: pos=({pos.x:.2f},{pos.y:.2f},{pos.z:.2f}), "
                       f"ori=({ori.x:.2f},{ori.y:.2f},{ori.z:.2f},{ori.w:.2f})")
            csv_row = [datetime.datetime.now(), 'odom', f"{pos.x:.2f},{pos.y:.2f},{pos.z:.2f},{ori.x:.2f},{ori.y:.2f},{ori.z:.2f},{ori.w:.2f}"]
            self._log(log_msg, csv_row, level='debug')
        except Exception as e:
            self.get_logger().error(f"Error logging odometry: {e}")

    def _imu_callback(self, msg):
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
        try:
            if self._should_log(level):
                if level == 'debug':
                    self.get_logger().debug(log_msg)
                elif level == 'warn':
                    self.get_logger().warn(log_msg)
                else:
                    self.get_logger().info(log_msg)
            if self._file_handle:
                if self._log_csv and csv_row:
                    self._csv_writer.writerow(csv_row)
                else:
                    self._file_handle.write(log_msg + '\n')
                self._file_handle.flush()
        except Exception as e:
            self.get_logger().error(f"Error writing log: {e}")

    def _should_log(self, level):
        levels = {'debug': 0, 'info': 1, 'warn': 2}
        return levels.get(level, 1) >= levels.get(self._log_level, 1)

    def _validate_topic_active(self, topic, timeout=5.0):
        try:
            start = datetime.datetime.now()
            while (datetime.datetime.now() - start).total_seconds() < timeout:
                info = self.get_topic_names_and_types()
                if any(t[0] == topic for t in info):
                    self.get_logger().info(f"Topic aktif: {topic}")
                    return True
                rclpy.spin_once(self, timeout_sec=0.5)
            self.get_logger().warn(f"Topic {topic} tidak aktif setelah {timeout} detik.")
            return False
        except Exception as e:
            self.get_logger().error(f"Error in _validate_topic_active: {e}")
            return False

    def destroy_node(self):
        if self._file_handle:
            self._file_handle.close()
        super().destroy_node()

def main(args=None):
    try:
        rclpy.init(args=args)
        node = LoggerNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("KeyboardInterrupt, shutting down logger node.")
        except Exception as e:
            node.get_logger().error(f"Exception utama: {e}")
        finally:
            node.destroy_node()
            rclpy.shutdown()
            node.get_logger().info("logger node shutdown complete.")
    except Exception as e:
        print(f"[FATAL] Exception di main(): {e}", file=sys.stderr)
        sys.exit(99)

if __name__ == '__main__':
    main()