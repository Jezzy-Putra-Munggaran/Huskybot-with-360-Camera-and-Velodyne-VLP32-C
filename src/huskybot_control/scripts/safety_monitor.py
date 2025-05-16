#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  # Import utama ROS2 Python
from rclpy.node import Node  # Base class Node ROS2
from rclpy.parameter import Parameter  # Untuk dynamic reconfigure parameter
from std_msgs.msg import Bool, Float32  # Import pesan Bool untuk safety_stop, Float32 untuk min_range dan direction
from sensor_msgs.msg import LaserScan  # Import pesan LaserScan (untuk safety monitor)
import threading  # Import threading untuk lock (thread-safe)
import math  # Untuk validasi nilai LaserScan (inf/nan)
import os
import logging
import time

# ===================== LOGGING TO FILE (OPSIONAL) =====================
def setup_file_logger(log_path="~/huskybot_safety_monitor.log"):
    log_path = os.path.expanduser(log_path)
    logger = logging.getLogger("safety_monitor_file_logger")
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

class SafetyMonitor(Node):  # Definisi class SafetyMonitor, turunan dari Node (OOP, best practice ROS2)
    def __init__(self):
        super().__init__('safety_monitor')
        try:
            self._safe_distance = self.declare_parameter('safe_distance', 0.5).value
            self._scan_topic = self.declare_parameter('scan_topic', '/scan').value
            self._safety_stop = False
            self._lock = threading.Lock()
            self._subscription = self.create_subscription(
                LaserScan,
                self._scan_topic,
                self._scan_callback,
                10
            )
            self._safety_pub = self.create_publisher(Bool, 'safety_stop', 10)
            self._min_range_pub = self.create_publisher(Float32, 'min_range', 10)
            self._direction_pub = self.create_publisher(Float32, 'obstacle_direction', 10)
            self.add_on_set_parameters_callback(self._parameter_event_handler)
            self.get_logger().info(
                f"SafetyMonitor node started, listening to {self._scan_topic} with safe_distance={self._safe_distance}"
            )
            log_to_file(f"SafetyMonitor node started, listening to {self._scan_topic} with safe_distance={self._safe_distance}")
            # Health check: cek apakah topic aktif (ada publisher)
            self._validate_topic_active(self._scan_topic)
        except Exception as e:
            self.get_logger().error(f"Error initializing SafetyMonitor: {e}")
            log_to_file(f"Error initializing SafetyMonitor: {e}", level='error')
            raise

    @property
    def safety_stop(self):
        with self._lock:
            return self._safety_stop

    def _scan_callback(self, msg):
        try:
            valid_ranges = [(i, r) for i, r in enumerate(msg.ranges) if not math.isinf(r) and not math.isnan(r) and r > 0.0]
            if valid_ranges:
                min_idx, min_range = min(valid_ranges, key=lambda x: x[1])
                direction = msg.angle_min + min_idx * msg.angle_increment
            else:
                min_range = float('inf')
                direction = float('nan')
            self.get_logger().debug(f"LaserScan min_range={min_range:.2f}, direction={direction:.2f}")
            log_to_file(f"LaserScan min_range={min_range:.2f}, direction={direction:.2f}", level='debug')
            self._min_range_pub.publish(Float32(data=min_range))
            self._direction_pub.publish(Float32(data=direction))
            with self._lock:
                if min_range < self._safe_distance:
                    if not self._safety_stop:
                        warn_msg = f"Obstacle too close! ({min_range:.2f} m < {self._safe_distance} m, direction={direction:.2f} rad). Stopping robot."
                        self.get_logger().warn(warn_msg)
                        log_to_file(warn_msg, level='warn')
                    self._safety_stop = True
                else:
                    if self._safety_stop:
                        info_msg = "Obstacle cleared. Resuming control."
                        self.get_logger().info(info_msg)
                        log_to_file(info_msg)
                    self._safety_stop = False
                self._safety_pub.publish(Bool(data=self._safety_stop))
        except Exception as e:
            self.get_logger().error(f"Error in _scan_callback: {e}")
            log_to_file(f"Error in _scan_callback: {e}", level='error')

    def _parameter_event_handler(self, params):
        try:
            for param in params:
                if param.name == 'safe_distance' and param.type_ == Parameter.Type.DOUBLE:
                    self._safe_distance = param.value
                    msg = f"safe_distance updated dynamically: {self._safe_distance}"
                    self.get_logger().info(msg)
                    log_to_file(msg)
                if param.name == 'scan_topic' and param.type_ == Parameter.Type.STRING:
                    warn_msg = "scan_topic dynamic update not supported at runtime. Restart node to apply."
                    self.get_logger().warn(warn_msg)
                    log_to_file(warn_msg, level='warn')
            return rclpy.parameter.ParameterEventHandlerResult(successful=True)
        except Exception as e:
            self.get_logger().error(f"Error in _parameter_event_handler: {e}")
            log_to_file(f"Error in _parameter_event_handler: {e}", level='error')
            return rclpy.parameter.ParameterEventHandlerResult(successful=False)

    def _validate_topic_active(self, topic, timeout=5.0):
        # Cek apakah topic aktif (ada publisher)
        try:
            start = time.time()
            while time.time() - start < timeout:
                info = self.get_topic_names_and_types()
                if any(t[0] == topic for t in info):
                    self.get_logger().info(f"Topic aktif: {topic}")
                    log_to_file(f"Topic aktif: {topic}")
                    return True
                time.sleep(0.5)
            warn_msg = f"Topic {topic} tidak aktif setelah {timeout} detik."
            self.get_logger().warn(warn_msg)
            log_to_file(warn_msg, level='warn')
            return False
        except Exception as e:
            self.get_logger().error(f"Error in _validate_topic_active: {e}")
            log_to_file(f"Error in _validate_topic_active: {e}", level='error')
            return False

def main(args=None):
    try:
        rclpy.init(args=args)
        node = SafetyMonitor()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("KeyboardInterrupt, shutting down safety_monitor node.")
            log_to_file("KeyboardInterrupt, shutting down safety_monitor node.", level='warn')
        except Exception as e:
            node.get_logger().error(f"Exception utama: {e}")
            log_to_file(f"Exception utama: {e}", level='error')
        finally:
            node.destroy_node()
            rclpy.shutdown()
            node.get_logger().info("safety_monitor node shutdown complete.")
            log_to_file("safety_monitor node shutdown complete.")
    except Exception as e:
        print(f"[FATAL] Exception di main(): {e}", file=sys.stderr)
        log_to_file(f"[FATAL] Exception di main(): {e}", level='error')
        sys.exit(99)

if __name__ == '__main__':
    main()