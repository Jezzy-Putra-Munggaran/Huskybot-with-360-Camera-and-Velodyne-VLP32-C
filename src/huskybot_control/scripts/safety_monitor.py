#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  # Import utama ROS2 Python
from rclpy.node import Node  # Base class Node ROS2
from rclpy.parameter import Parameter  # Untuk dynamic reconfigure parameter
from std_msgs.msg import Bool, Float32  # Import pesan Bool untuk safety_stop, Float32 untuk min_range dan direction
from sensor_msgs.msg import LaserScan  # Import pesan LaserScan (untuk safety monitor)
import threading  # Import threading untuk lock (thread-safe)
import math  # Untuk validasi nilai LaserScan (inf/nan)

class SafetyMonitor(Node):  # Definisi class SafetyMonitor, turunan dari Node (OOP, best practice ROS2)
    def __init__(self):
        super().__init__('safety_monitor')  # Inisialisasi node dengan nama 'safety_monitor'
        self._safe_distance = self.declare_parameter('safe_distance', 0.5).value  # Parameter jarak aman (bisa di-set via launch/YAML)
        self._scan_topic = self.declare_parameter('scan_topic', '/scan').value  # Parameter topic scan (bisa di-remap via launch/YAML)
        self._safety_stop = False  # Status safety stop (True jika obstacle terlalu dekat)
        self._lock = threading.Lock()  # Lock untuk thread-safe akses _safety_stop
        self._subscription = self.create_subscription(
            LaserScan,
            self._scan_topic,  # Subscribe ke topic scan (default /scan, bisa diubah via parameter)
            self._scan_callback,
            10  # Queue size
        )
        self._safety_pub = self.create_publisher(Bool, 'safety_stop', 10)  # Publisher status safety_stop ke topic /safety_stop
        self._min_range_pub = self.create_publisher(Float32, 'min_range', 10)  # Publisher min_range ke topic /min_range
        self._direction_pub = self.create_publisher(Float32, 'obstacle_direction', 10)  # Publisher arah obstacle terdekat ke topic /obstacle_direction
        self.add_on_set_parameters_callback(self._parameter_event_handler)  # Dynamic reconfigure: handler parameter event
        self.get_logger().info(
            f"SafetyMonitor node started, listening to {self._scan_topic} with safe_distance={self._safe_distance}"
        )  # Logging info saat node mulai

    @property
    def safety_stop(self):  # Property untuk akses status safety_stop secara thread-safe
        with self._lock:
            return self._safety_stop

    def _scan_callback(self, msg):  # Callback untuk pesan LaserScan
        # Filter nilai inf/nan dari ranges agar robust
        valid_ranges = [(i, r) for i, r in enumerate(msg.ranges) if not math.isinf(r) and not math.isnan(r) and r > 0.0]  # Ambil index dan nilai range yang valid
        if valid_ranges:
            min_idx, min_range = min(valid_ranges, key=lambda x: x[1])  # Ambil index dan jarak terdekat valid
            direction = msg.angle_min + min_idx * msg.angle_increment  # Hitung arah obstacle terdekat (radian)
        else:
            min_range = float('inf')  # Jika tidak ada data valid, set inf
            direction = float('nan')  # Jika tidak ada data valid, set nan
        self.get_logger().debug(f"LaserScan min_range={min_range:.2f}, direction={direction:.2f}")  # Debug log jarak & arah terdekat
        self._min_range_pub.publish(Float32(data=min_range))  # Publish min_range ke topic /min_range
        self._direction_pub.publish(Float32(data=direction))  # Publish arah obstacle terdekat ke topic /obstacle_direction
        with self._lock:
            if min_range < self._safe_distance:  # Jika obstacle terlalu dekat
                if not self._safety_stop:
                    self.get_logger().warn(
                        f"Obstacle too close! ({min_range:.2f} m < {self._safe_distance} m, direction={direction:.2f} rad). Stopping robot."
                    )  # Warning hanya saat status berubah
                self._safety_stop = True  # Aktifkan safety stop
            else:
                if self._safety_stop:
                    self.get_logger().info("Obstacle cleared. Resuming control.")  # Info jika obstacle sudah hilang
                self._safety_stop = False  # Nonaktifkan safety stop
            self._safety_pub.publish(Bool(data=self._safety_stop))  # Publish status ke topic /safety_stop

    def _parameter_event_handler(self, params):  # Handler untuk dynamic reconfigure parameter
        for param in params:
            if param.name == 'safe_distance' and param.type_ == Parameter.Type.DOUBLE:
                self._safe_distance = param.value  # Update safe_distance secara dinamis
                self.get_logger().info(f"safe_distance updated dynamically: {self._safe_distance}")
            if param.name == 'scan_topic' and param.type_ == Parameter.Type.STRING:
                self.get_logger().warn("scan_topic dynamic update not supported at runtime. Restart node to apply.")  # Tidak bisa update subscription runtime
        return rclpy.parameter.ParameterEventHandlerResult(successful=True)  # Wajib return result

def main(args=None):  # Fungsi utama ROS2 Python
    rclpy.init(args=args)  # Inisialisasi ROS2
    node = SafetyMonitor()  # Buat instance SafetyMonitor
    try:
        rclpy.spin(node)  # Spin node sampai shutdown
    except KeyboardInterrupt:
        pass
    node.destroy_node()  # Cleanup node
    rclpy.shutdown()  # Shutdown ROS2

if __name__ == '__main__':  # Jika dijalankan langsung
    main()  # Panggil main