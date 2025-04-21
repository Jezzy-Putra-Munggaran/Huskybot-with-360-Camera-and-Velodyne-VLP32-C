#!/usr/bin/python3
import os, sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, String, Int32
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
bridge = CvBridge()

import threading

vel_msg = Twist()
vel_lock = threading.Lock()  # Tambahkan lock untuk mencegah race condition

class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        self.publisher_ = self.create_publisher(Twist, '/yolobot/cmd_vel', 10)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global vel_msg
        with vel_lock:
            self.publisher_.publish(vel_msg)

class Joy_subscriber(Node):

    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, data):
        global vel_msg
        with vel_lock:
            # Gunakan mapping joystick yang benar untuk Twist
            # Misal: axes[1] untuk maju-mundur, axes[0] untuk belok kiri-kanan
            vel_msg.linear.x = float(data.axes[1])  # Forward/backward
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = float(data.axes[0])  # Turn

if __name__ == '__main__':
    rclpy.init(args=None)
    
    commander = Commander()
    joy_subscriber = Joy_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(joy_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    executor_thread.join()