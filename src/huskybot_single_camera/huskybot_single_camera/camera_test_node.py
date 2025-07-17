#!/usr/bin/env python3
# filepath: /home/jezzy/huskybot/src/huskybot_single_camera/huskybot_single_camera/camera_test_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class CameraTestNode(Node):
    def __init__(self):
        super().__init__('camera_test_node')
        
        self.bridge = CvBridge()
        
        # ✅ TEST MULTIPLE CAMERA TOPICS
        self.test_topics = [
            '/camera_rear/image_raw',
            '/camera_front/image_raw', 
            '/camera_left/image_raw',
            '/camera_right/image_raw',
            '/image_raw',
            '/usb_cam/image_raw'
        ]
        
        # ✅ Create subscribers for all possible topics
        self.subscribers = []
        for topic in self.test_topics:
            try:
                sub = self.create_subscription(
                    Image, topic, 
                    lambda msg, t=topic: self.camera_callback(msg, t), 
                    10
                )
                self.subscribers.append(sub)
                self.get_logger().info(f"📡 Testing topic: {topic}")
            except Exception as e:
                self.get_logger().error(f"❌ Failed to subscribe to {topic}: {e}")
        
        # ✅ Status tracking
        self.message_count = {}
        self.last_message_time = {}
        
        # ✅ Status timer
        self.timer = self.create_timer(5.0, self.status_callback)
        
        self.get_logger().info("🚀 CAMERA TEST NODE STARTED - Testing all possible camera topics!")

    def camera_callback(self, msg, topic):
        """Test callback untuk semua topic"""
        try:
            # Count messages
            if topic not in self.message_count:
                self.message_count[topic] = 0
            self.message_count[topic] += 1
            self.last_message_time[topic] = time.time()
            
            # Try to convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width = cv_image.shape[:2]
            
            # Show image
            cv2.imshow(f'Camera Test - {topic}', cv_image)
            cv2.waitKey(1)
            
            if self.message_count[topic] % 30 == 0:
                self.get_logger().info(
                    f"✅ {topic}: {self.message_count[topic]} messages, "
                    f"Size: {width}x{height}, "
                    f"Encoding: {msg.encoding}"
                )
                
        except Exception as e:
            self.get_logger().error(f"❌ Error processing {topic}: {e}")

    def status_callback(self):
        """Status report"""
        self.get_logger().info("📊 CAMERA STATUS REPORT:")
        
        if not self.message_count:
            self.get_logger().warn("❌ NO CAMERA DATA RECEIVED FROM ANY TOPIC!")
            self.get_logger().info("🔧 Available topics:")
            # List available topics
            import subprocess
            try:
                result = subprocess.run(['ros2', 'topic', 'list'], 
                                      capture_output=True, text=True)
                topics = [t for t in result.stdout.split('\n') if 'image' in t or 'camera' in t]
                for topic in topics:
                    if topic.strip():
                        self.get_logger().info(f"  📡 {topic}")
            except Exception as e:
                self.get_logger().error(f"❌ Error listing topics: {e}")
        else:
            for topic, count in self.message_count.items():
                last_time = self.last_message_time.get(topic, 0)
                age = time.time() - last_time
                status = "✅ ACTIVE" if age < 2.0 else "❌ STALE"
                self.get_logger().info(f"  {status} {topic}: {count} messages, last: {age:.1f}s ago")

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = CameraTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Shutting down camera test...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if node:
            node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()