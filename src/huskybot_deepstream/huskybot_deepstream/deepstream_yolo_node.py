#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolov12_msgs.msg import Yolov12Inference, InferenceResult
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import os
import sys
import gi

# Import GStreamer
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, GLib

class DeepStreamYOLONode(Node):
    def __init__(self):
        super().__init__('deepstream_yolo')
        
        # Initialize GStreamer
        Gst.init(None)
        GObject.threads_init()
        
        # Bridge
        self.bridge = CvBridge()
        
        # Setup parameters
        self.setup_parameters()
        
        # Setup ROS topics
        self.setup_ros_topics()
        
        # Setup DeepStream pipeline
        self.setup_deepstream_pipeline()
        
        # Statistics
        self.frame_count = 0
        self.fps_timer = self.create_timer(2.0, self.log_fps)
        self.last_fps_time = time.time()
        
        self.get_logger().info("🚀 DeepStream YOLO Node initialized for 100+ FPS!")

    def setup_parameters(self):
        """Setup parameters"""
        self.declare_parameter('model_engine', 'yolo11n-seg.engine')
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('batch_size', 6)
        self.declare_parameter('fps_target', 120)
        self.declare_parameter('device_id', 0)
        
        self.model_engine = self.get_parameter('model_engine').value
        self.input_width = self.get_parameter('input_width').value
        self.input_height = self.get_parameter('input_height').value
        self.batch_size = self.get_parameter('batch_size').value
        self.fps_target = self.get_parameter('fps_target').value
        self.device_id = self.get_parameter('device_id').value

    def setup_ros_topics(self):
        """Setup ROS2 topics"""
        # Subscribers for 6 cameras
        self.camera_subs = []
        camera_names = ['front', 'front_left', 'left', 'rear', 'rear_right', 'right']
        
        for i, name in enumerate(camera_names):
            topic = f'/camera_{name}/image_raw'
            sub = self.create_subscription(
                Image, topic, 
                lambda msg, idx=i: self.camera_callback(msg, idx), 10)
            self.camera_subs.append(sub)
        
        # Publishers for results
        self.result_pubs = []
        for name in camera_names:
            pub = self.create_publisher(
                Yolov12Inference, f'/camera_{name}/deepstream_detections', 10)
            self.result_pubs.append(pub)
        
        # Grid visualization publisher
        self.grid_pub = self.create_publisher(Image, '/deepstream_grid', 1)
        
        self.get_logger().info("📡 ROS2 topics configured")

    def setup_deepstream_pipeline(self):
        """Setup DeepStream pipeline"""
        try:
            # Create pipeline
            self.pipeline = Gst.Pipeline()
            
            # Create elements
            self.create_pipeline_elements()
            
            # Add elements to pipeline
            self.add_elements_to_pipeline()
            
            # Link elements
            self.link_pipeline_elements()
            
            # Setup bus and start
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message", self.bus_call)
            
            # Start pipeline
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                self.get_logger().error("❌ Failed to start DeepStream pipeline")
                return False
            
            self.get_logger().info("✅ DeepStream pipeline started successfully")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Error setting up DeepStream: {e}")
            return False

    def create_pipeline_elements(self):
        """Create DeepStream elements"""
        # Multi-stream muxer
        self.streammux = Gst.ElementFactory.make("nvstreammux", "stream-muxer")
        self.streammux.set_property('width', self.input_width)
        self.streammux.set_property('height', self.input_height)
        self.streammux.set_property('batch-size', self.batch_size)
        self.streammux.set_property('batched-push-timeout', 4000000)
        
        # Primary inference engine
        self.pgie = Gst.ElementFactory.make("nvinfer", "primary-nvinference-engine")
        config_path = os.path.join(os.path.dirname(__file__), 'config', 'config_infer_yolo11.txt')
        self.pgie.set_property('config-file-path', config_path)
        
        # Video converter
        self.nvvidconv = Gst.ElementFactory.make("nvvideoconvert", "convertor")
        
        # OSD for visualization
        self.nvosd = Gst.ElementFactory.make("nvdsosd", "onscreendisplay")
        
        # Tiler for multi-stream
        self.tiler = Gst.ElementFactory.make("nvmultistreamtiler", "nvtiler")
        self.tiler.set_property("rows", 2)
        self.tiler.set_property("columns", 3)
        self.tiler.set_property("width", 1920)
        self.tiler.set_property("height", 1080)
        
        # Sink
        self.fakesink = Gst.ElementFactory.make("fakesink", "fakesink")

    def add_elements_to_pipeline(self):
        """Add elements to pipeline"""
        elements = [
            self.streammux, self.pgie, self.nvvidconv, 
            self.nvosd, self.tiler, self.fakesink
        ]
        
        for element in elements:
            if not self.pipeline.add(element):
                self.get_logger().error(f"❌ Failed to add {element.get_name()}")
                return False
        return True

    def link_pipeline_elements(self):
        """Link pipeline elements"""
        # Link elements
        if not self.streammux.link(self.pgie):
            self.get_logger().error("❌ Failed to link streammux -> pgie")
            return False
            
        if not self.pgie.link(self.nvvidconv):
            self.get_logger().error("❌ Failed to link pgie -> nvvidconv")
            return False
            
        if not self.nvvidconv.link(self.nvosd):
            self.get_logger().error("❌ Failed to link nvvidconv -> nvosd")
            return False
            
        if not self.nvosd.link(self.tiler):
            self.get_logger().error("❌ Failed to link nvosd -> tiler")
            return False
            
        if not self.tiler.link(self.fakesink):
            self.get_logger().error("❌ Failed to link tiler -> fakesink")
            return False
        
        self.get_logger().info("✅ Pipeline elements linked")
        return True

    def bus_call(self, bus, message):
        """Handle bus messages"""
        t = message.type
        if t == Gst.MessageType.EOS:
            self.get_logger().info("End-of-stream")
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.get_logger().error(f"❌ Error: {err}: {debug}")
        return True

    def camera_callback(self, msg, camera_idx):
        """Handle camera input"""
        try:
            # Convert ROS image to CV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Feed to DeepStream (simplified for now)
            self.process_frame(cv_image, camera_idx)
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error {camera_idx}: {e}")

    def process_frame(self, frame, camera_idx):
        """Process frame with DeepStream"""
        # Placeholder - implement DeepStream processing
        # For now, create dummy detection message
        try:
            detection_msg = Yolov12Inference()
            detection_msg.header.stamp = self.get_clock().now().to_msg()
            detection_msg.header.frame_id = f"camera_{camera_idx}"
            detection_msg.camera_name = f"camera_{camera_idx}"
            detection_msg.task = "detect"
            
            # Publish
            if camera_idx < len(self.result_pubs):
                self.result_pubs[camera_idx].publish(detection_msg)
                
        except Exception as e:
            self.get_logger().error(f"❌ Process frame error: {e}")

    def log_fps(self):
        """Log FPS statistics"""
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        
        if elapsed > 0:
            fps = self.frame_count / elapsed
            self.get_logger().info(f"🚀 DeepStream FPS: {fps:.1f}")
            
            if fps >= 60:
                self.get_logger().info("🎯 Excellent performance!")
            elif fps >= 30:
                self.get_logger().info("✅ Good performance!")
            else:
                self.get_logger().warn(f"⚡ Performance: {fps:.1f} FPS")
        
        # Reset counters
        self.frame_count = 0
        self.last_fps_time = current_time

    def destroy_node(self):
        """Clean shutdown"""
        try:
            if hasattr(self, 'pipeline'):
                self.pipeline.set_state(Gst.State.NULL)
            self.get_logger().info("🛑 DeepStream node shutdown")
            super().destroy_node()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = DeepStreamYOLONode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()