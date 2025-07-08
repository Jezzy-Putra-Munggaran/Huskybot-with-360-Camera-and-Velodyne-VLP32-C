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
import queue
import colorsys

class UltraFastDeepStreamNode(Node):
    def __init__(self):
        super().__init__('deepstream_yolo')
        
        self.bridge = CvBridge()
        self.setup_parameters()
        
        # ✅ CRITICAL: Single model instance with optimizations
        self.setup_ultra_optimized_model()
        
        # ✅ Setup ROS topics
        self.setup_ros_topics()
        
        # ✅ Enhanced COCO colors
        self.setup_enhanced_coco_colors()
        
        # ✅ ULTRA-FAST threading with reduced overhead
        self.setup_ultra_fast_processing()
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0
        self.fps_timer = self.create_timer(1.0, self.log_fps)  # Reduced to 1s
        self.last_fps_time = time.time()
        
        # ✅ Camera data with minimal locking
        self.latest_images = [None] * 6
        self.latest_headers = [None] * 6
        self.latest_detections = [[] for _ in range(6)]
        self.processing_flags = [False] * 6  # Atomic flags instead of locks
        
        self.get_logger().info("🚀 ULTRA-FAST DeepStream Node initialized!")

    def setup_parameters(self):
        """Setup optimized parameters"""
        self.declare_parameter('model_engine', 'yolo11n-seg.engine')
        self.declare_parameter('input_width', 416)  # ✅ Reduced for speed
        self.declare_parameter('input_height', 416)
        self.declare_parameter('batch_size', 6)
        self.declare_parameter('fps_target', 120)
        self.declare_parameter('device_id', 0)
        self.declare_parameter('skip_frames', 2)  # ✅ Increased skipping
        
        self.model_engine = self.get_parameter('model_engine').value
        self.input_width = self.get_parameter('input_width').value
        self.input_height = self.get_parameter('input_height').value
        self.batch_size = self.get_parameter('batch_size').value
        self.fps_target = self.get_parameter('fps_target').value
        self.device_id = self.get_parameter('device_id').value
        self.skip_frames = self.get_parameter('skip_frames').value
        
        self.frame_skip_counter = 0

    def setup_ultra_optimized_model(self):
        """✅ ULTRA-OPTIMIZED model loading"""
        try:
            from ultralytics import YOLO
            
            # ✅ FIXED path
            model_path = f"/home/kmp-orin/jezzy/huskybot/{self.model_engine}"
            
            if not os.path.exists(model_path):
                self.get_logger().error(f"❌ Model not found: {model_path}")
                self.yolo_model = None
                return
            
            self.get_logger().info(f"🔥 Loading ULTRA-OPTIMIZED model: {model_path}")
            
            # ✅ Load with maximum optimizations
            self.yolo_model = YOLO(model_path)
            
            # ✅ SINGLE ultra-fast warmup
            dummy_img = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)
            
            start_time = time.time()
            
            # ✅ Ultra-optimized inference settings
            results = self.yolo_model(dummy_img, 
                                   conf=0.5,  # Higher confidence for speed
                                   device='cuda:0',
                                   half=True,
                                   verbose=False,
                                   agnostic_nms=True,
                                   max_det=50,  # Reduced max detections
                                   imgsz=self.input_width,
                                   task='segment')
            
            warmup_time = time.time() - start_time
            self.get_logger().info(f"✅ ULTRA-FAST Model ready: {warmup_time*1000:.1f}ms")
            
        except Exception as e:
            self.get_logger().error(f"❌ Model loading failed: {e}")
            self.yolo_model = None

    def setup_enhanced_coco_colors(self):
        """Setup 80 distinct colors"""
        self.coco_colors = []
        self.text_colors = []
        
        for i in range(80):
            hue = (i * 137.5) % 360
            sat = 0.9 + (i % 2) * 0.1
            val = 0.7 + (i % 3) * 0.1
            
            r, g, b = colorsys.hsv_to_rgb(hue/360.0, sat, val)
            color = [int(r*255), int(g*255), int(b*255)]
            self.coco_colors.append(color)
            
            brightness = (r * 0.299 + g * 0.587 + b * 0.114)
            if brightness > 0.5:
                self.text_colors.append((0, 0, 0))
            else:
                self.text_colors.append((255, 255, 255))

    def setup_ultra_fast_processing(self):
        """✅ ULTRA-FAST processing setup"""
        self.frame_queues = [queue.Queue(maxsize=2) for _ in range(6)]  # Smaller queues
        self.processing_active = True
        
        # ✅ SINGLE ultra-fast processing thread
        self.processing_thread = threading.Thread(
            target=self.ultra_fast_worker, 
            daemon=True
        )
        self.processing_thread.start()
        
        # ✅ Separate grid thread with lower priority
        self.grid_thread = threading.Thread(
            target=self.grid_creation_worker, 
            daemon=True
        )
        self.grid_thread.start()

    def setup_ros_topics(self):
        """Setup ROS2 topics with correct mapping"""
        self.camera_subs = []
        self.camera_names = ['rear', 'rear_left', 'front_left', 'front', 'front_right', 'rear_right']
        actual_topics = [
            '/camera_front/image_raw',      # KAMERA BELAKANG
            '/camera_right/image_raw',      # KAMERA KANAN BELAKANG
            '/camera_rear_right/image_raw', # KAMERA KANAN DEPAN
            '/camera_rear/image_raw',       # KAMERA DEPAN
            '/camera_left/image_raw',       # KAMERA KIRI DEPAN
            '/camera_front_left/image_raw'  # KAMERA KIRI BELAKANG
        ]
        
        for i, (name, topic) in enumerate(zip(self.camera_names, actual_topics)):
            sub = self.create_subscription(
                Image, topic, 
                lambda msg, idx=i: self.ultra_fast_callback(msg, idx), 
                1  # ✅ Minimal queue size
            )
            self.camera_subs.append(sub)
            self.get_logger().info(f"📡 Subscribed to: {topic} -> Camera {name}")
        
        # Publishers
        self.result_pubs = []
        for name in self.camera_names:
            det_pub = self.create_publisher(Yolov12Inference, f'/camera_{name}/detections', 1)
            seg_pub = self.create_publisher(Yolov12Inference, f'/camera_{name}/segmentation', 1)
            self.result_pubs.append((det_pub, seg_pub))
        
        # Grid publisher
        self.grid_pub = self.create_publisher(Image, '/deepstream_grid', 1)

    def ultra_fast_callback(self, msg, camera_idx):
        """✅ ULTRA-FAST callback with minimal processing"""
        try:
            # ✅ Aggressive frame skipping
            self.frame_skip_counter += 1
            if self.frame_skip_counter % (self.skip_frames + 1) != 0:
                return
            
            # ✅ Skip if already processing
            if self.processing_flags[camera_idx]:
                return
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # ✅ Store without locks (atomic update)
            self.latest_images[camera_idx] = cv_image
            self.latest_headers[camera_idx] = msg.header
            
            # ✅ Add to queue if space available
            try:
                self.frame_queues[camera_idx].put_nowait((cv_image, msg.header, camera_idx))
            except queue.Full:
                # Drop frame if queue full
                pass
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Callback error {camera_idx}: {e}")

    def ultra_fast_worker(self):
        """✅ SINGLE ultra-fast worker thread"""
        while self.processing_active:
            try:
                processed_any = False
                
                # ✅ Process all cameras in round-robin
                for i in range(6):
                    try:
                        frame_data = self.frame_queues[i].get_nowait()
                        cv_image, header, camera_idx = frame_data
                        
                        # ✅ Set processing flag
                        self.processing_flags[camera_idx] = True
                        
                        # ✅ ULTRA-FAST processing
                        detections = self.ultra_fast_inference(cv_image, header, camera_idx)
                        
                        # ✅ Store results
                        self.latest_detections[camera_idx] = detections
                        
                        # ✅ Clear processing flag
                        self.processing_flags[camera_idx] = False
                        
                        processed_any = True
                        
                    except queue.Empty:
                        continue
                    except Exception as e:
                        self.processing_flags[i] = False
                        continue
                
                if not processed_any:
                    time.sleep(0.001)  # Minimal sleep
                    
            except Exception as e:
                self.get_logger().error(f"❌ Worker error: {e}")
                time.sleep(0.01)

    def ultra_fast_inference(self, frame, header, camera_idx):
        """✅ ULTRA-FAST inference with maximum optimizations"""
        detections = []
        try:
            if not self.yolo_model:
                return detections
            
            # ✅ Ultra-fast resize
            resized = cv2.resize(frame, (self.input_width, self.input_height), 
                               interpolation=cv2.INTER_LINEAR)
            
            # ✅ ULTRA-OPTIMIZED inference
            start_time = time.time()
            results = self.yolo_model(resized, 
                                   conf=0.5,  # Higher confidence
                                   device='cuda:0',
                                   half=True,
                                   verbose=False,
                                   agnostic_nms=True,
                                   max_det=50,  # Reduced
                                   imgsz=self.input_width,
                                   task='segment')
            
            inference_time = time.time() - start_time
            
            # ✅ Create messages
            detection_msg = Yolov12Inference()
            detection_msg.header = header
            detection_msg.camera_name = f"camera_{self.camera_names[camera_idx]}"
            detection_msg.task = "detect"
            detection_msg.frame_type = "ultra_fast_segmentation"
            
            segmentation_msg = Yolov12Inference()
            segmentation_msg.header = header
            segmentation_msg.camera_name = f"camera_{self.camera_names[camera_idx]}"
            segmentation_msg.task = "segment"
            segmentation_msg.frame_type = "ultra_fast_segmentation"
            
            # ✅ Process results with optimizations
            detection_count = 0
            if results[0].boxes is not None and len(results[0].boxes) > 0:
                scale_x = frame.shape[1] / self.input_width
                scale_y = frame.shape[0] / self.input_height
                
                masks = None
                if results[0].masks is not None:
                    masks = results[0].masks.data.cpu().numpy()
                
                for i, box in enumerate(results[0].boxes):
                    result = InferenceResult()
                    class_id = int(box.cls)
                    result.class_name = results[0].names[class_id]
                    result.confidence = float(box.conf)
                    
                    # ✅ Scale coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    result.left = int(x1 * scale_x)
                    result.top = int(y1 * scale_y)
                    result.right = int(x2 * scale_x)
                    result.bottom = int(y2 * scale_y)
                    
                    # ✅ Temporary coordinates (fusion will update)
                    result.distance = 0.0
                    result.coordinate_x = 0.0
                    result.coordinate_y = 0.0
                    result.coordinate_z = 0.0
                    result.angle = 0.0
                    
                    # ✅ FAST mask processing
                    if masks is not None and i < len(masks):
                        mask = masks[i]
                        # ✅ Simplified mask processing
                        mask_resized = cv2.resize(mask, (result.right-result.left, result.bottom-result.top), 
                                                interpolation=cv2.INTER_NEAREST)
                        if mask_resized.size > 0:
                            mask_uint8 = (mask_resized * 255).astype(np.uint8)
                            result.mask_data = mask_uint8.flatten().tolist()
                            result.mask_width = mask_resized.shape[1]
                            result.mask_height = mask_resized.shape[0]
                        else:
                            result.mask_data = []
                            result.mask_width = 0
                            result.mask_height = 0
                    else:
                        result.mask_data = []
                        result.mask_width = 0
                        result.mask_height = 0
                    
                    # ✅ Enhanced colors
                    if class_id < len(self.coco_colors):
                        color = self.coco_colors[class_id]
                        result.color_r = color[0]
                        result.color_g = color[1]
                        result.color_b = color[2]
                    else:
                        result.color_r = (class_id * 67) % 256
                        result.color_g = (class_id * 131) % 256
                        result.color_b = (class_id * 197) % 256
                    
                    detection_msg.yolov12_inference.append(result)
                    segmentation_msg.yolov12_inference.append(result)
                    detections.append(result)
                    detection_count += 1
            
            # ✅ Publish results
            if camera_idx < len(self.result_pubs):
                det_pub, seg_pub = self.result_pubs[camera_idx]
                det_pub.publish(detection_msg)
                seg_pub.publish(segmentation_msg)
            
            self.detection_count += detection_count
            
            # ✅ Performance logging (reduced frequency)
            if camera_idx == 0 and detection_count > 0:
                fps_estimate = 1.0 / inference_time if inference_time > 0 else 0
                if fps_estimate >= 100:
                    self.get_logger().info(f"🎯 Camera {camera_idx}: {detection_count} det, {fps_estimate:.0f} FPS - TARGET ACHIEVED!")
                else:
                    self.get_logger().info(f"⚡ Camera {camera_idx}: {detection_count} det, {fps_estimate:.0f} FPS")
                
        except Exception as e:
            self.get_logger().error(f"❌ Inference error {camera_idx}: {e}")
        
        return detections

    def grid_creation_worker(self):
        """✅ Optimized grid creation"""
        while self.processing_active:
            try:
                self.create_ultra_fast_grid()
                time.sleep(0.05)  # 20 FPS for visualization
            except Exception as e:
                self.get_logger().error(f"❌ Grid error: {e}")
                time.sleep(0.1)

    def create_ultra_fast_grid(self):
        """✅ ULTRA-FAST grid visualization"""
        try:
            target_size = (640, 480)  # ✅ Optimized size for speed
            
            grid_images = []
            total_detections = 0
            
            for i in range(6):
                if self.latest_images[i] is not None and not self.processing_flags[i]:
                    img = cv2.resize(self.latest_images[i], target_size)
                    
                    # ✅ Draw detections if available
                    display_detections = self.latest_detections[i]
                    
                    if display_detections:
                        scale_x = target_size[0] / self.latest_images[i].shape[1]
                        scale_y = target_size[1] / self.latest_images[i].shape[0]
                        
                        for det in display_detections:
                            # ✅ Scale coordinates
                            x1 = int(det.left * scale_x)
                            y1 = int(det.top * scale_y)
                            x2 = int(det.right * scale_x)
                            y2 = int(det.bottom * scale_y)
                            
                            # ✅ FAST mask drawing (simplified)
                            if hasattr(det, 'mask_data') and det.mask_data and det.mask_width > 0:
                                try:
                                    mask_array = np.array(det.mask_data, dtype=np.uint8)
                                    mask = mask_array.reshape((det.mask_height, det.mask_width))
                                    
                                    if x2 > x1 and y2 > y1:
                                        mask_resized = cv2.resize(mask, (x2-x1, y2-y1))
                                        
                                        # ✅ Fast overlay
                                        roi = img[y1:y2, x1:x2]
                                        if roi.shape[:2] == mask_resized.shape:
                                            mask_3d = cv2.cvtColor(mask_resized, cv2.COLOR_GRAY2BGR)
                                            mask_colored = mask_3d * np.array([det.color_b/255, det.color_g/255, det.color_r/255])
                                            img[y1:y2, x1:x2] = cv2.addWeighted(roi, 0.7, mask_colored.astype(np.uint8), 0.3, 0)
                                except:
                                    pass
                            
                            # ✅ Draw bounding box
                            bbox_color = (int(det.color_b), int(det.color_g), int(det.color_r))
                            cv2.rectangle(img, (x1, y1), (x2, y2), bbox_color, 2)
                            
                            # ✅ Simplified label
                            label = f"{det.class_name} {det.confidence:.2f}"
                            cv2.putText(img, label, (x1, y1-10), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, bbox_color, 2)
                    
                    total_detections += len(display_detections)
                    
                    # ✅ Camera info
                    cv2.rectangle(img, (0, 0), (target_size[0], 30), (0, 0, 0), -1)
                    cv2.putText(img, f"{self.camera_names[i].upper()}: {len(display_detections)}", 
                              (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    
                    grid_images.append(img)
                else:
                    # ✅ Black placeholder
                    black_img = np.zeros((target_size[1], target_size[0], 3), dtype=np.uint8)
                    cv2.putText(black_img, f"{self.camera_names[i].upper()}", 
                              (target_size[0]//4, target_size[1]//2), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    grid_images.append(black_img)
            
            # ✅ Create 2x3 grid
            if len(grid_images) == 6:
                top_row = np.hstack([grid_images[0], grid_images[1], grid_images[2]])
                bottom_row = np.hstack([grid_images[3], grid_images[4], grid_images[5]])
                grid = np.vstack([top_row, bottom_row])
                
                # ✅ FAST info overlay
                current_fps = self.frame_count / 1.0 if self.frame_count > 0 else 0
                
                info_text = f"ULTRA-FAST | FPS: {current_fps:.1f} | Det: {total_detections} | Target: 100+ FPS"
                cv2.putText(grid, info_text, (10, grid.shape[0]-20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                
                # ✅ Publish grid
                grid_msg = self.bridge.cv2_to_imgmsg(grid, 'bgr8')
                grid_msg.header.stamp = self.get_clock().now().to_msg()
                grid_msg.header.frame_id = "ultra_fast_grid"
                self.grid_pub.publish(grid_msg)
                
        except Exception as e:
            self.get_logger().error(f"❌ Grid visualization error: {e}")

    def log_fps(self):
        """✅ ULTRA-FAST FPS logging"""
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        
        if elapsed > 0:
            fps = self.frame_count / elapsed
            detection_rate = self.detection_count / elapsed
            
            if fps >= 100:
                self.get_logger().info(f"🎯 TARGET ACHIEVED! FPS: {fps:.1f} | Det/s: {detection_rate:.1f}")
            elif fps >= 50:
                self.get_logger().info(f"✅ Excellent! FPS: {fps:.1f} | Det/s: {detection_rate:.1f}")
            else:
                self.get_logger().info(f"🔥 Optimizing: FPS: {fps:.1f} | Det/s: {detection_rate:.1f}")
        
        # ✅ Reset counters
        self.frame_count = 0
        self.detection_count = 0
        self.last_fps_time = current_time

    def destroy_node(self):
        """Clean shutdown"""
        self.processing_active = False
        self.get_logger().info("🛑 ULTRA-FAST DeepStream node shutdown")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = UltraFastDeepStreamNode()
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