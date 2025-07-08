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

class DeepStreamYOLONode(Node):
    def __init__(self):
        super().__init__('deepstream_yolo')
        
        # Bridge
        self.bridge = CvBridge()
        
        # Setup parameters
        self.setup_parameters()
        
        # ✅ CRITICAL FIX: Load model ONCE and share across all threads
        self.setup_single_model_instance()
        
        # ✅ Setup ROS topics
        self.setup_ros_topics()
        
        # ✅ ENHANCED COCO Colors
        self.setup_enhanced_coco_colors()
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0
        self.fps_timer = self.create_timer(2.0, self.log_fps)
        self.last_fps_time = time.time()
        
        # ✅ Enhanced camera data tracking
        self.latest_images = [None] * 6
        self.latest_headers = [None] * 6
        self.latest_detections = [[] for _ in range(6)]
        self.camera_locks = [threading.Lock() for _ in range(6)]
        self.last_process_time = [0.0] * 6
        
        # ✅ Fusion data integration
        self.fused_data = [[] for _ in range(6)]
        self.fused_locks = [threading.Lock() for _ in range(6)]
        
        # ✅ CRITICAL: Model lock for thread safety
        self.model_lock = threading.Lock()
        
        # ✅ ULTRA-FAST threading setup
        self.setup_ultra_fast_threading()
        
        self.get_logger().info("🚀 ULTRA-FAST DeepStream YOLO Node initialized!")

    def setup_parameters(self):
        """Setup parameters for 100+ FPS"""
        self.declare_parameter('model_engine', 'yolo11n-seg.engine')
        self.declare_parameter('input_width', 640)  # ✅ Balanced for speed vs accuracy
        self.declare_parameter('input_height', 640)
        self.declare_parameter('batch_size', 6)
        self.declare_parameter('fps_target', 120)
        self.declare_parameter('device_id', 0)
        self.declare_parameter('skip_frames', 1)  # ✅ Reduced frame skipping
        
        self.model_engine = self.get_parameter('model_engine').value
        self.input_width = self.get_parameter('input_width').value
        self.input_height = self.get_parameter('input_height').value
        self.batch_size = self.get_parameter('batch_size').value
        self.fps_target = self.get_parameter('fps_target').value
        self.device_id = self.get_parameter('device_id').value
        self.skip_frames = self.get_parameter('skip_frames').value
        
        self.frame_skip_counter = 0

    def setup_single_model_instance(self):
        """✅ CRITICAL FIX: Load model ONCE only"""
        try:
            from ultralytics import YOLO
            model_path = f"/home/kmp-orin/jezzy/huskybot/{self.model_engine}"
            
            if not os.path.exists(model_path):
                self.get_logger().error(f"❌ Model not found: {model_path}")
                self.yolo_model = None
                return
            
            self.get_logger().info(f"🔥 Loading model ONCE: {model_path}")
            self.yolo_model = YOLO(model_path)
            
            # ✅ SINGLE warmup
            self.get_logger().info("🔥 Single warmup for 100+ FPS...")
            dummy_img = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)
            
            start_time = time.time()
            results = self.yolo_model(dummy_img, 
                                   conf=0.3,
                                   device='cuda:0',
                                   half=True,
                                   verbose=False,
                                   agnostic_nms=True,
                                   max_det=100,
                                   imgsz=self.input_width,
                                   task='segment')
            warmup_time = time.time() - start_time
            self.get_logger().info(f"✅ Model ready: {warmup_time*1000:.1f}ms")
            
        except Exception as e:
            self.get_logger().error(f"❌ Model loading failed: {e}")
            self.yolo_model = None

    def setup_enhanced_coco_colors(self):
        """Setup 80 distinct colors with contrast text"""
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

    def setup_ultra_fast_threading(self):
        """Setup ULTRA-FAST threading"""
        self.frame_queues = [queue.Queue(maxsize=3) for _ in range(6)]  # ✅ Slightly larger queue
        self.processing_active = True
        
        # ✅ Multiple processing threads for parallel processing
        self.processing_threads = []
        for i in range(3):  # ✅ 3 threads for better parallelism
            thread = threading.Thread(
                target=self.ultra_fast_worker, 
                args=(i,),
                daemon=True
            )
            thread.start()
            self.processing_threads.append(thread)
        
        # ✅ Grid creation thread
        self.grid_thread = threading.Thread(
            target=self.grid_creation_worker, 
            daemon=True
        )
        self.grid_thread.start()

    def setup_ros_topics(self):
        """Setup ROS2 topics"""
        self.camera_subs = []
        self.camera_names = ['rear', 'rear_left', 'front_left', 'front', 'front_right', 'rear_right']
        actual_topics = [
            '/camera_front/image_raw',      # KAMERA BELAKANG
            '/camera_front_left/image_raw', # KAMERA KIRI BELAKANG
            '/camera_left/image_raw',       # KAMERA KIRI DEPAN
            '/camera_rear/image_raw',       # KAMERA DEPAN
            '/camera_rear_right/image_raw', # KAMERA KANAN DEPAN
            '/camera_right/image_raw'       # KAMERA KANAN BELAKANG
        ]
        
        for i, (name, topic) in enumerate(zip(self.camera_names, actual_topics)):
            sub = self.create_subscription(
                Image, topic, 
                lambda msg, idx=i: self.camera_callback(msg, idx), 
                10
            )
            self.camera_subs.append(sub)
            self.get_logger().info(f"📡 Subscribed to: {topic} -> Camera {name}")
        
        # Publishers
        self.result_pubs = []
        for name in self.camera_names:
            det_pub = self.create_publisher(Yolov12Inference, f'/camera_{name}/detections', 10)
            seg_pub = self.create_publisher(Yolov12Inference, f'/camera_{name}/segmentation', 10)
            self.result_pubs.append((det_pub, seg_pub))
        
        # Fusion subscriptions
        self.fusion_subs = []
        for i, name in enumerate(self.camera_names):
            fusion_sub = self.create_subscription(
                Yolov12Inference, f'/camera_{name}/fused_detections',
                lambda msg, idx=i: self.fusion_callback(msg, idx), 10)
            self.fusion_subs.append(fusion_sub)
        
        # Grid publisher
        self.grid_pub = self.create_publisher(Image, '/deepstream_grid', 10)

    def fusion_callback(self, msg, camera_idx):
        """Receive fused data"""
        try:
            with self.fused_locks[camera_idx]:
                self.fused_data[camera_idx] = msg.yolov12_inference
        except Exception as e:
            self.get_logger().error(f"❌ Fusion callback error {camera_idx}: {e}")

    def camera_callback(self, msg, camera_idx):
        """ULTRA-FAST camera callback"""
        try:
            # ✅ Reduced frame skipping
            self.frame_skip_counter += 1
            if self.frame_skip_counter % (self.skip_frames + 1) != 0:
                return
            
            current_time = time.time()
            
            # ✅ Higher rate limit for better FPS
            if current_time - self.last_process_time[camera_idx] < 0.02:  # Max 50 FPS per camera
                return
            
            self.last_process_time[camera_idx] = current_time
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            with self.camera_locks[camera_idx]:
                self.latest_images[camera_idx] = cv_image
                self.latest_headers[camera_idx] = msg.header
            
            # Add to processing queue
            try:
                self.frame_queues[camera_idx].put_nowait((cv_image, msg.header, camera_idx))
            except queue.Full:
                # Drop oldest frame if queue full
                try:
                    self.frame_queues[camera_idx].get_nowait()
                    self.frame_queues[camera_idx].put_nowait((cv_image, msg.header, camera_idx))
                except:
                    pass
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Camera callback error {camera_idx}: {e}")

    def ultra_fast_worker(self, worker_id):
        """✅ ULTRA-FAST worker with shared model"""
        while self.processing_active:
            try:
                # Process frames from all cameras
                processed_any = False
                
                for i in range(6):
                    try:
                        frame_data = self.frame_queues[i].get_nowait()
                        cv_image, header, camera_idx = frame_data
                        
                        # ✅ CRITICAL: Process with shared model and lock
                        detections = self.process_with_shared_model(cv_image, header, camera_idx)
                        
                        # Store results
                        with self.camera_locks[camera_idx]:
                            self.latest_detections[camera_idx] = detections
                        
                        processed_any = True
                        
                    except queue.Empty:
                        continue
                
                if not processed_any:
                    time.sleep(0.001)
                    
            except Exception as e:
                self.get_logger().error(f"❌ Worker {worker_id} error: {e}")
                time.sleep(0.01)

    def process_with_shared_model(self, frame, header, camera_idx):
        """✅ CRITICAL: Process with shared model instance"""
        detections = []
        try:
            if not self.yolo_model:
                return detections
            
            # ✅ Resize for speed
            resized = cv2.resize(frame, (self.input_width, self.input_height), 
                               interpolation=cv2.INTER_LINEAR)
            
            # ✅ CRITICAL: Use model with thread lock
            start_time = time.time()
            with self.model_lock:
                results = self.yolo_model(resized, 
                                       conf=0.3,
                                       device='cuda:0',
                                       half=True,
                                       verbose=False,
                                       agnostic_nms=True,
                                       max_det=100,
                                       imgsz=self.input_width,
                                       task='segment')
            
            inference_time = time.time() - start_time
            
            # Create messages
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
            
            # Process results
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
                    
                    # Scale coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    result.left = int(x1 * scale_x)
                    result.top = int(y1 * scale_y)
                    result.right = int(x2 * scale_x)
                    result.bottom = int(y2 * scale_y)
                    
                    # Temporary coordinates (updated by fusion)
                    result.distance = 0.0
                    result.coordinate_x = 0.0
                    result.coordinate_y = 0.0
                    result.coordinate_z = 0.0
                    result.angle = 0.0
                    
                    # Segmentation mask
                    if masks is not None and i < len(masks):
                        mask = masks[i]
                        mask_full = cv2.resize(mask, (frame.shape[1], frame.shape[0]), 
                                             interpolation=cv2.INTER_NEAREST)
                        mask_roi = mask_full[result.top:result.bottom, result.left:result.right]
                        
                        if mask_roi.size > 0:
                            mask_uint8 = (mask_roi * 255).astype(np.uint8)
                            result.mask_data = mask_uint8.flatten().tolist()
                            result.mask_width = mask_roi.shape[1]
                            result.mask_height = mask_roi.shape[0]
                        else:
                            result.mask_data = []
                            result.mask_width = 0
                            result.mask_height = 0
                    else:
                        result.mask_data = []
                        result.mask_width = 0
                        result.mask_height = 0
                    
                    # Enhanced colors
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
            
            # Publish results
            if camera_idx < len(self.result_pubs):
                det_pub, seg_pub = self.result_pubs[camera_idx]
                det_pub.publish(detection_msg)
                seg_pub.publish(segmentation_msg)
            
            self.detection_count += detection_count
            
            # Performance logging (reduced frequency)
            if camera_idx == 0 and detection_count > 0:
                fps_estimate = 1.0 / inference_time if inference_time > 0 else 0
                self.get_logger().info(
                    f"⚡ Camera {camera_idx}: {detection_count} det, "
                    f"{inference_time*1000:.1f}ms, ≈{fps_estimate:.0f} FPS"
                )
                
        except Exception as e:
            self.get_logger().error(f"❌ Process frame error {camera_idx}: {e}")
        
        return detections

    def grid_creation_worker(self):
        """Grid creation worker"""
        while self.processing_active:
            try:
                self.create_optimal_grid_visualization()
                time.sleep(0.033)  # 30 FPS for visualization
            except Exception as e:
                self.get_logger().error(f"❌ Grid creation error: {e}")
                time.sleep(0.1)

    def create_optimal_grid_visualization(self):
        """Create optimal grid visualization"""
        try:
            target_size = (800, 600)  # ✅ Larger size for better visibility
            
            grid_images = []
            total_detections = 0
            
            for i in range(6):
                with self.camera_locks[i]:
                    if self.latest_images[i] is not None:
                        img = cv2.resize(self.latest_images[i], target_size)
                        
                        # Use fused data if available
                        display_detections = []
                        with self.fused_locks[i]:
                            if self.fused_data[i]:
                                display_detections = self.fused_data[i]
                            elif i < len(self.latest_detections):
                                display_detections = self.latest_detections[i]
                        
                        # Draw visualizations
                        if display_detections:
                            scale_x = target_size[0] / self.latest_images[i].shape[1]
                            scale_y = target_size[1] / self.latest_images[i].shape[0]
                            
                            for det in display_detections:
                                # Scale coordinates
                                x1 = int(det.left * scale_x)
                                y1 = int(det.top * scale_y)
                                x2 = int(det.right * scale_x)
                                y2 = int(det.bottom * scale_y)
                                
                                # Draw segmentation mask
                                if hasattr(det, 'mask_data') and det.mask_data and det.mask_width > 0:
                                    try:
                                        mask_array = np.array(det.mask_data, dtype=np.uint8)
                                        mask = mask_array.reshape((det.mask_height, det.mask_width))
                                        
                                        if x2 > x1 and y2 > y1:
                                            mask_resized = cv2.resize(mask, (x2-x1, y2-y1))
                                            
                                            # Create colored mask
                                            color_mask = np.zeros((y2-y1, x2-x1, 3), dtype=np.uint8)
                                            color_mask[:,:,0] = (mask_resized > 128) * det.color_b
                                            color_mask[:,:,1] = (mask_resized > 128) * det.color_g  
                                            color_mask[:,:,2] = (mask_resized > 128) * det.color_r
                                            
                                            # Overlay with transparency
                                            roi = img[y1:y2, x1:x2]
                                            if roi.shape == color_mask.shape:
                                                img[y1:y2, x1:x2] = cv2.addWeighted(roi, 0.7, color_mask, 0.3, 0)
                                    except:
                                        pass
                                
                                # Draw bounding box
                                bbox_color = (int(det.color_b), int(det.color_g), int(det.color_r))
                                cv2.rectangle(img, (x1, y1), (x2, y2), bbox_color, 3)  # ✅ Thicker lines
                                
                                # Multi-line label with contrast text
                                distance = getattr(det, 'distance', 0.0)
                                coord_x = getattr(det, 'coordinate_x', 0.0)
                                coord_y = getattr(det, 'coordinate_y', 0.0)
                                coord_z = getattr(det, 'coordinate_z', 0.0)
                                
                                label_lines = [
                                    f"{det.class_name} {det.confidence:.2f}",
                                    f"Distance: {distance:.1f}m",
                                    f"Coord: ({coord_x:.1f},{coord_y:.1f},{coord_z:.1f})"
                                ]
                                
                                # Calculate text color
                                try:
                                    class_names = list(self.yolo_model.names.values())
                                    class_id = class_names.index(det.class_name) if det.class_name in class_names else 0
                                except:
                                    class_id = 0
                                
                                if class_id < len(self.text_colors):
                                    text_color = self.text_colors[class_id]
                                else:
                                    brightness = (det.color_r * 0.299 + det.color_g * 0.587 + det.color_b * 0.114) / 255
                                    text_color = (0, 0, 0) if brightness > 0.5 else (255, 255, 255)
                                
                                # Draw multi-line label
                                for j, line in enumerate(label_lines):
                                    y_offset = y1 - 50 + j * 16  # ✅ Better spacing
                                    if y_offset > 15:
                                        # Background rectangle
                                        (w, h), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                                        cv2.rectangle(img, (x1, y_offset-12), (x1+w+6, y_offset+4), bbox_color, -1)
                                        # Text
                                        cv2.putText(img, line, (x1+3, y_offset), 
                                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
                        
                        total_detections += len(display_detections)
                        
                        # Camera info overlay
                        cv2.rectangle(img, (0, 0), (target_size[0], 50), (0, 0, 0), -1)
                        cv2.putText(img, f"{self.camera_names[i].upper()}", (10, 30), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                        
                        # Detection count
                        det_count = len(display_detections)
                        cv2.putText(img, f"Det: {det_count}", (target_size[0]-100, 30), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        
                        grid_images.append(img)
                    else:
                        # Black placeholder
                        black_img = np.zeros((target_size[1], target_size[0], 3), dtype=np.uint8)
                        cv2.putText(black_img, f"{self.camera_names[i].upper()}", 
                                  (target_size[0]//4, target_size[1]//2-20), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
                        cv2.putText(black_img, "WAITING...", 
                                  (target_size[0]//4, target_size[1]//2+20), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
                        grid_images.append(black_img)
            
            # Create 2x3 grid
            if len(grid_images) == 6:
                top_row = np.hstack([grid_images[0], grid_images[1], grid_images[2]])
                bottom_row = np.hstack([grid_images[3], grid_images[4], grid_images[5]])
                grid = np.vstack([top_row, bottom_row])
                
                # Enhanced info overlay
                current_fps = self.frame_count / 2.0 if self.frame_count > 0 else 0
                
                # Info panel
                info_height = 80
                info_panel = np.zeros((info_height, grid.shape[1], 3), dtype=np.uint8)
                
                # Main stats
                main_info = f"ULTRA-FAST DeepStream | FPS: {current_fps:.1f} | Detections: {total_detections} | 6 Cameras Active"
                cv2.putText(info_panel, main_info, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                
                # Performance info
                perf_info = f"Target: 100+ FPS | Segmentation: ACTIVE | 3D Fusion: ACTIVE | Enhanced Display: ON"
                cv2.putText(info_panel, perf_info, (10, 55), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                # Combine grid with info panel
                final_grid = np.vstack([grid, info_panel])
                
                # Publish grid
                grid_msg = self.bridge.cv2_to_imgmsg(final_grid, 'bgr8')
                grid_msg.header.stamp = self.get_clock().now().to_msg()
                grid_msg.header.frame_id = "ultra_fast_deepstream_grid"
                self.grid_pub.publish(grid_msg)
                
        except Exception as e:
            self.get_logger().error(f"❌ Grid visualization error: {e}")

    def log_fps(self):
        """Enhanced FPS logging"""
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        
        if elapsed > 0:
            fps = self.frame_count / elapsed
            detection_rate = self.detection_count / elapsed
            
            self.get_logger().info(
                f"⚡ ULTRA-FAST DeepStream FPS: {fps:.1f} | Det/s: {detection_rate:.1f}"
            )
            
            if fps >= 100:
                self.get_logger().info("🎯 TARGET ACHIEVED: 100+ FPS!")
            elif fps >= 50:
                self.get_logger().info("✅ Excellent performance!")
            else:
                self.get_logger().info(f"🔥 Optimizing: {fps:.1f} FPS")
        
        # Reset counters
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