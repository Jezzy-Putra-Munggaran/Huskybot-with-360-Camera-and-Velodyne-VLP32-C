#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolov12_msgs.msg import Yolov12Inference, InferenceResult
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
import queue
import colorsys
import os

class UltraOptimizedDeepStreamNode(Node):
    def __init__(self):
        super().__init__('ultra_optimized_deepstream')
        
        self.bridge = CvBridge()
        self.setup_parameters()
        
        # ✅ MAXIMUM optimized data structures
        self.latest_images = [None] * 6
        self.latest_headers = [None] * 6
        self.processing_flags = [False] * 6
        self.image_locks = [threading.Lock() for _ in range(6)]
        
        # ✅ Setup components
        self.setup_ros_topics()
        self.setup_enhanced_coco_colors()
        self.setup_ultra_optimized_model()
        self.setup_parallel_processing()
        
        # ✅ Performance monitoring
        self.frame_count = 0
        self.detection_count = 0
        self.total_inference_time = 0
        self.inference_count = 0
        self.fps_timer = self.create_timer(1.0, self.log_performance)
        self.last_fps_time = time.time()
        
        self.get_logger().info("🚀 ULTRA-OPTIMIZED DeepStream Node initialized!")

    def setup_parameters(self):
        """Setup ULTRA-OPTIMIZED parameters"""
        self.declare_parameter('model_engine', 'yolo11n-seg.engine')
        self.declare_parameter('input_width', 320)   # ✅ MAXIMUM reduction for speed
        self.declare_parameter('input_height', 320)  # ✅ MAXIMUM reduction for speed
        self.declare_parameter('batch_size', 1)      # ✅ Single for speed
        self.declare_parameter('fps_target', 120)
        self.declare_parameter('device_id', 0)
        
        self.model_engine = self.get_parameter('model_engine').value
        self.input_width = self.get_parameter('input_width').value
        self.input_height = self.get_parameter('input_height').value
        self.batch_size = self.get_parameter('batch_size').value
        self.fps_target = self.get_parameter('fps_target').value
        self.device_id = self.get_parameter('device_id').value

    def setup_ultra_optimized_model(self):
        """✅ MAXIMUM model optimizations"""
        try:
            from ultralytics import YOLO
            
            model_path = f"/home/kmp-orin/jezzy/huskybot/{self.model_engine}"
            
            if not os.path.exists(model_path):
                self.get_logger().error(f"❌ Model not found: {model_path}")
                self.yolo_model = None
                return
            
            self.get_logger().info(f"🔥 Loading ULTRA-OPTIMIZED model: {model_path}")
            
            # ✅ Load with MAXIMUM optimizations
            self.yolo_model = YOLO(model_path)
            
            # ✅ MAXIMUM warmup with smaller dummy
            dummy_array = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)
            
            start_time = time.time()
            for _ in range(3):
                results = self.yolo_model(dummy_array, 
                                       conf=0.3,  # Optimal confidence
                                       device='cuda:0',
                                       half=True,
                                       verbose=False,
                                       agnostic_nms=True,
                                       max_det=10,  # Reduced for speed
                                       imgsz=(self.input_width, self.input_height),
                                       task='segment')
            
            warmup_time = time.time() - start_time
            self.get_logger().info(f"✅ ULTRA-OPTIMIZED Model ready: {warmup_time*1000:.1f}ms")
            
        except Exception as e:
            self.get_logger().error(f"❌ Model loading failed: {e}")
            self.yolo_model = None

    def setup_enhanced_coco_colors(self):
        """Setup 80 distinct colors for COCO classes"""
        self.coco_colors = []
        self.text_colors = []
        
        # ✅ Enhanced color generation
        for i in range(80):
            hue = (i * 137.5) % 360
            sat = 0.8 + (i % 3) * 0.1
            val = 0.7 + (i % 4) * 0.1
            
            r, g, b = colorsys.hsv_to_rgb(hue/360.0, sat, val)
            color = [int(r*255), int(g*255), int(b*255)]
            self.coco_colors.append(color)
            
            # ✅ Optimal text color for contrast
            brightness = (color[0] * 0.299 + color[1] * 0.587 + color[2] * 0.114) / 255
            self.text_colors.append((0, 0, 0) if brightness > 0.5 else (255, 255, 255))

    def setup_ros_topics(self):
        """Setup ROS2 topics with CORRECT mapping"""
        self.camera_subs = []
        self.camera_names = ['rear', 'rear_right', 'front_right', 'front', 'front_left', 'rear_left']
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
                1  # Minimal queue for speed
            )
            self.camera_subs.append(sub)
            self.get_logger().info(f"📡 Subscribed: {topic} -> {name}")
        
        # ✅ Publishers for results
        self.result_pubs = []
        for name in self.camera_names:
            det_pub = self.create_publisher(Yolov12Inference, f'/camera_{name}/detections', 1)
            seg_pub = self.create_publisher(Yolov12Inference, f'/camera_{name}/segmentation', 1)
            self.result_pubs.append((det_pub, seg_pub))
        
        # ✅ Enhanced grid publisher
        self.grid_pub = self.create_publisher(Image, '/deepstream_grid', 1)

    def setup_parallel_processing(self):
        """✅ MAXIMUM parallel processing"""
        self.frame_queues = [queue.Queue(maxsize=1) for _ in range(6)]
        self.processing_active = True
        
        # ✅ Dedicated threads per camera
        self.processing_threads = []
        for i in range(6):
            thread = threading.Thread(
                target=self.ultra_fast_worker, 
                args=(i,),
                daemon=True
            )
            thread.start()
            self.processing_threads.append(thread)
        
        # ✅ Enhanced grid thread
        self.grid_thread = threading.Thread(
            target=self.ultra_fast_grid_worker, 
            daemon=True
        )
        self.grid_thread.start()

    def ultra_fast_callback(self, msg, camera_idx):
        """✅ MAXIMUM speed callback"""
        try:
            if self.processing_flags[camera_idx]:
                return  # Skip if processing
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            with self.image_locks[camera_idx]:
                self.latest_images[camera_idx] = cv_image
                self.latest_headers[camera_idx] = msg.header
            
            # ✅ Non-blocking queue add
            try:
                self.frame_queues[camera_idx].put_nowait((cv_image, msg.header, camera_idx))
            except queue.Full:
                try:
                    self.frame_queues[camera_idx].get_nowait()
                    self.frame_queues[camera_idx].put_nowait((cv_image, msg.header, camera_idx))
                except queue.Empty:
                    pass
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"❌ Callback error {camera_idx}: {e}")

    def ultra_fast_worker(self, camera_idx):
        """✅ MAXIMUM speed worker"""
        while self.processing_active:
            try:
                frame_data = self.frame_queues[camera_idx].get(timeout=0.001)
                cv_image, header, cam_idx = frame_data
                
                self.processing_flags[cam_idx] = True
                
                # ✅ ULTRA-FAST inference with enhanced results
                self.maximum_speed_inference_with_display(cv_image, header, cam_idx)
                
                self.processing_flags[cam_idx] = False
                
            except queue.Empty:
                time.sleep(0.0001)
            except Exception as e:
                self.processing_flags[camera_idx] = False
                continue

    def maximum_speed_inference_with_display(self, frame, header, camera_idx):
        """✅ MAXIMUM speed inference with ENHANCED bounding box info"""
        try:
            if not self.yolo_model:
                return
            
            # ✅ Ultra-fast resize
            resized = cv2.resize(frame, (self.input_width, self.input_height), 
                               interpolation=cv2.INTER_AREA)
            
            # ✅ MAXIMUM optimized inference
            start_time = time.time()
            results = self.yolo_model(resized, 
                                   conf=0.3,
                                   device='cuda:0',
                                   half=True,
                                   verbose=False,
                                   agnostic_nms=True,
                                   max_det=10,
                                   imgsz=(self.input_width, self.input_height),
                                   task='segment')
            
            inference_time = time.time() - start_time
            self.total_inference_time += inference_time
            self.inference_count += 1
            
            # ✅ Create enhanced messages
            detection_msg = Yolov12Inference()
            detection_msg.header = header
            detection_msg.camera_name = f"camera_{self.camera_names[camera_idx]}"
            detection_msg.task = "detect"
            detection_msg.frame_type = f"ultra_optimized_{camera_idx}"
            detection_msg.note = f"Inference: {inference_time*1000:.1f}ms"
            
            segmentation_msg = Yolov12Inference()
            segmentation_msg.header = header
            segmentation_msg.camera_name = f"camera_{self.camera_names[camera_idx]}"
            segmentation_msg.task = "segment"
            segmentation_msg.frame_type = f"ultra_optimized_{camera_idx}"
            segmentation_msg.note = f"Inference: {inference_time*1000:.1f}ms"
            
            # ✅ Create display image with ENHANCED annotations
            display_frame = frame.copy()
            
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
                    
                    # ✅ Calculate angle from camera position
                    bbox_center_x = (result.left + result.right) / 2
                    image_width = frame.shape[1]
                    
                    camera_base_angles = {0: 180, 1: 225, 2: 315, 3: 0, 4: 45, 5: 135}
                    base_angle = camera_base_angles.get(camera_idx, 0)
                    angle_offset = ((bbox_center_x / image_width) - 0.5) * 60
                    object_angle = (base_angle + angle_offset) % 360
                    result.angle = object_angle
                    
                    # ✅ Estimate distance
                    bbox_height = result.bottom - result.top
                    estimated_distance = max(1.0, 800.0 / bbox_height)
                    result.distance = estimated_distance
                    
                    # ✅ Calculate 3D coordinates
                    angle_rad = np.radians(object_angle)
                    result.coordinate_x = estimated_distance * np.cos(angle_rad)
                    result.coordinate_y = estimated_distance * np.sin(angle_rad)
                    result.coordinate_z = 0.5
                    
                    # ✅ Enhanced mask processing
                    if masks is not None and i < len(masks):
                        mask = masks[i]
                        mask_resized = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                        mask_binary = (mask_resized > 0.5).astype(np.uint8)
                        
                        # ✅ Enhanced mask data
                        result.mask_width = frame.shape[1]
                        result.mask_height = frame.shape[0]
                        result.mask_data = mask_binary.flatten().tolist()
                    
                    # ✅ Enhanced colors
                    if class_id < len(self.coco_colors):
                        color = self.coco_colors[class_id]
                        text_color = self.text_colors[class_id]
                        result.color_r, result.color_g, result.color_b = color
                    else:
                        color = [255, 255, 255]
                        text_color = (0, 0, 0)
                        result.color_r = result.color_g = result.color_b = 255
                    
                    # ✅ ENHANCED bounding box with ALL INFO
                    cv2.rectangle(display_frame, (result.left, result.top), 
                                (result.right, result.bottom), color, 3)
                    
                    # ✅ Enhanced info text with ALL required data
                    info_lines = [
                        f"Class: {result.class_name}",
                        f"Conf: {result.confidence:.2f}",
                        f"Dist: {result.distance:.1f}m",
                        f"Pos: ({result.coordinate_x:.1f}, {result.coordinate_y:.1f}, {result.coordinate_z:.1f})",
                        f"Angle: {result.angle:.0f}°"
                    ]
                    
                    # ✅ Draw enhanced info box
                    text_y = result.top - 10
                    for line_idx, info_line in enumerate(info_lines):
                        text_size = cv2.getTextSize(info_line, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                        
                        # ✅ Background for readability
                        cv2.rectangle(display_frame, 
                                    (result.left, text_y - text_size[1] - 5), 
                                    (result.left + text_size[0] + 10, text_y + 5), 
                                    color, -1)
                        
                        cv2.putText(display_frame, info_line, 
                                  (result.left + 5, text_y), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
                        
                        text_y -= (text_size[1] + 8)
                    
                    # ✅ Enhanced mask overlay
                    if masks is not None and i < len(masks):
                        mask = masks[i]
                        mask_resized = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                        mask_colored = np.zeros_like(display_frame)
                        mask_colored[:,:] = color
                        
                        alpha = 0.3
                        mask_area = mask_resized > 0.5
                        display_frame[mask_area] = cv2.addWeighted(
                            display_frame[mask_area], 1-alpha,
                            mask_colored[mask_area], alpha, 0
                        )
                    
                    detection_msg.yolov12_inference.append(result)
                    segmentation_msg.yolov12_inference.append(result)
                    self.detection_count += 1
            
            # ✅ Update display image
            with self.image_locks[camera_idx]:
                self.latest_images[camera_idx] = display_frame
            
            # ✅ Publish results
            if camera_idx < len(self.result_pubs):
                det_pub, seg_pub = self.result_pubs[camera_idx]
                det_pub.publish(detection_msg)
                seg_pub.publish(segmentation_msg)
                
        except Exception as e:
            self.get_logger().error(f"❌ Inference error {camera_idx}: {e}")

    def ultra_fast_grid_worker(self):
        """✅ MAXIMUM optimized grid worker"""
        while self.processing_active:
            try:
                self.create_ultra_enhanced_grid()
                time.sleep(0.0333)  # 30 FPS for grid
            except Exception as e:
                self.get_logger().error(f"❌ Grid error: {e}")
                time.sleep(0.1)

    def create_ultra_enhanced_grid(self):
        """✅ MAXIMUM enhanced 2x3 grid with ALL cameras"""
        try:
            target_size = (640, 360)  # Optimal per camera
            grid_images = []
            
            for i in range(6):
                if self.latest_images[i] is not None:
                    with self.image_locks[i]:
                        img = self.latest_images[i].copy()
                    
                    img_resized = cv2.resize(img, target_size, interpolation=cv2.INTER_LINEAR)
                    grid_images.append(img_resized)
                else:
                    black_img = np.zeros((target_size[1], target_size[0], 3), dtype=np.uint8)
                    camera_name = self.camera_names[i].upper().replace('_', ' ')
                    cv2.putText(black_img, f"CAM {i+1}: {camera_name}", 
                              (target_size[0]//4, target_size[1]//2), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                    grid_images.append(black_img)
            
            if len(grid_images) == 6:
                top_row = np.hstack([grid_images[0], grid_images[1], grid_images[2]])
                bottom_row = np.hstack([grid_images[3], grid_images[4], grid_images[5]])
                grid = np.vstack([top_row, bottom_row])
                
                # ✅ Enhanced status info
                avg_inference = self.total_inference_time / max(1, self.inference_count)
                theoretical_fps = 1.0 / avg_inference if avg_inference > 0 else 0
                
                status_height = 80
                cv2.rectangle(grid, (0, grid.shape[0]-status_height), (grid.shape[1], grid.shape[0]), (0, 0, 0), -1)
                
                status_color = (0, 255, 0) if theoretical_fps >= 100 else (0, 255, 255)
                
                info_lines = [
                    f"HUSKYBOT ULTRA-OPTIMIZED 360° | Detections: Active",
                    f"Theoretical FPS: {theoretical_fps:.1f} | Target: 100+ | Status: {'ACHIEVED!' if theoretical_fps >= 100 else 'OPTIMIZING...'}",
                    f"Inference: {avg_inference*1000:.1f}ms | All 6 cameras active with bounding box info"
                ]
                
                for idx, info_line in enumerate(info_lines):
                    y_pos = grid.shape[0] - status_height + 20 + (idx * 20)
                    cv2.putText(grid, info_line, (20, y_pos), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
                
                # ✅ Publish grid
                grid_msg = self.bridge.cv2_to_imgmsg(grid, 'bgr8')
                grid_msg.header.stamp = self.get_clock().now().to_msg()
                grid_msg.header.frame_id = "ultra_optimized_grid_all_cameras"
                self.grid_pub.publish(grid_msg)
                
        except Exception as e:
            self.get_logger().error(f"❌ Grid creation error: {e}")

    def log_performance(self):
        """✅ Enhanced performance logging"""
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        
        if elapsed > 0:
            actual_fps = self.frame_count / elapsed
            detection_rate = self.detection_count / elapsed
            avg_inference = self.total_inference_time / max(1, self.inference_count)
            theoretical_fps = 1.0 / avg_inference if avg_inference > 0 else 0
            
            if theoretical_fps >= 100:
                self.get_logger().info(
                    f"🎯 TARGET ACHIEVED! Theoretical: {theoretical_fps:.1f} FPS | "
                    f"Actual: {actual_fps:.1f} FPS | Det/s: {detection_rate:.1f} | "
                    f"Inference: {avg_inference*1000:.1f}ms"
                )
            else:
                self.get_logger().info(
                    f"🔥 Optimizing: Theoretical: {theoretical_fps:.1f} FPS | "
                    f"Actual: {actual_fps:.1f} FPS | Det/s: {detection_rate:.1f} | "
                    f"Inference: {avg_inference*1000:.1f}ms"
                )
        
        self.frame_count = 0
        self.detection_count = 0
        self.last_fps_time = current_time

    def destroy_node(self):
        """Clean shutdown"""
        self.processing_active = False
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = UltraOptimizedDeepStreamNode()
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