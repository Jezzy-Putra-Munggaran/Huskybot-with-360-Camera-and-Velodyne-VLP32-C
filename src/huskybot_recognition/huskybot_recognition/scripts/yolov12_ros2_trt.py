#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os  # Untuk operasi file dan path
import sys  # Untuk exit dan akses error output
import traceback  # Untuk logging error detail
import threading  # Untuk thread-safe statistik dan retry
import time  # Untuk timestamp dan logging
import csv  # Untuk logging statistik ke file
import logging  # Untuk logging ke file
import rclpy  # Modul utama ROS2 Python
from rclpy.node import Node  # Base class Node untuk ROS2
from sensor_msgs.msg import Image  # Message standar ROS2 untuk gambar
from cv_bridge import CvBridge, CvBridgeError  # Konversi ROS Image <-> OpenCV
from yolov12_msgs.msg import InferenceResult, Yolov12Inference  # Custom message hasil deteksi YOLOv12
import numpy as np  # Library array/matrix
import cv2  # OpenCV untuk image processing
import torch  # PyTorch utama
import tensorrt as trt  # TensorRT untuk optimasi Jetson
import pycuda.driver as cuda  # CUDA untuk GPU acceleration
import pycuda.autoinit  # Inisialisasi CUDA
from collections import OrderedDict, namedtuple  # Untuk struktur data hasil deteksi

# ===================== LOGGING TO FILE (OPSIONAL) =====================
def setup_file_logger(log_path="~/huskybot_yolov12_trt.log"):
    """Setup logger file untuk pencatatan error dan info penting."""
    log_path = os.path.expanduser(log_path)
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    file_logger = logging.getLogger("yolov12_trt_logger")
    file_logger.setLevel(logging.INFO)
    file_handler = logging.FileHandler(log_path)
    file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
    file_logger.addHandler(file_handler)
    return file_logger

file_logger = setup_file_logger()  # Inisialisasi logger file global

def log_to_file(msg, level='info'):
    """Log message ke file, dengan level berbeda."""
    if level == 'debug':
        file_logger.debug(msg)
    elif level == 'info':
        file_logger.info(msg)
    elif level == 'warn':
        file_logger.warning(msg)
    elif level == 'error':
        file_logger.error(msg)
    elif level == 'critical':
        file_logger.critical(msg)

bridge = CvBridge()  # Inisialisasi bridge untuk konversi gambar

def validate_yolov12_inference(msg):
    """Validasi message Yolov12Inference agar konsisten."""
    if not msg or not hasattr(msg, 'header'):
        return False
    return True

# ===================== CLASS DETECTION UNTUK PASCA PEMROSESAN =====================
class Detections:
    """Class untuk menyimpan hasil deteksi dengan format yang konsisten."""
    def __init__(self, boxes=None, scores=None, class_ids=None, names=None):
        self.boxes = boxes if boxes is not None else torch.zeros(0, 4)  # Koordinat box [x1, y1, x2, y2]
        self.scores = scores if scores is not None else torch.zeros(0)  # Confidence score
        self.class_ids = class_ids if class_ids is not None else torch.zeros(0)  # ID class
        self.names = names if names is not None else {}  # Mapping ID ke nama class
        
    def cpu(self):
        """Pindahkan semua tensor ke CPU."""
        return Detections(
            self.boxes.cpu() if isinstance(self.boxes, torch.Tensor) else self.boxes,
            self.scores.cpu() if isinstance(self.scores, torch.Tensor) else self.scores,
            self.class_ids.cpu() if isinstance(self.class_ids, torch.Tensor) else self.class_ids,
            self.names
        )
    
    def numpy(self):
        """Konversi semua tensor ke numpy array."""
        return Detections(
            self.boxes.numpy() if isinstance(self.boxes, torch.Tensor) else self.boxes,
            self.scores.numpy() if isinstance(self.scores, torch.Tensor) else self.scores,
            self.class_ids.numpy() if isinstance(self.class_ids, torch.Tensor) else self.class_ids,
            self.names
        )
    
    def to(self, device):
        """Pindahkan semua tensor ke device tertentu."""
        return Detections(
            self.boxes.to(device) if isinstance(self.boxes, torch.Tensor) else self.boxes,
            self.scores.to(device) if isinstance(self.scores, torch.Tensor) else self.scores,
            self.class_ids.to(device) if isinstance(self.class_ids, torch.Tensor) else self.class_ids,
            self.names
        )
    
    def __len__(self):
        """Jumlah deteksi."""
        return len(self.boxes)

class YOLOTensorRT:
    """Class untuk inferensi YOLOv12 dengan TensorRT."""
    def __init__(self, engine_path, confidence_threshold=0.25, device='cuda'):
        self.confidence_threshold = confidence_threshold
        self.device = device
        
        # Setup TensorRT Logger
        self.trt_logger = trt.Logger(trt.Logger.INFO)
        
        # Load TensorRT engine
        log_to_file(f"Loading TensorRT engine from: {engine_path}", level='info')
        
        # Load engine dari file
        with open(engine_path, "rb") as f:
            engine_data = f.read()
            
        # Deserialize engine
        self.runtime = trt.Runtime(self.trt_logger)
        self.engine = self.runtime.deserialize_cuda_engine(engine_data)
        
        if not self.engine:
            log_to_file(f"Failed to load TensorRT engine: {engine_path}", level='error')
            raise RuntimeError(f"Failed to load TensorRT engine: {engine_path}")
            
        # Alokasi buffer untuk input dan output
        self.context = self.engine.create_execution_context()
        self.input_shape = (3, 640, 640)  # Format HWC yang biasa digunakan YOLOv12
        
        # Get bindings
        self.inputs = []
        self.outputs = []
        self.allocations = []
        
        for i in range(self.engine.num_bindings):
            name = self.engine.get_binding_name(i)
            dtype = trt.nptype(self.engine.get_binding_dtype(i))
            shape = self.engine.get_binding_shape(i)
            
            # Alokasi memory
            allocation = cuda.mem_alloc(trt.volume(shape) * np.dtype(dtype).itemsize)
            self.allocations.append(allocation)
            
            if self.engine.binding_is_input(i):
                self.inputs.append({'index': i, 'name': name, 'dtype': dtype, 'shape': shape, 'allocation': allocation})
            else:
                self.outputs.append({'index': i, 'name': name, 'dtype': dtype, 'shape': shape, 'allocation': allocation})
        
        # Informasi input
        input_names = [inp['name'] for inp in self.inputs]
        output_names = [out['name'] for out in self.outputs]
        log_to_file(f"Engine inputs: {input_names}, outputs: {output_names}", level='info')
        
        # Class names
        self.names = {
            0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 
            5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light', 
            10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench', 
            14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow', 
            20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe', 24: 'backpack', 
            25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase', 29: 'frisbee', 
            30: 'skis', 31: 'snowboard', 32: 'sports ball', 33: 'kite', 34: 'baseball bat', 
            35: 'baseball glove', 36: 'skateboard', 37: 'surfboard', 38: 'tennis racket', 
            39: 'bottle', 40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 
            44: 'spoon', 45: 'bowl', 46: 'banana', 47: 'apple', 48: 'sandwich', 
            49: 'orange', 50: 'broccoli', 51: 'carrot', 52: 'hot dog', 53: 'pizza', 
            54: 'donut', 55: 'cake', 56: 'chair', 57: 'couch', 58: 'potted plant', 
            59: 'bed', 60: 'dining table', 61: 'toilet', 62: 'tv', 63: 'laptop', 
            64: 'mouse', 65: 'remote', 66: 'keyboard', 67: 'cell phone', 68: 'microwave', 
            69: 'oven', 70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book', 
            74: 'clock', 75: 'vase', 76: 'scissors', 77: 'teddy bear', 78: 'hair drier', 
            79: 'toothbrush'
        }
        
        # Stream untuk async execution
        self.stream = cuda.Stream()
        
        log_to_file("TensorRT engine initialized successfully", level='info')
    
    def preprocess(self, image):
        """Preprocess image untuk inferensi."""
        # Resize dan padding untuk ukuran input engine
        input_height, input_width = self.input_shape[1:]
        img_height, img_width = image.shape[:2]
        
        # Hitung scale dan padding
        r = min(input_height / img_height, input_width / img_width)
        new_h, new_w = int(img_height * r), int(img_width * r)
        dw, dh = (input_width - new_w) / 2, (input_height - new_h) / 2
        
        # Resize dan padding
        img_resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        img_padded = np.full((input_height, input_width, 3), 114, dtype=np.uint8)
        img_padded[int(dh):int(dh)+new_h, int(dw):int(dw)+new_w, :] = img_resized
        
        # Normalisasi dan transpose (HWC -> CHW)
        img_input = img_padded.transpose((2, 0, 1)).astype(np.float32) / 255.0
        
        # Simpan info scaling untuk postprocess
        self.scale_factors = (r, dw, dh, img_height, img_width)
        
        return img_input
    
    def __call__(self, img):
        """Run inferensi pada image."""
        # Preprocess
        input_data = self.preprocess(img)
        
        # Setup dan copy input data ke GPU
        cuda.memcpy_htod_async(self.inputs[0]['allocation'], input_data, self.stream)
        
        # Run inference
        self.context.execute_async_v2(
            bindings=[allocation for allocation in self.allocations],
            stream_handle=self.stream.handle
        )
        
        # Get output
        output_data = []
        for out in self.outputs:
            # Alokasi host memory
            host_output = np.zeros(trt.volume(out['shape']), dtype=out['dtype'])
            # Copy dari device ke host
            cuda.memcpy_dtoh_async(host_output, out['allocation'], self.stream)
            output_data.append(host_output)
        
        # Sinkronisasi
        self.stream.synchronize()
        
        # Format output sesuai dengan format YOLOv12
        # Biasanya output[0] adalah output deteksi dengan shape [num_dets, 7]
        # di mana setiap baris: [x1, y1, x2, y2, confidence, class_id, _]
        if len(output_data) > 0 and len(output_data[0]) > 0:
            results = output_data[0]
            
            # Reshape jika perlu
            if len(results.shape) == 1:
                results = results.reshape(-1, 7)  # format [x1, y1, x2, y2, conf, cls, _]
            
            # Filter berdasarkan confidence
            mask = results[:, 4] > self.confidence_threshold
            filtered_results = results[mask]
            
            if len(filtered_results) > 0:
                # Extract boxes, scores, class_ids
                boxes = filtered_results[:, :4]  # x1, y1, x2, y2
                scores = filtered_results[:, 4]  # confidence scores
                class_ids = filtered_results[:, 5].astype(np.int32)  # class ids
                
                # Rescale boxes ke ukuran original
                r, dw, dh, img_height, img_width = self.scale_factors
                boxes[:, 0] = (boxes[:, 0] - dw) / r  # x1
                boxes[:, 1] = (boxes[:, 1] - dh) / r  # y1
                boxes[:, 2] = (boxes[:, 2] - dw) / r  # x2
                boxes[:, 3] = (boxes[:, 3] - dh) / r  # y2
                
                # Clip boxes ke frame
                boxes[:, 0].clip(0, img_width, out=boxes[:, 0])
                boxes[:, 1].clip(0, img_height, out=boxes[:, 1])
                boxes[:, 2].clip(0, img_width, out=boxes[:, 2])
                boxes[:, 3].clip(0, img_height, out=boxes[:, 3])
                
                # Konversi ke torch tensor untuk kompatibilitas
                boxes_tensor = torch.from_numpy(boxes)
                scores_tensor = torch.from_numpy(scores)
                class_ids_tensor = torch.from_numpy(class_ids)
                
                # Buat hasil dengan format yang sama seperti YOLO
                results = [Detections(boxes_tensor, scores_tensor, class_ids_tensor, self.names)]
                
                # Tambahkan plot function agar kompatibel dengan ultralytics API
                results[0].plot = lambda **kwargs: self.plot_results(img, results[0], **kwargs)
                
                return results
        
        # Return empty results
        empty_results = [Detections(
            torch.zeros(0, 4),
            torch.zeros(0),
            torch.zeros(0, dtype=torch.int32),
            self.names
        )]
        empty_results[0].plot = lambda **kwargs: img.copy()
        return empty_results
    
    def plot_results(self, img, det, line_thickness=2, font_scale=0.5):
        """Plot hasil deteksi ke image."""
        img_copy = img.copy()
        boxes = det.boxes.cpu().numpy() if isinstance(det.boxes, torch.Tensor) else det.boxes
        scores = det.scores.cpu().numpy() if isinstance(det.scores, torch.Tensor) else det.scores
        class_ids = det.class_ids.cpu().numpy() if isinstance(det.class_ids, torch.Tensor) else det.class_ids
        
        for i, box in enumerate(boxes):
            # Get box coordinates
            x1, y1, x2, y2 = map(int, box)
            
            # Get class name and score
            class_id = int(class_ids[i])
            class_name = det.names.get(class_id, f"class_{class_id}")
            score = scores[i]
            
            # Draw box
            color = (0, 255, 0)  # Green
            cv2.rectangle(img_copy, (x1, y1), (x2, y2), color, thickness=line_thickness)
            
            # Draw label
            label = f"{class_name} {score:.2f}"
            text_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, line_thickness)
            cv2.rectangle(img_copy, (x1, y1 - text_size[1] - 5), (x1 + text_size[0], y1), color, -1)
            cv2.putText(img_copy, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), line_thickness)
        
        return img_copy

class CameraSubscriberTRT(Node):  # Node deteksi YOLOv12 TensorRT, FULL OOP
    def __init__(self):
        super().__init__('camera_subscriber_trt')  # Inisialisasi node dengan nama unik
        try:
            # ===================== PARAMETERISASI NODE =====================
            self.declare_parameter('model_path', os.path.expanduser('~/jezzy/huskybot/src/huskybot_recognition/huskybot_recognition/scripts/yolo12x.engine'))  # Path default model YOLOv12 TensorRT
            self.declare_parameter('confidence_threshold', 0.25)  # Threshold confidence default
            self.declare_parameter('log_stats', True)  # Logging statistik deteksi ke file
            self.declare_parameter('log_stats_path', os.path.expanduser('~/huskybot_detection_log/yolov12_trt_stats.csv'))  # Path file statistik
            self.declare_parameter('output_topic', '/inference_result')  # Parameterisasi topic output annotated
            self.declare_parameter('inference_topic', '/Yolov12_Inference')  # Parameterisasi topic hasil deteksi
            self.declare_parameter('class_filter', '')  # Filter class deteksi (kosong = semua)
            self.declare_parameter('min_confidence', 0.0)  # Threshold confidence minimum

            model_path = self.get_parameter('model_path').get_parameter_value().string_value  # Ambil path model dari parameter
            self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value  # Ambil threshold dari parameter
            self.log_stats = self.get_parameter('log_stats').get_parameter_value().bool_value  # Ambil flag logging statistik
            self.log_stats_path = self.get_parameter('log_stats_path').get_parameter_value().string_value  # Ambil path file statistik
            output_topic = self.get_parameter('output_topic').get_parameter_value().string_value  # Ambil topic output annotated
            inference_topic = self.get_parameter('inference_topic').get_parameter_value().string_value  # Ambil topic hasil deteksi
            self.class_filter = self.get_parameter('class_filter').get_parameter_value().string_value  # Ambil filter class
            self.min_confidence = self.get_parameter('min_confidence').get_parameter_value().double_value  # Ambil threshold confidence

            self.get_logger().info(f"Parameter: model_path={model_path}, confidence_threshold={self.confidence_threshold}, log_stats={self.log_stats}, log_stats_path={self.log_stats_path}, output_topic={output_topic}, inference_topic={inference_topic}, class_filter={self.class_filter}, min_confidence={self.min_confidence}")  # Log parameter ke terminal
            log_to_file(f"Parameter: model_path={model_path}, confidence_threshold={self.confidence_threshold}, log_stats={self.log_stats}, log_stats_path={self.log_stats_path}, output_topic={output_topic}, inference_topic={inference_topic}, class_filter={self.class_filter}, min_confidence={self.min_confidence}")  # Log parameter ke file

            # ===================== VALIDASI FILE MODEL YOLO =====================
            if not os.path.isfile(model_path):  # Validasi file model YOLOv12
                self.get_logger().error(f"Model YOLOv12 TensorRT tidak ditemukan: {model_path}")
                log_to_file(f"Model YOLOv12 TensorRT tidak ditemukan: {model_path}", level='error')
                sys.exit(1)

            # ===================== LOAD MODEL YOLOv12 TENSORRT =====================
            try:
                # Initialize TensorRT model
                self.model = YOLOTensorRT(model_path, confidence_threshold=self.confidence_threshold)
                self.get_logger().info("YOLOv12 TensorRT model loaded successfully.")
                log_to_file("YOLOv12 TensorRT model loaded successfully.")
            except Exception as e:
                self.get_logger().error(f"Gagal load model YOLOv12 TensorRT: {e}")
                log_to_file(f"Gagal load model YOLOv12 TensorRT: {e}", level='error')
                sys.exit(2)

            # ===================== PUBLISHER & SUBSCRIBER (RETRY) =====================
            self.yolov12_pub = self._create_publisher_with_retry(Yolov12Inference, inference_topic, 1)  # Publisher hasil deteksi
            self.img_pub = self._create_publisher_with_retry(Image, output_topic, 1)  # Publisher gambar dengan annotasi
            self._create_subscription_with_retry(Image, '/camera_front/image_raw', lambda msg: self.camera_callback(msg, 'camera_front'), 10)  # Subscribe kamera depan
            
            # ===================== STATISTIK THREAD-SAFE =====================
            self.stats = {
                'total_images': 0,
                'total_detections': 0,
                'per_class': {}
            }
            self.stats_lock = threading.Lock()  # Lock untuk thread-safety
            
            # ===================== LOGGING STATISTIK KE FILE CSV =====================
            if self.log_stats:  # Cek apakah logging statistik aktif
                try:
                    os.makedirs(os.path.dirname(self.log_stats_path), exist_ok=True)  # Buat folder jika belum ada
                    self.stats_file = open(self.log_stats_path, 'a', newline='')  # Buka file statistik (append)
                    self.stats_writer = csv.writer(self.stats_file)  # CSV writer
                    
                    # Tulis header jika file baru (size = 0)
                    if os.path.getsize(self.log_stats_path) == 0:  # Check if file is empty
                        self.stats_writer.writerow(['timestamp', 'camera', 'detections', 'class_counts'])
                    
                    self.get_logger().info(f"Logging detection stats to: {self.log_stats_path}")
                    log_to_file(f"Logging detection stats to: {self.log_stats_path}")
                except Exception as e:
                    self.get_logger().error(f"Error membuka file statistik: {e}")
                    log_to_file(f"Error membuka file statistik: {e}", level='error')
                    self.log_stats = False
        except Exception as e:
            self.get_logger().error(f"Error initializing CameraSubscriberTRT: {e}\n{traceback.format_exc()}")
            log_to_file(f"Error initializing CameraSubscriberTRT: {e}\n{traceback.format_exc()}", level='error')
            sys.exit(99)
    
    def _create_publisher_with_retry(self, msg_type, topic, queue_size, max_retry=5):
        """Buat publisher dengan retry jika gagal."""
        retry_count = 0
        while retry_count < max_retry:
            try:
                pub = self.create_publisher(msg_type, topic, queue_size)
                self.get_logger().info(f"Publisher created for topic: {topic}")
                return pub
            except Exception as e:
                retry_count += 1
                self.get_logger().warn(f"Failed to create publisher for {topic}, retry {retry_count}/{max_retry}: {e}")
                time.sleep(1.0)  # Tunggu sebentar sebelum retry
        
        self.get_logger().error(f"Failed to create publisher for {topic} after {max_retry} retries")
        return None
    
    def _create_subscription_with_retry(self, msg_type, topic, callback, queue_size, max_retry=5):
        """Buat subscription dengan retry jika gagal."""
        retry_count = 0
        while retry_count < max_retry:
            try:
                sub = self.create_subscription(msg_type, topic, callback, queue_size)
                self.get_logger().info(f"Subscription created for topic: {topic}")
                return sub
            except Exception as e:
                retry_count += 1
                self.get_logger().warn(f"Failed to create subscription for {topic}, retry {retry_count}/{max_retry}: {e}")
                time.sleep(1.0)  # Tunggu sebentar sebelum retry
        
        self.get_logger().error(f"Failed to create subscription for {topic} after {max_retry} retries")
        return None
    
    def camera_callback(self, data, cam_name):
        try:
            img = bridge.imgmsg_to_cv2(data, "bgr8")  # Konversi ROS Image ke OpenCV BGR
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            log_to_file(f"CV Bridge error: {e}", level='error')
            return
        except Exception as e:
            self.get_logger().error(f"Error konversi gambar kamera: {e}")
            log_to_file(f"Error konversi gambar kamera: {e}", level='error')
            return

        try:
            results = self.model(img)  # Inference YOLOv12 TensorRT
        except Exception as e:
            self.get_logger().error(f"YOLOv12 TensorRT inference error: {e}\n{traceback.format_exc()}")
            log_to_file(f"YOLOv12 TensorRT inference error: {e}\n{traceback.format_exc()}", level='error')
            return

        yolov12_inference = Yolov12Inference()
        yolov12_inference.header.frame_id = cam_name
        yolov12_inference.header.stamp = self.get_clock().now().to_msg()
        yolov12_inference.camera_name = cam_name
        yolov12_inference.frame_type = "raw"
        yolov12_inference.task = "detect"
        yolov12_inference.note = ""
        yolov12_inference.yolov12_inference = []

        num_detections = 0
        class_counts = {}

        for r in results:
            boxes = r.boxes if hasattr(r, 'boxes') else []
            for box in boxes:
                try:
                    conf = float(box.scores.item())
                    # ===================== FILTER CLASS & CONFIDENCE =====================
                    if self.class_filter and self.model.names[int(box.class_ids.item())].lower() != self.class_filter.strip().lower():
                        continue  # Skip jika class tidak sesuai filter
                    if conf < max(self.confidence_threshold, self.min_confidence):
                        continue
                    b = box.boxes.cpu().numpy()[0] if hasattr(box, 'boxes') else None
                    if b is None:
                        continue
                    c = int(box.class_ids.item())
                    class_name = self.model.names[c]
                    inference_result = InferenceResult()
                    inference_result.class_name = class_name
                    inference_result.confidence = conf
                    inference_result.top = int(b[1])
                    inference_result.left = int(b[0])
                    inference_result.bottom = int(b[3])
                    inference_result.right = int(b[2])
                    yolov12_inference.yolov12_inference.append(inference_result)
                    num_detections += 1
                    class_counts[class_name] = class_counts.get(class_name, 0) + 1
                except Exception as e:
                    self.get_logger().warn(f"Error parsing detection result: {e}")
                    log_to_file(f"Error parsing detection result: {e}", level='warn')

        # ===================== STATISTIK DETEKSI (THREAD-SAFE) =====================
        with self.stats_lock:
            self.stats['total_images'] += 1
            self.stats['total_detections'] += num_detections
            for cname, cnt in class_counts.items():
                self.stats['per_class'][cname] = self.stats['per_class'].get(cname, 0) + cnt
            if self.log_stats:
                try:
                    self.stats_writer.writerow([
                        time.strftime('%Y-%m-%d %H:%M:%S'),
                        cam_name,
                        num_detections,
                        dict(class_counts)
                    ])
                    self.stats_file.flush()
                except Exception as e:
                    self.get_logger().warn(f"Error writing stats to file: {e}")
                    log_to_file(f"Error writing stats to file: {e}", level='warn')

        # ===================== PUBLISH HASIL DETEKSI =====================
        try:
            self.yolov12_pub.publish(yolov12_inference)
            self.get_logger().debug(f"Published {num_detections} detections")
        except Exception as e:
            self.get_logger().error(f"Error publishing detection results: {e}")
            log_to_file(f"Error publishing detection results: {e}", level='error')
        
        # ===================== PUBLISH GAMBAR DENGAN ANNOTASI =====================
        try:
            annotated_img = results[0].plot()  # Plot hasil deteksi ke image
            annotated_msg = bridge.cv2_to_imgmsg(annotated_img, "bgr8")
            annotated_msg.header = data.header
            self.img_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing annotated image: {e}")
            log_to_file(f"Error publishing annotated image: {e}", level='error')
    
    def destroy_node(self):
        try:
            if self.log_stats and hasattr(self, 'stats_file'):
                self.stats_file.close()
        except Exception as e:
            self.get_logger().warn(f"Error closing stats file: {e}")
            log_to_file(f"Error closing stats file: {e}", level='warn')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        camera_subscriber = CameraSubscriberTRT()
        rclpy.spin(camera_subscriber)
    except Exception as e:
        print(f"[ERROR] {e}\n{traceback.format_exc()}")
        log_to_file(f"Exception utama: {e}\n{traceback.format_exc()}", level='error')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()