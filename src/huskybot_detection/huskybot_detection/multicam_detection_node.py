#!/usr/bin/env python3  # Interpreter Python3 (wajib untuk ROS2 node)
# -*- coding: utf-8 -*-  # Encoding UTF-8 (wajib untuk support karakter non-ASCII)

# File: multicam_detection_node.py - Node deteksi multicam YOLOv12 untuk Huskybot
# Node ini menerima image dari 6 kamera Arducam IMX477 (hexagonal) dan publish hasil deteksi ke topic /detection
# Siap untuk ROS2 Humble, Gazebo, Jetson Orin, Clearpath Husky A200, multi-robot, audit trail

import os  # Operasi file dan path
import sys  # Akses sys.stderr, sys.exit
import time  # Timestamp, delay, log
import traceback  # Print stack trace saat exception
import logging  # Logging ke file dan terminal
import platform  # Deteksi hardware/OS
from threading import Lock  # Thread safety di callback paralel

import rclpy  # Library utama ROS2 Python
from rclpy.node import Node  # Base class node ROS2
from rclpy.exceptions import ParameterNotDeclaredException  # Exception parameter tidak ditemukan
from rclpy.parameter import Parameter  # Kelas Parameter ROS2
from rcl_interfaces.msg import ParameterDescriptor, ParameterType  # Deskriptor parameter
from rcl_interfaces.msg import SetParametersResult  # Return value untuk parameter callback

from sensor_msgs.msg import Image  # Message ROS2 untuk image kamera
from std_msgs.msg import Header  # Header standar ROS2
from std_srvs.srv import Trigger  # Service type untuk restart_model dan health check
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue  # Diagnostics ROS2

from cv_bridge import CvBridge, CvBridgeError  # Konversi ROS Image <-> OpenCV
import numpy as np  # Array/matrix untuk image processing
import cv2  # OpenCV untuk image processing/visualisasi
from yolov12_msgs.msg import InferenceResult, Yolov12Inference  # Custom message hasil deteksi YOLOv12

# ===================== DETECTOR LIBRARY (YOLOv12) DENGAN FALLBACK =====================
try:
    from ultralytics import YOLO  # Library YOLOv12 (pastikan sudah install ultralytics>=v12)
    ULTRALYTICS_AVAILABLE = True  # Flag ketersediaan ultralytics
except ImportError:
    ULTRALYTICS_AVAILABLE = False  # Set flag False jika import error

LOG_DIR = os.path.expanduser('~/huskybot_detection_log')  # Directory log (default di home)
DEFAULT_CONFIDENCE_THRESHOLD = 0.25  # Threshold confidence default
JETSON_PLATFORMS = ['aarch64', 'arm64']  # Platform Jetson
DETECTION_TOPIC = '/detection'  # Topic hasil deteksi
DIAGNOSTIC_TOPIC = '/diagnostics'  # Topic diagnostics

class MultiCamDetectionNode(Node):  # Node deteksi multicam YOLOv12, FULL OOP
    def __init__(self):
        super().__init__('multicam_detection')  # Inisialisasi node ROS2
        self._setup_logging()  # Setup logging ke file dan terminal
        self.get_logger().info("Initializing MultiCam YOLOv12 Detection Node...")  # Log startup

        self._declare_parameters()  # Declare semua parameter
        self._load_parameters()  # Load parameter dari server

        self.bridge = CvBridge()  # Bridge konversi image
        self.mutex = Lock()  # Mutex untuk thread safety
        self.images = [None] * self.cam_count  # Buffer image
        self.last_frame_time = [None] * self.cam_count  # Timestamp terakhir
        self.detection_counts = [0] * self.cam_count  # Counter deteksi
        self.inference_times = [0.0] * self.cam_count  # Waktu inference
        self.is_initialized = False  # Flag init
        self.model = None  # Model YOLOv12
        self.running = True  # Flag running

        self._detect_platform()  # Deteksi Jetson/CUDA
        self._create_publishers()  # Publisher deteksi/diagnostics
        self._create_services()  # Service restart_model/get_status
        self._load_model()  # Load YOLOv12 model
        self._create_subscribers()  # Subscriber kamera
        self._create_timers()  # Timer deteksi/diagnostics

        self.is_initialized = True  # Set flag init selesai
        self.get_logger().info(f"MultiCam YOLOv12 Detection Node initialized with {self.cam_count} cameras")  # Log selesai init

    def _setup_logging(self):
        # Setup logging ke file dan terminal, fallback ke /tmp jika gagal
        try:
            log_dir = LOG_DIR
            try:
                if not os.path.exists(log_dir):
                    os.makedirs(log_dir, exist_ok=True)
            except Exception:
                log_dir = '/tmp'
                if not os.path.exists(log_dir):
                    os.makedirs(log_dir, exist_ok=True)
            log_file = os.path.join(log_dir, f"huskybot_detection_{time.strftime('%Y%m%d')}.log")
            logging.basicConfig(
                level=logging.INFO,
                format='%(asctime)s [%(levelname)s] %(message)s',
                handlers=[
                    logging.FileHandler(log_file),
                    logging.StreamHandler(sys.stdout)
                ]
            )
            logging.info("Logging system initialized")
        except Exception as e:
            print(f"Error setting up logging: {e}", file=sys.stderr)

    def _declare_parameters(self):
        # Deklarasi semua parameter node (wajib agar bisa diubah via launch file)
        self.declare_parameter('cam_count', 6, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Number of cameras in the hexagonal array (1-12)',
            integer_range=[{'from_value': 1, 'to_value': 12, 'step': 1}]
        ))
        self.declare_parameter('model_path', "yolo12x.engine", ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Path to YOLOv12 model file (.pt, .onnx, or .engine)'
        ))
        self.declare_parameter('camera_topics', [
            '/camera_front/image_raw',
            '/camera_right/image_raw',
            '/camera_rear_right/image_raw',
            '/camera_rear/image_raw',
            '/camera_left/image_raw',
            '/camera_front_left/image_raw'
        ], ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING_ARRAY,
            description='List of camera topic names to subscribe to'
        ))
        self.declare_parameter('conf_thres', DEFAULT_CONFIDENCE_THRESHOLD, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Confidence threshold for filtering detections (0.0-1.0)',
            floating_point_range=[{'from_value': 0.0, 'to_value': 1.0, 'step': 0.01}]
        ))
        self.declare_parameter('class_filter', [], ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER_ARRAY,
            description='List of class IDs to keep (empty for all classes)'
        ))
        self.declare_parameter('visualization_enabled', True, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Enable/disable OpenCV visualization of detection results'
        ))
        self.declare_parameter('log_to_file', True, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Enable/disable logging to file'
        ))
        self.declare_parameter('log_level', 'info', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Log level (debug, info, warning, error, critical)'
        ))

    def _load_parameters(self):
        # Load semua parameter dari server, validasi, dan fallback jika error
        try:
            self.cam_count = self.get_parameter('cam_count').value
            if not 1 <= self.cam_count <= 12:
                self.get_logger().warning(f"Invalid cam_count {self.cam_count}, using default (6)")
                self.cam_count = 6
            self.model_path = self.get_parameter('model_path').value
            self.camera_topics = self.get_parameter('camera_topics').value
            if len(self.camera_topics) < self.cam_count:
                self.get_logger().warning(
                    f"Not enough camera topics ({len(self.camera_topics)}) for cam_count ({self.cam_count})"
                )
                self.cam_count = len(self.camera_topics)
            self.conf_thres = self.get_parameter('conf_thres').value
            self.class_filter = self.get_parameter('class_filter').value
            self.visualization_enabled = self.get_parameter('visualization_enabled').value
            self.log_to_file = self.get_parameter('log_to_file').value
            log_level = self.get_parameter('log_level').value.lower()
            if log_level == 'debug':
                logging.getLogger().setLevel(logging.DEBUG)
            elif log_level == 'info':
                logging.getLogger().setLevel(logging.INFO)
            elif log_level == 'warning':
                logging.getLogger().setLevel(logging.WARNING)
            elif log_level == 'error':
                logging.getLogger().setLevel(logging.ERROR)
            elif log_level == 'critical':
                logging.getLogger().setLevel(logging.CRITICAL)
            else:
                self.get_logger().warning(f"Invalid log_level '{log_level}', using INFO")
                logging.getLogger().setLevel(logging.INFO)
        except ParameterNotDeclaredException as e:
            self.get_logger().error(f"Parameter error: {e}")
            raise
        except Exception as e:
            self.get_logger().error(f"Error loading parameters: {e}")
            self.get_logger().error(traceback.format_exc())
            raise

    def _detect_platform(self):
        # Deteksi platform Jetson/CUDA untuk optimasi model
        try:
            self.is_jetson = platform.machine() in JETSON_PLATFORMS
            if self.is_jetson:
                self.get_logger().info("Detected Jetson platform, using hardware acceleration")
                try:
                    import torch
                    self.cuda_available = torch.cuda.is_available()
                    if self.cuda_available:
                        self.get_logger().info(f"CUDA available: {torch.cuda.get_device_name(0)}")
                    else:
                        self.get_logger().warning("CUDA not available on Jetson device")
                except ImportError:
                    self.get_logger().warning("PyTorch not installed, can't check CUDA availability")
                    self.cuda_available = False
            else:
                self.get_logger().info("Running on non-Jetson platform")
                self.cuda_available = False
        except Exception as e:
            self.get_logger().error(f"Error detecting platform: {e}")
            self.is_jetson = False
            self.cuda_available = False

    def _create_publishers(self):
        # Buat publisher untuk hasil deteksi dan diagnostics
        try:
            self.publisher = self.create_publisher(Yolov12Inference, DETECTION_TOPIC, 10)
            self.diagnostic_pub = self.create_publisher(DiagnosticArray, DIAGNOSTIC_TOPIC, 10)
            self.get_logger().info(f"Publishers created: {DETECTION_TOPIC}, {DIAGNOSTIC_TOPIC}")
        except Exception as e:
            self.get_logger().error(f"Error creating publishers: {e}")
            raise

    def _create_services(self):
        # Buat service untuk restart model dan get status
        try:
            self.restart_srv = self.create_service(Trigger, 'restart_model', self.restart_model_callback)
            self.status_srv = self.create_service(Trigger, 'get_status', self.get_status_callback)
            self.get_logger().info("Services created: restart_model, get_status")
        except Exception as e:
            self.get_logger().error(f"Error creating services: {e}")
            self.get_logger().error(traceback.format_exc())

    def _load_model(self):
        # Load YOLOv12 model, fallback ke .pt jika .engine/.onnx gagal
        if not ULTRALYTICS_AVAILABLE:
            self.get_logger().error("Ultralytics not available. Cannot load YOLOv12 model.")
            raise ImportError("Required package 'ultralytics' not found")
        model_loaded = False
        original_path = self.model_path
        try:
            self.get_logger().info(f"Loading YOLOv12 model from: {self.model_path}")
            if not os.path.exists(self.model_path):
                self.get_logger().warning(f"Model file not found: {self.model_path}")
                possible_paths = [
                    self.model_path,
                    os.path.join(os.path.expanduser('~'), self.model_path),
                    os.path.join('/opt/models', self.model_path),
                    os.path.join(os.getcwd(), self.model_path),
                    os.path.join(os.getcwd(), 'models', self.model_path),
                ]
                base_name = os.path.splitext(self.model_path)[0]
                possible_paths.extend([
                    f"{base_name}.pt",
                    f"{base_name}.onnx",
                    f"{base_name}.engine"
                ])
                for path in possible_paths:
                    if os.path.exists(path):
                        self.get_logger().info(f"Found model at: {path}")
                        self.model_path = path
                        break
            self.model = YOLO(self.model_path, task="detect")
            model_loaded = True
            self.get_logger().info(f"Successfully loaded YOLOv12 model: {self.model_path}")
            if hasattr(self.model, 'info'):
                self.get_logger().info(f"Model info: {self.model.info}")
            self.get_logger().info(f"Model task: detect")
        except Exception as e:
            self.get_logger().error(f"Error loading YOLOv12 model: {e}")
            self.get_logger().error(traceback.format_exc())
            if not model_loaded and original_path.endswith(('.engine', '.onnx')):
                try:
                    fallback_path = original_path.rsplit('.', 1)[0] + '.pt'
                    self.get_logger().info(f"Attempting fallback to PyTorch model: {fallback_path}")
                    if os.path.exists(fallback_path):
                        self.model = YOLO(fallback_path, task="detect")
                        self.model_path = fallback_path
                        model_loaded = True
                        self.get_logger().info(f"Successfully loaded fallback model: {fallback_path}")
                    else:
                        self.get_logger().error(f"Fallback model not found: {fallback_path}")
                except Exception as e2:
                    self.get_logger().error(f"Error loading fallback model: {e2}")
            if not model_loaded:
                self.get_logger().error("Could not load any YOLOv12 model. Exiting.")
                raise

    def _create_subscribers(self):
        # Buat subscriber untuk semua kamera, error handling jika topic tidak ada
        try:
            if not self.camera_topics or len(self.camera_topics) == 0:
                self.get_logger().error("No camera topics specified")
                raise ValueError("No camera topics to subscribe to")
            active_topics = self.camera_topics[:self.cam_count]
            self.get_logger().info(f"Creating subscribers for {len(active_topics)} camera topics")
            self.subs = []
            for i, topic in enumerate(active_topics):
                try:
                    sub = self.create_subscription(
                        Image,
                        topic,
                        lambda msg, idx=i: self.image_callback(msg, idx),
                        10
                    )
                    self.subs.append(sub)
                    self.get_logger().info(f"Subscribed to camera topic: {topic} (idx={i})")
                except Exception as e:
                    self.get_logger().error(f"Error subscribing to {topic}: {e}")
            if len(self.subs) == 0:
                self.get_logger().error("Failed to create any camera subscribers")
                raise RuntimeError("No camera subscriptions created")
            if len(self.subs) < self.cam_count:
                self.get_logger().warning(f"Only {len(self.subs)} camera topics subscribed out of {self.cam_count} requested")
                self.cam_count = len(self.subs)
                self.images = [None] * self.cam_count
                self.last_frame_time = [None] * self.cam_count
                self.detection_counts = [0] * self.cam_count
                self.inference_times = [0.0] * self.cam_count
        except Exception as e:
            self.get_logger().error(f"Error setting up camera subscribers: {e}")
            self.get_logger().error(traceback.format_exc())
            raise

    def _create_timers(self):
        # Buat timer untuk proses deteksi dan diagnostics
        try:
            self.timer = self.create_timer(0.2, self.process_images)
            self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)
            self.get_logger().info("Timers created for processing and diagnostics")
        except Exception as e:
            self.get_logger().error(f"Error creating timers: {e}")
            raise

    def image_callback(self, msg, idx):
        # Callback untuk setiap image kamera, simpan ke buffer
        try:
            with self.mutex:
                self.last_frame_time[idx] = self.get_clock().now()
                self.images[idx] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warning(f"Error converting image from topic {self.camera_topics[idx]}: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in image callback for camera {idx}: {e}")

    def process_images(self):
        # Proses deteksi untuk semua kamera, publish hasil ke /detection
        if not self.running or not self.is_initialized:
            return
        with self.mutex:
            images_copy = self.images.copy()
        available_count = sum(1 for img in images_copy if img is not None)
        if available_count == 0:
            return
        try:
            for idx, img in enumerate(images_copy):
                if img is not None:
                    start_time = time.time()
                    try:
                        results = self.model(img, verbose=False, conf=self.conf_thres)
                        infer_time = time.time() - start_time
                        self.inference_times[idx] = infer_time
                        if results and self.class_filter:
                            results = [result for result in results if any(cls in self.class_filter for cls in result.boxes.cls)]
                        self.detection_counts[idx] = len(results[0].boxes) if results else 0
                        if self.visualization_enabled:
                            # TODO: Annotate image with bounding boxes (implement as needed)
                            pass
                        self.get_logger().debug(
                            f"Camera {idx}: {self.detection_counts[idx]} detections in {infer_time:.3f}s"
                        )
                        self.publish_results(results, f"Camera_{idx}")
                    except Exception as e:
                        self.get_logger().error(f"Error processing image from camera {idx}: {e}")
                        self.get_logger().error(traceback.format_exc())
            if self.visualization_enabled:
                self.visualize_results(images_copy)
        except Exception as e:
            self.get_logger().error(f"Error in process_images: {e}")
            self.get_logger().error(traceback.format_exc())

    def publish_results(self, results, camera_name):
        # Publish hasil deteksi ke topic /detection (Yolov12Inference)
        if not results:
            return
        try:
            msg = Yolov12Inference()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = camera_name
            msg.camera_name = camera_name
            msg.frame_type = "raw"
            msg.task = "detect"
            msg.note = ""
            msg.yolov12_inference = []
            try:
                for box in results[0].boxes:
                    det = InferenceResult()
                    try:
                        det.class_name = str(box.cls)  # Ganti sesuai format YOLOv12
                    except (AttributeError, TypeError):
                        det.class_name = ""
                    try:
                        det.confidence = float(box.conf)
                    except (AttributeError, TypeError):
                        det.confidence = 0.0
                    try:
                        det.top = int(box.xyxy[1])
                        det.left = int(box.xyxy[0])
                        det.bottom = int(box.xyxy[3])
                        det.right = int(box.xyxy[2])
                    except Exception:
                        det.top = det.left = det.bottom = det.right = 0
                    det.track_id = -1
                    det.obb_angle = -1
                    det.mask_indices = []
                    msg.yolov12_inference.append(det)
            except Exception as e:
                self.get_logger().error(f"Error parsing detection results: {e}")
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing detection results: {e}")
            self.get_logger().error(traceback.format_exc())

    def visualize_results(self, images):
        # Visualisasi hasil deteksi semua kamera (side-by-side)
        if not self.visualization_enabled:
            return
        try:
            valid_images = [img for img in images if img is not None]
            if not valid_images:
                return
            target_height = 240
            resized_images = []
            for image in valid_images:
                h, w = image.shape[:2]
                scale = target_height / h
                resized = cv2.resize(image, (int(w * scale), target_height))
                resized_images.append(resized)
            border_thickness = 5
            bordered_images = []
            for idx, img in enumerate(resized_images):
                bordered = cv2.copyMakeBorder(img, 0, 0, 0, border_thickness, cv2.BORDER_CONSTANT, value=(0, 0, 0))
                bordered_images.append(bordered)
            try:
                vis = np.hstack(bordered_images)
                cv2.imshow("MultiCam Detection Results", vis)
                cv2.waitKey(1)
            except cv2.error as e:
                self.get_logger().warning(f"OpenCV visualization error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in visualization: {e}")
            if 'Cannot connect to X server' in str(e):
                self.get_logger().warning("OpenCV GUI not available (headless mode)")

    def publish_diagnostics(self):
        # Publish diagnostics ke /diagnostics untuk monitoring health node
        try:
            diag_msg = DiagnosticArray()
            diag_msg.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "huskybot_detection"
            status.hardware_id = platform.node()
            if not self.is_initialized or self.model is None:
                status.level = DiagnosticStatus.ERROR
                status.message = "Node not initialized or model not loaded"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "Node running"
            status.values.append(KeyValue(key="model_path", value=self.model_path))
            status.values.append(KeyValue(key="camera_count", value=str(self.cam_count)))
            status.values.append(KeyValue(key="platform", value="Jetson" if self.is_jetson else "Generic"))
            for i, count in enumerate(self.detection_counts):
                status.values.append(KeyValue(key=f"camera_{i}_detections", value=str(count)))
            for i, infer_time in enumerate(self.inference_times):
                status.values.append(KeyValue(key=f"camera_{i}_inference_time", value=f"{infer_time:.3f}s"))
            diag_msg.status.append(status)
            self.diagnostic_pub.publish(diag_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing diagnostics: {e}")

    def restart_model_callback(self, request, response):
        # Service callback untuk restart model YOLOv12
        try:
            self.get_logger().info("Restarting YOLOv12 model")
            self._load_model()
            response.success = True
            response.message = f"Model restarted successfully: {self.model_path}"
            return response
        except Exception as e:
            self.get_logger().error(f"Error restarting model: {e}")
            response.success = False
            response.message = f"Failed to restart model: {str(e)}"
            return response

    def get_status_callback(self, request, response):
        # Service callback untuk get status node (health check)
        try:
            status_msg = f"MultiCamDetectionNode Status:\n"
            status_msg += f"- Camera count: {self.cam_count}\n"
            status_msg += f"- Model path: {self.model_path}\n"
            status_msg += f"- Platform: {platform.platform()}\n"
            status_msg += f"- Is Jetson: {self.is_jetson}\n"
            status_msg += "- Detection counts:\n"
            for i, count in enumerate(self.detection_counts):
                status_msg += f"  Camera {i}: {count}\n"
            status_msg += "- Inference times:\n"
            for i, infer_time in enumerate(self.inference_times):
                status_msg += f"  Camera {i}: {infer_time:.3f}s\n"
            response.success = True
            response.message = status_msg
            return response
        except Exception as e:
            self.get_logger().error(f"Error getting status: {e}")
            response.success = False
            response.message = f"Failed to get status: {str(e)}"
            return response

    def on_shutdown(self):
        # Shutdown node dengan aman, tutup window OpenCV dan logging
        self.running = False
        self.get_logger().info("Shutting down MultiCamDetectionNode...")
        if self.visualization_enabled:
            try:
                cv2.destroyAllWindows()
            except Exception as e:
                self.get_logger().warning(f"Error closing OpenCV windows: {e}")
        try:
            logging.shutdown()
        except Exception as e:
            print(f"Error shutting down logging: {e}")

def main(args=None):
    # Entry point ROS2 node
    rclpy.init(args=args)
    node = None
    try:
        node = MultiCamDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt, shutting down...")
        if node:
            node.get_logger().info('KeyboardInterrupt, shutting down node.')
    except Exception as e:
        print(f"Fatal error: {e}")
        if node:
            node.get_logger().error(f"Fatal error: {e}")
            node.get_logger().error(traceback.format_exc())
    finally:
        if node:
            node.on_shutdown()
            try:
                node.destroy_node()
            except Exception as e:
                print(f"Error destroying node: {e}")
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"Error during rclpy shutdown: {e}")
        print("Multicam detection node shutdown complete.")

if __name__ == "__main__":
    main()  # Jalankan main jika file dieksekusi langsung

# ===================== PENJELASAN TAMBAHAN =====================
# File ini mengimplementasikan node deteksi objek multicamera dengan YOLOv12 untuk robot Huskybot.
# Node menerima image dari 6 kamera dalam konfigurasi hexagonal, menjalankan deteksi objek pada
# setiap image, dan mempublish hasil ke topic /detection dengan message type Yolov12Inference.
# Node sudah FULL OOP, dengan error handling komprehensif di setiap bagian.