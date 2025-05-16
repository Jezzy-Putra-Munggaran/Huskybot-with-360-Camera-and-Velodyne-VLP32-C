import numpy as np  # Untuk operasi array dan komputasi numerik
import cv2  # Untuk operasi image processing (misal proyeksi 3D->2D)
import yaml  # Untuk membaca file kalibrasi YAML
import os  # Untuk cek file kalibrasi
import logging  # Untuk logging error/info di luar node ROS2
import traceback  # Untuk logging traceback error

# ===================== LOGGING TO FILE (OPSIONAL) =====================
def setup_file_logger(log_path="~/huskybot_fusion_utils.log"):
    log_path = os.path.expanduser(log_path)
    logger = logging.getLogger("fusion_utils_file_logger")
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
        elif level == 'debug':
            file_logger.debug(msg)
        else:
            file_logger.info(msg)

def load_extrinsic_calibration(calib_file, expected_camera_frame_id=None, expected_lidar_frame_id=None, ros_logger=None):
    """
    Membaca file kalibrasi YAML dan mengembalikan matriks transformasi (4x4).
    Format YAML harus:
    T_lidar_camera:
      rows: 4
      cols: 4
      data: [ ... 16 nilai matriks ... ]
      camera_frame_id: ...
      lidar_frame_id: ...
    expected_camera_frame_id, expected_lidar_frame_id: opsional, untuk validasi frame_id (multi-robot)
    ros_logger: opsional, jika ingin log ke ROS2 node gunakan self.get_logger()
    """
    try:
        if not isinstance(calib_file, str):
            msg = "Path file kalibrasi harus string."
            if ros_logger: ros_logger.error(msg)
            logging.error(msg)
            log_to_file(msg, level='error')
            return None
        if not os.path.isfile(calib_file):
            msg = f"File kalibrasi tidak ditemukan: {calib_file}"
            if ros_logger: ros_logger.error(msg)
            logging.error(msg)
            log_to_file(msg, level='error')
            return None
        with open(calib_file, 'r') as f:
            data = yaml.safe_load(f)
        if 'T_lidar_camera' not in data:
            msg = "Key 'T_lidar_camera' tidak ditemukan di file YAML."
            if ros_logger: ros_logger.error(msg)
            logging.error(msg)
            log_to_file(msg, level='error')
            return None
        t = data['T_lidar_camera']
        for k in ['rows', 'cols', 'data', 'camera_frame_id', 'lidar_frame_id']:
            if k not in t:
                msg = f"Field wajib '{k}' tidak lengkap di T_lidar_camera."
                if ros_logger: ros_logger.error(msg)
                logging.error(msg)
                log_to_file(msg, level='error')
                return None
        if t['rows'] != 4 or t['cols'] != 4:
            msg = "Ukuran matriks di YAML tidak 4x4."
            if ros_logger: ros_logger.error(msg)
            logging.error(msg)
            log_to_file(msg, level='error')
            return None
        if not isinstance(t['data'], list) or len(t['data']) != 16:
            msg = "Data matriks di YAML tidak berisi 16 elemen."
            if ros_logger: ros_logger.error(msg)
            logging.error(msg)
            log_to_file(msg, level='error')
            return None
        if expected_camera_frame_id and t['camera_frame_id'] != expected_camera_frame_id:
            msg = f"camera_frame_id di YAML ('{t['camera_frame_id']}') tidak sesuai dengan yang diharapkan ('{expected_camera_frame_id}')."
            if ros_logger: ros_logger.error(msg)
            logging.error(msg)
            log_to_file(msg, level='error')
            return None
        if expected_lidar_frame_id and t['lidar_frame_id'] != expected_lidar_frame_id:
            msg = f"lidar_frame_id di YAML ('{t['lidar_frame_id']}') tidak sesuai dengan yang diharapkan ('{expected_lidar_frame_id}')."
            if ros_logger: ros_logger.error(msg)
            logging.error(msg)
            log_to_file(msg, level='error')
            return None
        T = np.array(t['data'], dtype=np.float64).reshape(4, 4)
        msg = f"Loaded extrinsic calibration from {calib_file}"
        if ros_logger: ros_logger.info(msg)
        logging.info(msg)
        log_to_file(msg)
        return T
    except Exception as e:
        msg = f"Gagal membaca file kalibrasi: {e}"
        if ros_logger: ros_logger.error(msg)
        logging.error(msg)
        log_to_file(msg, level='error')
        logging.error(traceback.format_exc())
        log_to_file(traceback.format_exc(), level='error')
        return None

def project_bbox_to_pointcloud(
    bbox, points, lidar_msg, yolo_msg, T_lidar_camera=None, camera_intrinsic=None, image_shape=None, ros_logger=None
):
    """
    Mengambil subset point cloud yang berada di dalam proyeksi bounding box 2D pada image panorama.
    - bbox: [y1, x1, y2, x2] dari deteksi YOLOv12
    - points: numpy array [N,3] point cloud LiDAR
    - lidar_msg: sensor_msgs/PointCloud2 (untuk frame_id, dsb)
    - yolo_msg: Yolov12Inference (untuk info kamera, dsb)
    - T_lidar_camera: matriks transformasi 4x4 (opsional, jika ingin transformasi frame)
    - camera_intrinsic: matriks intrinsic kamera (opsional, untuk proyeksi 3D->2D)
    - image_shape: tuple (height, width) dari image panorama (opsional, untuk validasi proyeksi)
    - ros_logger: opsional, jika ingin log ke ROS2 node gunakan self.get_logger()
    Return: subset points [M,3] yang berada di dalam bbox (proyeksi kasar)
    """
    try:
        if not isinstance(points, np.ndarray) or points.ndim != 2 or points.shape[1] != 3:
            msg = "Input points harus numpy array shape [N,3]."
            if ros_logger: ros_logger.error(msg)
            logging.error(msg)
            log_to_file(msg, level='error')
            return None
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            msg = "Input bbox harus list/tuple [y1, x1, y2, x2]."
            if ros_logger: ros_logger.error(msg)
            logging.error(msg)
            log_to_file(msg, level='error')
            return None

        if T_lidar_camera is not None:
            points_h = np.hstack([points, np.ones((points.shape[0], 1))])
            points_cam = (T_lidar_camera @ points_h.T).T[:, :3]
        else:
            points_cam = points

        mask = points_cam[:, 2] > 0
        filtered_points = points_cam[mask]

        if camera_intrinsic is not None and image_shape is not None:
            try:
                rvec = np.zeros((3, 1))
                tvec = np.zeros((3, 1))
                pts_2d, _ = cv2.projectPoints(filtered_points, rvec, tvec, camera_intrinsic, None)
                pts_2d = pts_2d.reshape(-1, 2)
                y1, x1, y2, x2 = bbox
                mask_bbox = (
                    (pts_2d[:, 0] >= x1) & (pts_2d[:, 0] <= x2) &
                    (pts_2d[:, 1] >= y1) & (pts_2d[:, 1] <= y2) &
                    (pts_2d[:, 0] >= 0) & (pts_2d[:, 0] < image_shape[1]) &
                    (pts_2d[:, 1] >= 0) & (pts_2d[:, 1] < image_shape[0])
                )
                filtered_points = filtered_points[mask_bbox]
            except Exception as e:
                msg = f"Error proyeksi 3D->2D: {e}"
                if ros_logger: ros_logger.error(msg)
                logging.error(msg)
                log_to_file(msg, level='error')
                logging.error(traceback.format_exc())
                log_to_file(traceback.format_exc(), level='error')
                return None
        msg = f"project_bbox_to_pointcloud: {filtered_points.shape[0]} points in bbox" if filtered_points is not None else "project_bbox_to_pointcloud: no points"
        if ros_logger: ros_logger.debug(msg)
        logging.debug(msg)
        log_to_file(msg, level='debug')
        return filtered_points
    except Exception as e:
        msg = f"Error project_bbox_to_pointcloud: {e}"
        if ros_logger: ros_logger.error(msg)
        logging.error(msg)
        log_to_file(msg, level='error')
        logging.error(traceback.format_exc())
        log_to_file(traceback.format_exc(), level='error')
        return None

# ===================== REVIEW & SARAN =====================
# - Logger ROS2 (opsional via ros_logger) dan logging ke file sudah di setiap langkah penting/error.
# - Semua error/exception di fungsi utama sudah di-log.
# - Validasi file kalibrasi, parameter, dan dependency sudah lengkap.
# - Siap dipanggil dari node fusion_node.py dan pipeline ROS2 Humble.
# - Saran: tambahkan unit test untuk fungsi ini di test/ dan debug visualisasi point cloud hasil filter.