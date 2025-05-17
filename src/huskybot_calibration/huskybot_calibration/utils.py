#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os  # Untuk operasi file/folder
import yaml  # Untuk baca/tulis file YAML
import numpy as np  # Untuk operasi matriks
import logging  # Untuk logging error/info
import traceback  # Untuk logging error detail

try:
    from scipy.spatial.transform import Rotation as R  # Untuk konversi quaternion <-> matrix
except ImportError:
    R = None  # Jika scipy tidak ada, fungsi konversi akan error

class HuskybotCalibrationUtils:
    """
    Kumpulan static utility untuk operasi file YAML, logging, konversi matrix-quaternion,
    dan validasi file kalibrasi di package huskybot_calibration.
    Semua fungsi staticmethod, bisa dipanggil tanpa inisialisasi class.
    """

    file_logger = None  # Logger global untuk file (bisa di-set dari node utama)

    @staticmethod
    def setup_file_logger(log_path="~/huskybot_utils.log"):
        """Setup logger untuk logging ke file."""
        log_path = os.path.expanduser(log_path)
        logger = logging.getLogger("huskybot_utils_file_logger")
        logger.setLevel(logging.INFO)
        if not logger.hasHandlers():
            try:
                fh = logging.FileHandler(log_path)
                fh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s: %(message)s'))
                logger.addHandler(fh)
            except Exception as e:
                print(f"[ERROR] Tidak bisa setup file logger: {e}")
        HuskybotCalibrationUtils.file_logger = logger  # Set logger global class
        return logger

    @staticmethod
    def log_to_file(msg, level='info'):
        """Logging ke file jika file_logger aktif, fallback ke print jika tidak."""
        logger = HuskybotCalibrationUtils.file_logger
        if logger:
            if level == 'error':
                logger.error(msg)
            elif level == 'warn':
                logger.warning(msg)
            elif level == 'debug':
                logger.debug(msg)
            else:
                logger.info(msg)
        else:
            print(f"[LOG][{level.upper()}] {msg}")

    @staticmethod
    def validate_extrinsic_yaml(yaml_path, logger=None):
        """Validasi isi file YAML hasil kalibrasi extrinsic (lidar ke kamera)."""
        if not isinstance(yaml_path, str):
            msg = f"yaml_path harus string, dapat: {type(yaml_path)}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='error')
            return False
        if not os.path.isfile(yaml_path):
            msg = f"File YAML tidak ditemukan: {yaml_path}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='error')
            return False
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            if 'T_lidar_camera' not in data:
                msg = "Key 'T_lidar_camera' tidak ditemukan di file YAML."
                if logger: logger.error(msg)
                else: logging.error(msg)
                HuskybotCalibrationUtils.log_to_file(msg, level='error')
                return False
            t = data['T_lidar_camera']
            for k in ['rows', 'cols', 'data', 'camera_frame_id', 'lidar_frame_id']:
                if k not in t:
                    msg = f"Field wajib '{k}' tidak ada di T_lidar_camera."
                    if logger: logger.error(msg)
                    else: logging.error(msg)
                    HuskybotCalibrationUtils.log_to_file(msg, level='error')
                    return False
            if t['rows'] != 4 or t['cols'] != 4:
                msg = "Ukuran matriks di YAML tidak 4x4."
                if logger: logger.error(msg)
                else: logging.error(msg)
                HuskybotCalibrationUtils.log_to_file(msg, level='error')
                return False
            if not isinstance(t['data'], list) or len(t['data']) != 16:
                msg = "Data matriks di YAML tidak berisi 16 elemen."
                if logger: logger.error(msg)
                else: logging.error(msg)
                HuskybotCalibrationUtils.log_to_file(msg, level='error')
                return False
            return True
        except Exception as e:
            msg = f"Error validasi file YAML: {e}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='error')
            HuskybotCalibrationUtils.log_to_file(traceback.format_exc(), level='error')
            return False

    @staticmethod
    def load_extrinsic_matrix(yaml_path, logger=None):
        """Membaca file YAML kalibrasi extrinsic dan mengembalikan matriks 4x4 (numpy array)."""
        if not HuskybotCalibrationUtils.validate_extrinsic_yaml(yaml_path, logger=logger):
            return None
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            t = data['T_lidar_camera']
            mat = np.array(t['data']).reshape((4, 4))
            return mat
        except Exception as e:
            msg = f"Gagal load matriks extrinsic dari YAML: {e}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='error')
            HuskybotCalibrationUtils.log_to_file(traceback.format_exc(), level='error')
            return None

    @staticmethod
    def ensure_dir_exists(path, logger=None):
        """Membuat folder jika belum ada. Return True jika sukses/ada, False jika gagal."""
        if not isinstance(path, str):
            msg = f"path harus string, dapat: {type(path)}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='error')
            return False
        try:
            if not os.path.exists(path):
                os.makedirs(path)
            return True
        except PermissionError as e:
            msg = f"Permission denied saat membuat folder {path}: {e}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='error')
            HuskybotCalibrationUtils.log_to_file(traceback.format_exc(), level='error')
            return False
        except Exception as e:
            msg = f"Gagal membuat folder {path}: {e}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='error')
            HuskybotCalibrationUtils.log_to_file(traceback.format_exc(), level='error')
            return False

    @staticmethod
    def save_yaml(data, yaml_path, logger=None, backup=True):
        """Simpan data (dict) ke file YAML. Return True jika sukses, False jika gagal."""
        if not isinstance(yaml_path, str):
            msg = f"yaml_path harus string, dapat: {type(yaml_path)}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='error')
            return False
        if backup and os.path.isfile(yaml_path):
            try:
                backup_path = yaml_path + ".bak"
                os.rename(yaml_path, backup_path)
                HuskybotCalibrationUtils.log_to_file(f"Backup file YAML lama ke: {backup_path}")
            except Exception as e:
                msg = f"Gagal backup file YAML lama: {e}"
                if logger: logger.warning(msg)
                else: logging.warning(msg)
                HuskybotCalibrationUtils.log_to_file(msg, level='warn')
        try:
            with open(yaml_path, 'w') as f:
                yaml.dump(data, f)
            return True
        except PermissionError as e:
            msg = f"Permission denied saat menulis file YAML: {e}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='error')
            HuskybotCalibrationUtils.log_to_file(traceback.format_exc(), level='error')
            return False
        except Exception as e:
            msg = f"Gagal menulis file YAML: {e}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='error')
            HuskybotCalibrationUtils.log_to_file(traceback.format_exc(), level='error')
            return False

    @staticmethod
    def safe_load_yaml(yaml_path, logger=None):
        """Membaca file YAML dan mengembalikan dict, None jika gagal."""
        if not isinstance(yaml_path, str):
            msg = f"yaml_path harus string, dapat: {type(yaml_path)}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='error')
            return None
        if not os.path.isfile(yaml_path):
            msg = f"File YAML tidak ditemukan: {yaml_path}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='error')
            return None
        try:
            with open(yaml_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            msg = f"Gagal membaca file YAML: {e}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='error')
            HuskybotCalibrationUtils.log_to_file(traceback.format_exc(), level='error')
            return None

    @staticmethod
    def matrix_to_quaternion(T, logger=None):
        """Konversi matrix rotasi (3x3 atau 4x4) ke quaternion [x, y, z, w]."""
        if R is None:
            msg = "scipy.spatial.transform.Rotation tidak tersedia, install scipy!"
            if logger: logger.error(msg)
            else: logging.error(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='error')
            return None
        try:
            if not isinstance(T, np.ndarray):
                msg = f"Input T harus numpy.ndarray, dapat: {type(T)}"
                if logger: logger.error(msg)
                else: logging.error(msg)
                HuskybotCalibrationUtils.log_to_file(msg, level='error')
                return None
            if T.shape == (4, 4):
                rot = T[:3, :3]
            elif T.shape == (3, 3):
                rot = T
            else:
                msg = f"Bentuk matrix rotasi tidak valid: {T.shape}"
                if logger: logger.error(msg)
                else: logging.error(msg)
                HuskybotCalibrationUtils.log_to_file(msg, level='error')
                return None
            quat = R.from_matrix(rot).as_quat()
            return quat.tolist()
        except Exception as e:
            msg = f"Gagal konversi matrix ke quaternion: {e}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='error')
            HuskybotCalibrationUtils.log_to_file(traceback.format_exc(), level='error')
            return None

    @staticmethod
    def quaternion_to_matrix(quat, logger=None):
        """Konversi quaternion [x, y, z, w] ke matrix rotasi 3x3 (numpy array)."""
        if R is None:
            msg = "scipy.spatial.transform.Rotation tidak tersedia, install scipy!"
            if logger: logger.error(msg)
            else: logging.error(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='error')
            return None
        try:
            quat = np.asarray(quat)
            if quat.shape == (4,):
                rot = R.from_quat(quat)
                return rot.as_matrix()
            else:
                msg = f"Bentuk quaternion tidak valid: {quat}"
                if logger: logger.error(msg)
                else: logging.error(msg)
                HuskybotCalibrationUtils.log_to_file(msg, level='error')
                return None
        except Exception as e:
            msg = f"Gagal konversi quaternion ke matrix: {e}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='error')
            HuskybotCalibrationUtils.log_to_file(traceback.format_exc(), level='error')
            return None

    @staticmethod
    def check_frame_id_consistency(yaml_path, expected_camera_frame, expected_lidar_frame, logger=None):
        """Cek apakah camera_frame_id dan lidar_frame_id di file YAML sesuai dengan yang diharapkan."""
        data = HuskybotCalibrationUtils.safe_load_yaml(yaml_path, logger=logger)
        if data is None:
            return False
        t = data.get('T_lidar_camera', {})
        cam_id = t.get('camera_frame_id', None)
        lidar_id = t.get('lidar_frame_id', None)
        if cam_id != expected_camera_frame or lidar_id != expected_lidar_frame:
            msg = f"Frame ID tidak konsisten: camera_frame_id={cam_id}, lidar_frame_id={lidar_id} (ekspektasi: {expected_camera_frame}, {expected_lidar_frame})"
            if logger: logger.warning(msg)
            else: logging.warning(msg)
            HuskybotCalibrationUtils.log_to_file(msg, level='warn')
            return False
        return True

# --- Penjelasan & Review ---
# - Semua fungsi utility sekarang staticmethod dalam class OOP.
# - Logger file tetap bisa diakses via HuskybotCalibrationUtils.file_logger.
# - Semua error/exception di fungsi utama sudah di-log.
# - Validasi file, parameter, dan dependency sudah lengkap.
# - Siap dipanggil dari node lain di huskybot_calibration dan pipeline ROS2 Humble.
# - Saran: tambahkan unit test untuk semua fungsi ini di test/test_utils.py.
# - Saran: tambahkan validasi folder output sebelum simpan file.
# - Saran: backup file YAML sebelum overwrite (sudah diimplementasi).
# - Saran: cek konsistensi frame_id di YAML (sudah diimplementasi).