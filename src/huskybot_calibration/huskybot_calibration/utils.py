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

# ===================== LOGGING TO FILE (OPSIONAL) =====================
def setup_file_logger(log_path="~/huskybot_utils.log"):
    log_path = os.path.expanduser(log_path)
    logger = logging.getLogger("huskybot_utils_file_logger")
    logger.setLevel(logging.INFO)
    if not logger.hasHandlers():
        fh = logging.FileHandler(log_path)
        fh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s: %(message)s'))
        logger.addHandler(fh)
    return logger

file_logger = None

def log_to_file(msg, level='info'):
    global file_logger
    if file_logger:
        if level == 'error':
            file_logger.error(msg)
        elif level == 'warn':
            file_logger.warning(msg)
        elif level == 'debug':
            file_logger.debug(msg)
        else:
            file_logger.info(msg)

def validate_extrinsic_yaml(yaml_path, logger=None):  # Fungsi untuk validasi file YAML kalibrasi extrinsic
    """
    Validasi isi file YAML hasil kalibrasi extrinsic (lidar ke kamera).
    Return True jika valid, False jika tidak.
    logger: opsional, jika ingin log ke ROS2 node gunakan self.get_logger()
    """
    if not os.path.isfile(yaml_path):  # Cek file ada
        msg = f"File YAML tidak ditemukan: {yaml_path}"
        if logger: logger.error(msg)
        else: logging.error(msg)
        log_to_file(msg, level='error')
        return False
    try:
        with open(yaml_path, 'r') as f:  # Buka file YAML
            data = yaml.safe_load(f)  # Load isi file YAML
        if 'T_lidar_camera' not in data:  # Cek key utama
            msg = "Key 'T_lidar_camera' tidak ditemukan di file YAML."
            if logger: logger.error(msg)
            else: logging.error(msg)
            log_to_file(msg, level='error')
            return False
        t = data['T_lidar_camera']  # Ambil dict transformasi
        # Cek semua field wajib ada
        for k in ['rows', 'cols', 'data', 'camera_frame_id', 'lidar_frame_id']:
            if k not in t:
                msg = f"Field wajib '{k}' tidak ada di T_lidar_camera."
                if logger: logger.error(msg)
                else: logging.error(msg)
                log_to_file(msg, level='error')
                return False
        if t['rows'] != 4 or t['cols'] != 4:  # Cek ukuran matriks
            msg = "Ukuran matriks di YAML tidak 4x4."
            if logger: logger.error(msg)
            else: logging.error(msg)
            log_to_file(msg, level='error')
            return False
        if not isinstance(t['data'], list) or len(t['data']) != 16:  # Cek jumlah elemen matriks
            msg = "Data matriks di YAML tidak berisi 16 elemen."
            if logger: logger.error(msg)
            else: logging.error(msg)
            log_to_file(msg, level='error')
            return False
        return True  # Semua valid
    except Exception as e:
        msg = f"Error validasi file YAML: {e}"
        if logger: logger.error(msg)
        else: logging.error(msg)
        log_to_file(msg, level='error')
        log_to_file(traceback.format_exc(), level='error')
        return False

def load_extrinsic_matrix(yaml_path, logger=None):  # Fungsi untuk load matriks extrinsic dari YAML
    """
    Membaca file YAML kalibrasi extrinsic dan mengembalikan matriks 4x4 (numpy array).
    Return None jika gagal.
    logger: opsional, jika ingin log ke ROS2 node gunakan self.get_logger()
    """
    if not validate_extrinsic_yaml(yaml_path, logger=logger):  # Validasi dulu
        return None
    try:
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)  # Load isi file YAML
        t = data['T_lidar_camera']  # Ambil dict transformasi
        mat = np.array(t['data']).reshape((4, 4))  # Konversi ke numpy array 4x4
        return mat
    except Exception as e:
        msg = f"Gagal load matriks extrinsic dari YAML: {e}"
        if logger: logger.error(msg)
        else: logging.error(msg)
        log_to_file(msg, level='error')
        log_to_file(traceback.format_exc(), level='error')
        return None

def ensure_dir_exists(path, logger=None):  # Fungsi untuk memastikan folder ada
    """
    Membuat folder jika belum ada. Return True jika sukses/ada, False jika gagal.
    logger: opsional, jika ingin log ke ROS2 node gunakan self.get_logger()
    """
    try:
        if not os.path.exists(path):  # Cek folder ada
            os.makedirs(path)  # Buat folder jika belum ada
        return True
    except Exception as e:
        msg = f"Gagal membuat folder {path}: {e}"
        if logger: logger.error(msg)
        else: logging.error(msg)
        log_to_file(msg, level='error')
        log_to_file(traceback.format_exc(), level='error')
        return False

def save_yaml(data, yaml_path, logger=None):  # Fungsi untuk simpan data ke file YAML
    """
    Simpan data (dict) ke file YAML. Return True jika sukses, False jika gagal.
    logger: opsional, jika ingin log ke ROS2 node gunakan self.get_logger()
    """
    try:
        with open(yaml_path, 'w') as f:
            yaml.dump(data, f)  # Simpan data ke file YAML
        return True
    except Exception as e:
        msg = f"Gagal menulis file YAML: {e}"
        if logger: logger.error(msg)
        else: logging.error(msg)
        log_to_file(msg, level='error')
        log_to_file(traceback.format_exc(), level='error')
        return False

def safe_load_yaml(yaml_path, logger=None):  # Fungsi untuk load YAML dengan error handling
    """
    Membaca file YAML dan mengembalikan dict, None jika gagal.
    logger: opsional, jika ingin log ke ROS2 node gunakan self.get_logger()
    """
    if not os.path.isfile(yaml_path):  # Cek file ada
        msg = f"File YAML tidak ditemukan: {yaml_path}"
        if logger: logger.error(msg)
        else: logging.error(msg)
        log_to_file(msg, level='error')
        return None
    try:
        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f)  # Load dan return dict
    except Exception as e:
        msg = f"Gagal membaca file YAML: {e}"
        if logger: logger.error(msg)
        else: logging.error(msg)
        log_to_file(msg, level='error')
        log_to_file(traceback.format_exc(), level='error')
        return None

def matrix_to_quaternion(T, logger=None):  # Fungsi konversi matrix rotasi 3x3/4x4 ke quaternion [x, y, z, w]
    """
    Konversi matrix rotasi (3x3 atau 4x4) ke quaternion [x, y, z, w].
    Return list [x, y, z, w] atau None jika gagal.
    """
    if R is None:
        msg = "scipy.spatial.transform.Rotation tidak tersedia, install scipy!"
        if logger: logger.error(msg)
        else: logging.error(msg)
        log_to_file(msg, level='error')
        return None
    try:
        if T.shape == (4, 4):
            rot = T[:3, :3]
        elif T.shape == (3, 3):
            rot = T
        else:
            msg = f"Bentuk matrix rotasi tidak valid: {T.shape}"
            if logger: logger.error(msg)
            else: logging.error(msg)
            log_to_file(msg, level='error')
            return None
        quat = R.from_matrix(rot).as_quat()  # [x, y, z, w]
        return quat.tolist()
    except Exception as e:
        msg = f"Gagal konversi matrix ke quaternion: {e}"
        if logger: logger.error(msg)
        else: logging.error(msg)
        log_to_file(msg, level='error')
        log_to_file(traceback.format_exc(), level='error')
        return None

def quaternion_to_matrix(quat, logger=None):  # Fungsi konversi quaternion [x, y, z, w] ke matrix rotasi 3x3
    """
    Konversi quaternion [x, y, z, w] ke matrix rotasi 3x3 (numpy array).
    Return matrix 3x3 atau None jika gagal.
    """
    if R is None:
        msg = "scipy.spatial.transform.Rotation tidak tersedia, install scipy!"
        if logger: logger.error(msg)
        else: logging.error(msg)
        log_to_file(msg, level='error')
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
            log_to_file(msg, level='error')
            return None
    except Exception as e:
        msg = f"Gagal konversi quaternion ke matrix: {e}"
        if logger: logger.error(msg)
        else: logging.error(msg)
        log_to_file(msg, level='error')
        log_to_file(traceback.format_exc(), level='error')
        return None

# --- Penjelasan & Review ---
# - Logger ROS2 (opsional via argumen logger) dan logging ke file sudah di setiap langkah penting/error.
# - Semua error/exception di fungsi utama sudah di-log.
# - Validasi file, parameter, dan dependency sudah lengkap.
# - Siap dipanggil dari node lain di huskybot_calibration dan pipeline ROS2 Humble.
# - Saran: tambahkan unit test untuk semua fungsi ini di test/test_utils.py.
# - Saran: tambahkan validasi folder output sebelum simpan file.