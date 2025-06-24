#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# filepath: /home/jezzy/huskybot/src/huskybot_fusion/huskybot_fusion/fusion_marker_utils.py

import traceback                                      # Untuk stack trace exception saat error
import numpy as np                                    # Untuk operasi numerik efisien
import logging                                        # Untuk logging error ke file
from visualization_msgs.msg import Marker, MarkerArray  # Import marker dan array untuk visualisasi RViz2

# ===================== SETUP LOGGING =====================
def setup_logger(log_file="~/huskybot_fusion_marker_utils.log"):  # Fungsi untuk setup logger
    """Setup logger file untuk error handling dan debugging"""
    try:
        import os                                     # Import os untuk manipulasi path
        log_file = os.path.expanduser(log_file)       # Ekspansi ~ ke home directory
        logger = logging.getLogger('marker_utils')    # Buat logger dengan nama unik
        if not logger.handlers:                       # Cek jika handler belum ditambahkan
            logger.setLevel(logging.INFO)             # Set level logging INFO
            handler = logging.FileHandler(log_file)   # Buat file handler
            formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')  # Format logging
            handler.setFormatter(formatter)           # Set formatter ke handler
            logger.addHandler(handler)                # Tambahkan handler ke logger
        return logger                                 # Return logger yang sudah dikonfigurasi
    except Exception as e:                            # Tangani semua exception saat setup
        print(f"[FATAL] Gagal setup logger: {e}")     # Print error ke console
        return None                                   # Return None jika gagal

# Inisialisasi logger sekali saja (singleton)
logger = setup_logger()                               # Buat instance logger

def log_to_file(msg, level='info'):                   # Fungsi untuk log ke file
    """Log pesan ke file dengan level tertentu"""
    if not logger:                                    # Cek jika logger tidak ada
        return                                        # Exit jika tidak ada logger
    
    try:                                              # Try-except untuk menangani error saat logging
        if level == 'error':                          # Jika level error
            logger.error(msg)                         # Log sebagai error
        elif level == 'warn':                         # Jika level warning
            logger.warning(msg)                       # Log sebagai warning
        elif level == 'debug':                        # Jika level debug
            logger.debug(msg)                         # Log sebagai debug
        else:                                         # Default level info
            logger.info(msg)                          # Log sebagai info
    except Exception as e:                            # Tangani exception saat logging
        print(f"[ERROR] Gagal log ke file: {e}")      # Print error ke console

# Dictionary untuk warna marker berdasarkan kelas objek
CLASS_COLORS = {                                      # Definisi warna untuk kelas umum (RGB)
    'person': (1.0, 0.0, 0.0),                        # Merah untuk person/orang
    'pedestrian': (1.0, 0.0, 0.0),                    # Merah untuk pedestrian
    'vehicle': (0.0, 0.0, 1.0),                       # Biru untuk vehicle/kendaraan
    'car': (0.0, 0.0, 1.0),                           # Biru untuk car/mobil
    'truck': (0.0, 0.0, 0.8),                         # Biru tua untuk truck
    'bicycle': (0.0, 1.0, 0.0),                       # Hijau untuk bicycle/sepeda
    'motorcycle': (0.0, 1.0, 0.0),                    # Hijau untuk motorcycle/motor
    'traffic sign': (1.0, 1.0, 0.0),                  # Kuning untuk traffic sign/rambu
    'traffic light': (1.0, 0.5, 0.0),                 # Oranye untuk traffic light/lampu
    'obstacle': (0.5, 0.5, 0.5),                      # Abu-abu untuk obstacle/halangan
    'default': (1.0, 1.0, 1.0)                        # Putih untuk kelas lainnya
}

def get_marker_color(class_name):                     # Fungsi untuk mendapatkan warna marker berdasarkan kelas
    """
    Menentukan warna marker berdasarkan kelas objek.
    
    Args:
        class_name (str): Nama kelas objek
        
    Returns:
        tuple: (r, g, b) nilai warna RGB dalam range 0-1
    """
    if not isinstance(class_name, str):               # Validasi jika class_name string
        return CLASS_COLORS['default']                # Return warna default jika bukan string
    
    # Lakukan lowercase dan cari match di kamus warna
    class_lower = class_name.lower()                  # Konversi ke lowercase untuk matching case-insensitive
    for key in CLASS_COLORS:                          # Iterasi semua kelas yang ada di kamus
        if key in class_lower:                        # Cek jika key ada dalam nama kelas
            return CLASS_COLORS[key]                  # Return warna sesuai kelas
    
    return CLASS_COLORS['default']                    # Return warna default jika tidak ada match

def create_object_marker(obj, idx, frame_id="base_link", ros_logger=None):
    """
    Membuat Marker RViz2 untuk label objek 3D hasil fusion.
    
    Args:
        obj (Object3D): Pesan objek 3D (harus punya .header, .center, .label, .confidence)
        idx (int): Index marker (unik per objek)
        frame_id (str): Frame referensi marker (default: base_link)
        ros_logger (rclpy.Logger): Optional logger dari node ROS2
        
    Returns:
        Marker: Marker RViz2 untuk visualisasi, atau None jika error
    """
    try:
        # ========== VALIDASI INPUT ==========
        # Validasi obj memiliki semua atribut yang diperlukan
        required_attrs = ['header', 'center', 'label']  # Daftar atribut yang harus ada di obj
        for attr in required_attrs:                    # Iterasi setiap atribut wajib
            if not hasattr(obj, attr):                # Cek jika atribut ada di obj
                error_msg = f"Objek tidak memiliki atribut wajib: {attr}"  # Pesan error
                if ros_logger:                        # Jika ros_logger disediakan
                    ros_logger.error(error_msg)       # Log error ke ROS logger
                log_to_file(error_msg, 'error')       # Log error ke file
                return None                           # Return None menandakan error
        
        # Validasi center adalah array/list dengan 3 elemen
        if not isinstance(obj.center, (list, tuple, np.ndarray)) or len(obj.center) != 3:  # Cek tipe dan panjang center
            error_msg = f"Format obj.center tidak valid: {obj.center}, harus array [x,y,z]"  # Pesan error
            if ros_logger:                            # Jika ros_logger disediakan
                ros_logger.error(error_msg)           # Log error ke ROS logger
            log_to_file(error_msg, 'error')           # Log error ke file
            return None                               # Return None menandakan error
        
        # Validasi setiap elemen center adalah nilai numerik valid
        for i, val in enumerate(obj.center):          # Iterasi nilai x,y,z 
            if not isinstance(val, (int, float)) or np.isnan(val) or np.isinf(val):  # Cek tipe dan validitas
                error_msg = f"Nilai obj.center[{i}]={val} tidak valid (harus finite number)"  # Pesan error
                if ros_logger:                        # Jika ros_logger disediakan
                    ros_logger.error(error_msg)       # Log error ke ROS logger
                log_to_file(error_msg, 'error')       # Log error ke file
                return None                           # Return None menandakan error
            
        # Validasi idx adalah integer
        if not isinstance(idx, int):                  # Cek tipe idx
            error_msg = f"idx harus integer, bukan {type(idx)}"  # Pesan error
            if ros_logger:                            # Jika ros_logger disediakan
                ros_logger.error(error_msg)           # Log error ke ROS logger
            log_to_file(error_msg, 'error')           # Log error ke file
            return None                               # Return None menandakan error
            
        # ========== BUAT MARKER ==========
        marker = Marker()                             # Inisialisasi objek Marker ROS2
        marker.header.frame_id = frame_id             # Set frame ID marker (harus konsisten dengan frame sensor/LiDAR)
        marker.header.stamp = obj.header.stamp        # Set timestamp marker (sinkron dengan data sensor)
        marker.ns = "objects3d"                       # Set namespace marker (agar tidak bentrok dengan marker lain di RViz2)
        marker.id = idx                               # Set ID unik marker (harus unik per objek)
        marker.type = Marker.TEXT_VIEW_FACING         # Set tipe marker: teks yang selalu menghadap kamera RViz2
        marker.action = Marker.ADD                    # Set action marker (ADD untuk menambah/replace marker)
        
        # Set posisi marker (posisi objek + offset z untuk keterbacaan)
        marker.pose.position.x = obj.center[0]        # Set posisi X dari hasil fusion
        marker.pose.position.y = obj.center[1]        # Set posisi Y dari hasil fusion
        marker.pose.position.z = obj.center[2] + 0.5  # Set posisi Z dari hasil fusion + offset 0.5m ke atas
        
        # Set orientasi marker (identity quaternion = no rotation)
        marker.pose.orientation.w = 1.0               # Set orientasi quaternion W=1 (default, no rotation)
        marker.pose.orientation.x = 0.0               # Set orientasi quaternion X=0 (default, no rotation)
        marker.pose.orientation.y = 0.0               # Set orientasi quaternion Y=0 (default, no rotation)
        marker.pose.orientation.z = 0.0               # Set orientasi quaternion Z=0 (default, no rotation)
        
        # Set skala marker (ukuran teks)
        marker.scale.x = 0.1                          # Set lebar teks (tidak dipakai untuk TEXT_VIEW_FACING)
        marker.scale.y = 0.1                          # Set tinggi teks (tidak dipakai untuk TEXT_VIEW_FACING)
        marker.scale.z = 0.4                          # Set ukuran font teks (besar kecilnya teks di RViz2)
        
        # Set warna marker berdasarkan kelas objek
        r, g, b = get_marker_color(obj.label)         # Dapatkan warna RGB berdasarkan kelas
        marker.color.r = r                            # Set warna Red
        marker.color.g = g                            # Set warna Green
        marker.color.b = b                            # Set warna Blue
        marker.color.a = 1.0                          # Set alpha (transparansi, 1.0 = solid)
        
        # Hitung jarak dari origin ke objek dengan numpy (lebih efisien)
        try:
            distance = np.linalg.norm(obj.center)     # Hitung jarak Euclidean dengan numpy (efisien)
        except Exception as e:                        # Jika error dengan numpy
            # Fallback ke metode manual jika numpy error
            distance = sum([x**2 for x in obj.center])**0.5  # Hitung jarak manual (pythagoras)
            
        # Set isi teks marker dengan informasi detail objek
        confidence_str = ""                           # Inisialisasi string confidence
        if hasattr(obj, 'confidence'):                # Cek jika objek punya atribut confidence
            if isinstance(obj.confidence, (int, float)) and 0 <= obj.confidence <= 1.0:  # Validasi nilai confidence
                confidence_str = f"\nConf: {obj.confidence:.2f}"  # Format confidence 2 desimal
                
        # Format teks dengan nama kelas, jarak, dan posisi
        marker.text = f"[{obj.label}]\nJarak: {distance:.2f}m\nPosisi: ({obj.center[0]:.2f}, {obj.center[1]:.2f}, {obj.center[2]:.2f}){confidence_str}"
        
        # Set lifetime marker (durasi marker ditampilkan)
        marker.lifetime.sec = 1                       # Set lifetime 1 detik (update per cycle)
        marker.lifetime.nanosec = 0                   # Set lifetime nanosecond = 0
        
        # Log info marker berhasil dibuat
        info_msg = f"Marker untuk {obj.label} berhasil dibuat di posisi {obj.center}"  # Pesan info
        if ros_logger:                                # Jika ros_logger disediakan
            ros_logger.debug(info_msg)                # Log debug ke ROS logger
        log_to_file(info_msg, 'debug')                # Log debug ke file
            
        return marker                                 # Return objek Marker ke caller (fusion_node.py)
        
    except Exception as e:                            # Tangani semua exception lainnya
        error_msg = f"Error saat membuat marker: {str(e)}\n{traceback.format_exc()}"  # Pesan error dengan traceback
        if ros_logger:                                # Jika ros_logger disediakan
            ros_logger.error(error_msg)               # Log error ke ROS logger
        log_to_file(error_msg, 'error')               # Log error ke file
        return None                                   # Return None menandakan error

def create_bbox_marker(obj, idx, frame_id="base_link", ros_logger=None):
    """
    Membuat Marker RViz2 untuk bounding box 3D hasil fusion.
    
    Args:
        obj (Object3D): Pesan objek 3D (harus punya .header, .center, .size, .orientation)
        idx (int): Index marker (unik per objek)
        frame_id (str): Frame referensi marker (default: base_link)
        ros_logger (rclpy.Logger): Optional logger dari node ROS2
        
    Returns:
        Marker: Marker RViz2 untuk visualisasi bounding box, atau None jika error
    """
    try:
        # ========== VALIDASI INPUT ==========
        # Validasi obj memiliki semua atribut yang diperlukan
        required_attrs = ['header', 'center', 'size', 'orientation', 'label']  # Daftar atribut wajib
        for attr in required_attrs:                   # Iterasi setiap atribut wajib
            if not hasattr(obj, attr):                # Cek jika atribut ada di obj
                error_msg = f"Objek tidak memiliki atribut wajib: {attr}"  # Pesan error
                if ros_logger:                        # Jika ros_logger disediakan
                    ros_logger.error(error_msg)       # Log error ke ROS logger
                log_to_file(error_msg, 'error')       # Log error ke file
                return None                           # Return None menandakan error
        
        # Validasi center, size, dan orientation adalah array dengan jumlah elemen yang benar
        if not isinstance(obj.center, (list, tuple, np.ndarray)) or len(obj.center) != 3:
            error_msg = f"Format obj.center tidak valid: {obj.center}, harus array [x,y,z]"
            if ros_logger:                            # Jika ros_logger disediakan
                ros_logger.error(error_msg)           # Log error ke ROS logger
            log_to_file(error_msg, 'error')           # Log error ke file
            return None                               # Return None menandakan error
            
        if not isinstance(obj.size, (list, tuple, np.ndarray)) or len(obj.size) != 3:
            error_msg = f"Format obj.size tidak valid: {obj.size}, harus array [dx,dy,dz]"
            if ros_logger:                            # Jika ros_logger disediakan
                ros_logger.error(error_msg)           # Log error ke ROS logger
            log_to_file(error_msg, 'error')           # Log error ke file
            return None                               # Return None menandakan error
            
        if not isinstance(obj.orientation, (list, tuple, np.ndarray)) or len(obj.orientation) != 4:
            error_msg = f"Format obj.orientation tidak valid: {obj.orientation}, harus quaternion [x,y,z,w]"
            if ros_logger:                            # Jika ros_logger disediakan
                ros_logger.error(error_msg)           # Log error ke ROS logger
            log_to_file(error_msg, 'error')           # Log error ke file
            return None                               # Return None menandakan error
            
        # ========== BUAT MARKER BOUNDING BOX ==========
        marker = Marker()                             # Inisialisasi objek Marker ROS2
        marker.header.frame_id = frame_id             # Set frame ID marker
        marker.header.stamp = obj.header.stamp        # Set timestamp marker
        marker.ns = "objects3d_bbox"                  # Set namespace marker (berbeda dengan text marker)
        marker.id = idx * 1000                        # Set ID unik marker (offset berbeda dari text marker)
        marker.type = Marker.CUBE                     # Set tipe marker: CUBE untuk bounding box 3D
        marker.action = Marker.ADD                    # Set action marker
        
        # Set posisi marker (posisi objek)
        marker.pose.position.x = obj.center[0]        # Set posisi X dari hasil fusion
        marker.pose.position.y = obj.center[1]        # Set posisi Y dari hasil fusion
        marker.pose.position.z = obj.center[2]        # Set posisi Z dari hasil fusion
        
        # Set orientasi marker (dari quaternion objek)
        marker.pose.orientation.x = obj.orientation[0]  # Set orientasi X dari hasil fusion
        marker.pose.orientation.y = obj.orientation[1]  # Set orientasi Y dari hasil fusion
        marker.pose.orientation.z = obj.orientation[2]  # Set orientasi Z dari hasil fusion
        marker.pose.orientation.w = obj.orientation[3]  # Set orientasi W dari hasil fusion
        
        # Set ukuran marker (dari size objek)
        marker.scale.x = max(0.1, obj.size[0])        # Set ukuran X (minimal 0.1m agar tetap terlihat)
        marker.scale.y = max(0.1, obj.size[1])        # Set ukuran Y (minimal 0.1m agar tetap terlihat)
        marker.scale.z = max(0.1, obj.size[2])        # Set ukuran Z (minimal 0.1m agar tetap terlihat)
        
        # Set warna marker semi-transparan berdasarkan kelas objek
        r, g, b = get_marker_color(obj.label)         # Dapatkan warna RGB berdasarkan kelas
        marker.color.r = r                            # Set warna Red
        marker.color.g = g                            # Set warna Green
        marker.color.b = b                            # Set warna Blue
        marker.color.a = 0.5                          # Set alpha (transparansi, 0.5 = semi-transparan)
        
        # Set lifetime marker (durasi marker ditampilkan)
        marker.lifetime.sec = 1                       # Set lifetime 1 detik (update per cycle)
        marker.lifetime.nanosec = 0                   # Set lifetime nanosecond = 0
        
        # Log info marker berhasil dibuat
        info_msg = f"BBox marker untuk {obj.label} berhasil dibuat di posisi {obj.center}"  # Pesan info
        if ros_logger:                                # Jika ros_logger disediakan
            ros_logger.debug(info_msg)                # Log debug ke ROS logger
        log_to_file(info_msg, 'debug')                # Log debug ke file
            
        return marker                                 # Return objek Marker ke caller
        
    except Exception as e:                            # Tangani semua exception lainnya
        error_msg = f"Error saat membuat bbox marker: {str(e)}\n{traceback.format_exc()}"  # Pesan error dengan traceback
        if ros_logger:                                # Jika ros_logger disediakan
            ros_logger.error(error_msg)               # Log error ke ROS logger
        log_to_file(error_msg, 'error')               # Log error ke file
        return None                                   # Return None menandakan error

def create_object_markers(objects, frame_id="base_link", ros_logger=None):
    """
    Membuat MarkerArray untuk semua objek 3D hasil fusion.
    
    Args:
        objects (list): List objek 3D (harus punya .header, .center, .size, .orientation, .label)
        frame_id (str): Frame referensi marker (default: base_link)
        ros_logger (rclpy.Logger): Optional logger dari node ROS2
        
    Returns:
        MarkerArray: MarkerArray untuk semua objek, atau array kosong jika error
    """
    try:
        # ========== VALIDASI INPUT ==========
        if not isinstance(objects, (list, tuple)):    # Validasi objects adalah list/tuple
            error_msg = f"Parameter objects harus list/tuple, bukan {type(objects)}"  # Pesan error
            if ros_logger:                            # Jika ros_logger disediakan
                ros_logger.error(error_msg)           # Log error ke ROS logger
            log_to_file(error_msg, 'error')           # Log error ke file
            return MarkerArray()                      # Return array kosong
            
        markers = MarkerArray()                       # Inisialisasi MarkerArray untuk semua marker
        
        # Iterasi setiap objek dan buat marker label dan bounding box
        for idx, obj in enumerate(objects):           # Iterasi semua objek dengan index
            # Buat marker text label
            text_marker = create_object_marker(obj, idx, frame_id, ros_logger)  # Buat marker label
            if text_marker:                           # Jika berhasil dibuat
                markers.markers.append(text_marker)   # Tambahkan ke array
            
            # Buat marker bounding box
            bbox_marker = create_bbox_marker(obj, idx, frame_id, ros_logger)  # Buat marker bbox
            if bbox_marker:                           # Jika berhasil dibuat
                markers.markers.append(bbox_marker)   # Tambahkan ke array
            
        return markers                                # Return MarkerArray dengan semua marker
        
    except Exception as e:                            # Tangani semua exception lainnya
        error_msg = f"Error dalam create_object_markers: {str(e)}\n{traceback.format_exc()}"  # Pesan error
        if ros_logger:                                # Jika ros_logger disediakan
            ros_logger.error(error_msg)               # Log error ke ROS logger
        log_to_file(error_msg, 'error')               # Log error ke file
        return MarkerArray()                          # Return array kosong

# ===================== COMPATIBILITY FUNCTIONS =====================
def create_marker_array_from_objects(objects, frame_id="base_link", ros_logger=None):
    """
    Fungsi kompatibilitas untuk membuat MarkerArray dari objek-objek 3D.
    Wrapper ke fungsi create_object_markers.
    
    Args:
        objects (list): List objek 3D (harus punya .header, .center, .size, .orientation, .label)
        frame_id (str): Frame referensi marker (default: base_link)
        ros_logger (rclpy.Logger): Optional logger dari node ROS2
        
    Returns:
        MarkerArray: MarkerArray untuk semua objek
    """
    return create_object_markers(objects, frame_id, ros_logger)  # Panggil fungsi utama

# ===================== PENJELASAN & SARAN PENINGKATAN =====================
# - Fungsi telah diupgrade dengan error handling komprehensif di setiap langkah.
# - Marker text dan bounding box sekarang dibuat untuk visualisasi lengkap.
# - Warna marker disesuaikan dengan kelas objek untuk visualisasi intuitif.
# - Logging lengkap ke file dan ROS logger untuk audit dan debugging.
# - Validasi input ekstensif untuk mencegah runtime error.
# - Penghitungan jarak lebih efisien dengan numpy.linalg.norm dan fallback.
# - Namespace berbeda untuk marker teks dan bounding box mencegah konflik ID.
# - Dokumentasi dan komentar lengkap di setiap baris untuk maintainability.
# - Fungsi utamanya modular dan dapat digunakan untuk membuat marker tunggal atau batch.
# - Fungsi kompatibilitas disediakan untuk mendukung kode yang mungkin masih menggunakan API lama.
# - Mudah digunakan dari node fusion atau node visualisasi lainnya.