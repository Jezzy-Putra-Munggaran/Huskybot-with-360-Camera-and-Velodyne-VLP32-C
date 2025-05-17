#!/usr/bin/env python3  
# -*- coding: utf-8 -*- 

import os  # Untuk operasi file/folder
import tempfile  # Untuk membuat file/folder sementara saat test
import numpy as np  # Untuk operasi matriks numerik
import yaml  # Untuk baca/tulis file YAML
import unittest  # Framework unit test Python

from huskybot_calibration.utils import (
    validate_extrinsic_yaml, load_extrinsic_matrix, save_yaml, ensure_dir_exists, safe_load_yaml, matrix_to_quaternion, quaternion_to_matrix
)  # Import semua fungsi utilitas yang akan diuji

class TestCalibrationUtils(unittest.TestCase):  # Kelas unit test untuk fungsi utilitas kalibrasi

    def setUp(self):  # Fungsi setup sebelum setiap test
        self.test_dir = tempfile.mkdtemp()  # Buat folder sementara untuk test
        self.yaml_path = os.path.join(self.test_dir, "test_extrinsic.yaml")  # Path file YAML sementara

    def tearDown(self):  # Fungsi cleanup setelah setiap test
        try:
            if os.path.exists(self.yaml_path):
                os.remove(self.yaml_path)  # Hapus file YAML jika ada
            os.rmdir(self.test_dir)  # Hapus folder sementara
        except Exception:
            pass  # Jangan error jika sudah terhapus

    def test_validate_extrinsic_yaml_valid(self):  # Test validasi file YAML extrinsic yang benar
        T = np.eye(4)  # Matriks identitas 4x4
        data = {
            'T_lidar_camera': {
                'rows': 4,
                'cols': 4,
                'data': T.flatten().tolist(),
                'camera_frame_id': 'camera',
                'lidar_frame_id': 'lidar'
            }
        }
        save_yaml(data, self.yaml_path)  # Simpan file YAML
        self.assertTrue(validate_extrinsic_yaml(self.yaml_path))  # Harus valid

    def test_validate_extrinsic_yaml_missing_field(self):  # Test validasi file YAML extrinsic yang kurang field wajib
        data = {
            'T_lidar_camera': {
                'rows': 4,
                'cols': 4,
                'data': [0]*16,
                # 'camera_frame_id' hilang
                'lidar_frame_id': 'lidar'
            }
        }
        save_yaml(data, self.yaml_path)
        self.assertFalse(validate_extrinsic_yaml(self.yaml_path))  # Harus tidak valid

    def test_validate_extrinsic_yaml_wrong_shape(self):  # Test validasi file YAML extrinsic dengan shape salah (bukan 4x4)
        data = {
            'T_lidar_camera': {
                'rows': 3,
                'cols': 3,
                'data': [0]*9,
                'camera_frame_id': 'camera',
                'lidar_frame_id': 'lidar'
            }
        }
        save_yaml(data, self.yaml_path)
        self.assertFalse(validate_extrinsic_yaml(self.yaml_path))  # Harus tidak valid

    def test_load_extrinsic_matrix(self):  # Test load matriks extrinsic dari YAML yang valid
        T = np.eye(4)
        data = {
            'T_lidar_camera': {
                'rows': 4,
                'cols': 4,
                'data': T.flatten().tolist(),
                'camera_frame_id': 'camera',
                'lidar_frame_id': 'lidar'
            }
        }
        save_yaml(data, self.yaml_path)
        T_loaded = load_extrinsic_matrix(self.yaml_path)
        self.assertIsInstance(T_loaded, np.ndarray)  # Harus numpy array
        self.assertEqual(T_loaded.shape, (4, 4))  # Harus 4x4
        np.testing.assert_array_almost_equal(T_loaded, T)  # Harus sama nilainya

    def test_ensure_dir_exists(self):  # Test fungsi memastikan folder ada
        test_dir2 = os.path.join(self.test_dir, "subdir")
        self.assertTrue(ensure_dir_exists(test_dir2))  # Harus bisa buat folder
        self.assertTrue(os.path.isdir(test_dir2))  # Folder harus ada

    def test_save_yaml_and_safe_load_yaml(self):  # Test simpan dan load file YAML
        data = {'foo': 123, 'bar': 'baz'}
        path = os.path.join(self.test_dir, "test.yaml")
        self.assertTrue(save_yaml(data, path))  # Simpan file YAML
        loaded = safe_load_yaml(path)  # Load file YAML
        self.assertEqual(loaded, data)  # Data harus sama

    def test_matrix_to_quaternion_and_back(self):  # Test konversi matrix <-> quaternion
        T = np.eye(4)  # Matriks identitas 4x4
        quat = matrix_to_quaternion(T)  # Konversi ke quaternion
        self.assertIsInstance(quat, list)  # Harus list
        self.assertEqual(len(quat), 4)  # Harus 4 elemen
        rot = quaternion_to_matrix(quat)  # Kembali ke matrix
        self.assertIsInstance(rot, np.ndarray)  # Harus numpy array
        self.assertEqual(rot.shape, (3, 3))  # Harus 3x3
        np.testing.assert_array_almost_equal(rot, T[:3, :3])  # Harus sama nilainya

    def test_validate_extrinsic_yaml_file_not_found(self):  # Test validasi file YAML yang tidak ada
        fake_path = os.path.join(self.test_dir, "not_exist.yaml")
        self.assertFalse(validate_extrinsic_yaml(fake_path))  # Harus tidak valid

    def test_load_extrinsic_matrix_file_not_found(self):  # Test load matriks extrinsic dari file yang tidak ada
        fake_path = os.path.join(self.test_dir, "not_exist.yaml")
        self.assertIsNone(load_extrinsic_matrix(fake_path))  # Harus None

    def test_validate_extrinsic_yaml_corrupt(self):  # Test validasi file YAML yang corrupt (tidak bisa di-parse)
        with open(self.yaml_path, 'w') as f:
            f.write("T_lidar_camera: [this is not valid yaml")
        self.assertFalse(validate_extrinsic_yaml(self.yaml_path))  # Harus tidak valid

    def test_safe_load_yaml_corrupt(self):  # Test safe_load_yaml pada file YAML corrupt
        path = os.path.join(self.test_dir, "corrupt.yaml")
        with open(path, 'w') as f:
            f.write("foo: [this is not valid yaml")
        self.assertIsNone(safe_load_yaml(path))  # Harus None

if __name__ == '__main__':  # Jika file dijalankan langsung
    unittest.main()  # Jalankan semua test

# Penjelasan:
# - Semua fungsi utility yang diuji sudah terhubung dengan baik ke node utama, file YAML, dan pipeline workspace.
# - Test ini tidak tergantung node ROS2, bisa dijalankan langsung dengan python3 -m unittest test/test_calibration.py.
# - Output YAML yang dihasilkan dan divalidasi langsung dipakai node kalibrasi dan pipeline lain di workspace.
# - Sudah aman untuk ROS2 Humble, simulasi Gazebo, dan robot real.
# - Komentar penjelasan di setiap baris agar mudah dipahami siapapun.

# Saran peningkatan:
# - Tambahkan test untuk logger ROS2 (opsional, jika ingin coverage log error ke node).
# - Tambahkan test untuk permission error (opsional, sulit di CI).
# - Dokumentasikan cara menjalankan test di README.