#!/usr/bin/env python3  
# -*- coding: utf-8 -*-  

import os  # Untuk operasi file/folder
import tempfile  # Untuk membuat file/folder sementara saat test
import numpy as np  # Untuk operasi matriks
import yaml  # Untuk baca/tulis file YAML
import unittest  # Framework unit test Python

from huskybot_calibration.utils import (  # Import semua fungsi utilitas yang akan diuji
    validate_extrinsic_yaml,
    load_extrinsic_matrix,
    save_yaml,
    ensure_dir_exists,
    safe_load_yaml,
    matrix_to_quaternion,
    quaternion_to_matrix
)

class TestUtils(unittest.TestCase):  # Kelas unit test untuk fungsi utilitas di utils.py

    def setUp(self):
        # Setup sebelum setiap test, buat folder sementara
        self.test_dir = tempfile.mkdtemp()  # Buat folder sementara
        self.yaml_path = os.path.join(self.test_dir, "test.yaml")  # Path file YAML sementara

    def tearDown(self):
        # Cleanup setelah setiap test, hapus file/folder sementara
        try:
            if os.path.exists(self.yaml_path):
                os.remove(self.yaml_path)  # Hapus file YAML jika ada
            os.rmdir(self.test_dir)  # Hapus folder sementara
        except Exception:
            pass  # Jangan error jika sudah terhapus

    def test_save_and_load_yaml(self):
        # Test simpan dan load file YAML
        data = {'foo': 123, 'bar': 'baz'}  # Data dummy
        self.assertTrue(save_yaml(data, self.yaml_path))  # Simpan file YAML, harus True jika sukses
        loaded = safe_load_yaml(self.yaml_path)  # Load file YAML
        self.assertEqual(loaded, data)  # Data harus sama

    def test_ensure_dir_exists(self):
        # Test fungsi memastikan folder ada
        test_dir2 = os.path.join(self.test_dir, "subdir")  # Path folder baru
        self.assertTrue(ensure_dir_exists(test_dir2))  # Harus bisa buat folder
        self.assertTrue(os.path.isdir(test_dir2))  # Folder harus ada

    def test_validate_extrinsic_yaml_valid(self):
        # Test validasi file YAML extrinsic yang benar
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

    def test_validate_extrinsic_yaml_missing_field(self):
        # Test validasi file YAML extrinsic yang kurang field wajib
        data = {
            'T_lidar_camera': {
                'rows': 4,
                'cols': 4,
                'data': [0]*16,
                # 'camera_frame_id' hilang
                'lidar_frame_id': 'lidar'
            }
        }
        save_yaml(data, self.yaml_path)  # Simpan file YAML
        self.assertFalse(validate_extrinsic_yaml(self.yaml_path))  # Harus tidak valid

    def test_validate_extrinsic_yaml_wrong_shape(self):
        # Test validasi file YAML extrinsic dengan shape salah (bukan 4x4)
        data = {
            'T_lidar_camera': {
                'rows': 3,
                'cols': 3,
                'data': [0]*9,
                'camera_frame_id': 'camera',
                'lidar_frame_id': 'lidar'
            }
        }
        save_yaml(data, self.yaml_path)  # Simpan file YAML
        self.assertFalse(validate_extrinsic_yaml(self.yaml_path))  # Harus tidak valid

    def test_load_extrinsic_matrix(self):
        # Test load matriks extrinsic dari YAML yang valid
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
        T_loaded = load_extrinsic_matrix(self.yaml_path)  # Load matriks dari file
        self.assertIsInstance(T_loaded, np.ndarray)  # Harus numpy array
        self.assertEqual(T_loaded.shape, (4, 4))  # Harus 4x4
        np.testing.assert_array_almost_equal(T_loaded, T)  # Harus sama nilainya

    def test_matrix_to_quaternion_and_back(self):
        # Test konversi matrix <-> quaternion
        T = np.eye(4)  # Matriks identitas 4x4
        quat = matrix_to_quaternion(T)  # Konversi ke quaternion
        self.assertIsInstance(quat, list)  # Harus list
        self.assertEqual(len(quat), 4)  # Harus 4 elemen
        rot = quaternion_to_matrix(quat)  # Kembali ke matrix
        self.assertIsInstance(rot, np.ndarray)  # Harus numpy array
        self.assertEqual(rot.shape, (3, 3))  # Harus 3x3
        np.testing.assert_array_almost_equal(rot, T[:3, :3])  # Harus sama nilainya

    def test_safe_load_yaml_corrupt(self):
        # Test safe_load_yaml pada file YAML corrupt
        with open(self.yaml_path, 'w') as f:
            f.write("foo: [this is not valid yaml")  # Simpan YAML corrupt
        self.assertIsNone(safe_load_yaml(self.yaml_path))  # Harus None

    def test_validate_extrinsic_yaml_file_not_found(self):
        # Test validasi file YAML yang tidak ada
        fake_path = os.path.join(self.test_dir, "not_exist.yaml")  # Path file tidak ada
        self.assertFalse(validate_extrinsic_yaml(fake_path))  # Harus tidak valid

    def test_load_extrinsic_matrix_file_not_found(self):
        # Test load matriks extrinsic dari file yang tidak ada
        fake_path = os.path.join(self.test_dir, "not_exist.yaml")  # Path file tidak ada
        self.assertIsNone(load_extrinsic_matrix(fake_path))  # Harus None

if __name__ == '__main__':
    unittest.main()  # Jalankan semua unit test di file ini

# Penjelasan:
# - Semua fungsi utilitas di huskybot_calibration/utils.py diuji di sini, termasuk error handling file tidak ada, field kurang, shape salah, file corrupt, dan konversi quaternion.
# - Test ini tidak tergantung node ROS2, bisa dijalankan langsung dengan python3 -m unittest test/test_utils.py.
# - Output YAML yang dihasilkan dan divalidasi langsung dipakai node kalibrasi dan pipeline lain di workspace.
# - Sudah aman untuk ROS2 Humble, simulasi Gazebo, dan robot real.
# - Komentar penjelasan di setiap baris agar mudah dipahami siapapun.

# Saran peningkatan:
# - Tambahkan test untuk logger ROS2 (opsional, jika ingin coverage log error ke node).
# - Tambahkan test untuk permission error (opsional, sulit di CI).
# - Dokumentasikan cara menjalankan test di README.