#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os  # Untuk operasi file/folder (cek, hapus, buat)
import tempfile  # Untuk membuat file/folder sementara saat test
import numpy as np  # Untuk operasi matriks numerik (validasi, assert)
import yaml  # Untuk baca/tulis file YAML (validasi isi file)
import unittest  # Framework unit test Python (wajib untuk test OOP)

from huskybot_calibration.utils import (  # Import semua fungsi utilitas yang akan diuji (terhubung ke node utama)
    validate_extrinsic_yaml,  # Fungsi validasi file YAML extrinsic
    load_extrinsic_matrix,    # Fungsi load matriks extrinsic dari YAML
    save_yaml,                # Fungsi simpan file YAML
    ensure_dir_exists,        # Fungsi pastikan folder ada
    safe_load_yaml,           # Fungsi load YAML dengan error handling
    matrix_to_quaternion,     # Fungsi konversi matriks ke quaternion
    quaternion_to_matrix      # Fungsi konversi quaternion ke matriks
)

class TestCalibrationUtils(unittest.TestCase):  # Kelas unit test untuk fungsi utilitas kalibrasi (FULL OOP, best practice)

    def setUp(self):  # Fungsi setup sebelum setiap test (otomatis dipanggil unittest)
        self.test_dir = tempfile.mkdtemp()  # Buat folder sementara untuk test (isolasi test, tidak ganggu file asli)
        self.yaml_path = os.path.join(self.test_dir, "test_extrinsic.yaml")  # Path file YAML sementara

    def tearDown(self):  # Fungsi cleanup setelah setiap test (otomatis dipanggil unittest)
        try:
            if os.path.exists(self.yaml_path):  # Jika file YAML ada
                os.remove(self.yaml_path)  # Hapus file YAML agar tidak numpuk
            os.rmdir(self.test_dir)  # Hapus folder sementara
        except Exception:
            pass  # Jangan error jika sudah terhapus (robust)

    def test_validate_extrinsic_yaml_valid(self):  # Test validasi file YAML extrinsic yang benar (happy path)
        T = np.eye(4)  # Matriks identitas 4x4 (ground truth)
        data = {
            'T_lidar_camera': {
                'rows': 4,  # Field wajib
                'cols': 4,  # Field wajib
                'data': T.flatten().tolist(),  # Data matriks 4x4 (16 elemen)
                'camera_frame_id': 'camera',  # Field wajib
                'lidar_frame_id': 'lidar'     # Field wajib
            }
        }
        save_yaml(data, self.yaml_path)  # Simpan file YAML (uji save_yaml juga)
        self.assertTrue(validate_extrinsic_yaml(self.yaml_path))  # Harus valid (assert True)

    def test_validate_extrinsic_yaml_missing_field(self):  # Test validasi file YAML extrinsic yang kurang field wajib
        data = {
            'T_lidar_camera': {
                'rows': 4,
                'cols': 4,
                'data': [0]*16,
                # 'camera_frame_id' hilang (error case)
                'lidar_frame_id': 'lidar'
            }
        }
        save_yaml(data, self.yaml_path)  # Simpan file YAML
        self.assertFalse(validate_extrinsic_yaml(self.yaml_path))  # Harus tidak valid (assert False)

    def test_validate_extrinsic_yaml_wrong_shape(self):  # Test validasi file YAML extrinsic dengan shape salah (bukan 4x4)
        data = {
            'T_lidar_camera': {
                'rows': 3,  # Salah, harus 4
                'cols': 3,  # Salah, harus 4
                'data': [0]*9,  # Salah, harus 16 elemen
                'camera_frame_id': 'camera',
                'lidar_frame_id': 'lidar'
            }
        }
        save_yaml(data, self.yaml_path)
        self.assertFalse(validate_extrinsic_yaml(self.yaml_path))  # Harus tidak valid

    def test_load_extrinsic_matrix(self):  # Test load matriks extrinsic dari YAML yang valid
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
        save_yaml(data, self.yaml_path)
        T_loaded = load_extrinsic_matrix(self.yaml_path)  # Load matriks dari file
        self.assertIsInstance(T_loaded, np.ndarray)  # Harus numpy array
        self.assertEqual(T_loaded.shape, (4, 4))  # Harus 4x4
        np.testing.assert_array_almost_equal(T_loaded, T)  # Harus sama nilainya (toleransi float)

    def test_ensure_dir_exists(self):  # Test fungsi memastikan folder ada (robust folder creation)
        test_dir2 = os.path.join(self.test_dir, "subdir")  # Path folder baru
        self.assertTrue(ensure_dir_exists(test_dir2))  # Harus bisa buat folder (return True)
        self.assertTrue(os.path.isdir(test_dir2))  # Folder harus benar-benar ada

    def test_save_yaml_and_safe_load_yaml(self):  # Test simpan dan load file YAML (robust file I/O)
        data = {'foo': 123, 'bar': 'baz'}  # Data dummy
        path = os.path.join(self.test_dir, "test.yaml")  # Path file YAML
        self.assertTrue(save_yaml(data, path))  # Simpan file YAML (return True)
        loaded = safe_load_yaml(path)  # Load file YAML (harus tidak error)
        self.assertEqual(loaded, data)  # Data harus sama (assert equality)

    def test_matrix_to_quaternion_and_back(self):  # Test konversi matrix <-> quaternion (robust math)
        T = np.eye(4)  # Matriks identitas 4x4
        quat = matrix_to_quaternion(T)  # Konversi ke quaternion
        self.assertIsInstance(quat, list)  # Harus list
        self.assertEqual(len(quat), 4)  # Harus 4 elemen (x, y, z, w)
        rot = quaternion_to_matrix(quat)  # Kembali ke matrix
        self.assertIsInstance(rot, np.ndarray)  # Harus numpy array
        self.assertEqual(rot.shape, (3, 3))  # Harus 3x3
        np.testing.assert_array_almost_equal(rot, T[:3, :3])  # Harus sama nilainya

    def test_validate_extrinsic_yaml_file_not_found(self):  # Test validasi file YAML yang tidak ada (robust error handling)
        fake_path = os.path.join(self.test_dir, "not_exist.yaml")  # Path file tidak ada
        self.assertFalse(validate_extrinsic_yaml(fake_path))  # Harus tidak valid

    def test_load_extrinsic_matrix_file_not_found(self):  # Test load matriks extrinsic dari file yang tidak ada
        fake_path = os.path.join(self.test_dir, "not_exist.yaml")
        self.assertIsNone(load_extrinsic_matrix(fake_path))  # Harus None (robust error handling)

    def test_validate_extrinsic_yaml_corrupt(self):  # Test validasi file YAML yang corrupt (tidak bisa di-parse)
        with open(self.yaml_path, 'w') as f:
            f.write("T_lidar_camera: [this is not valid yaml")  # Corrupt YAML
        self.assertFalse(validate_extrinsic_yaml(self.yaml_path))  # Harus tidak valid

    def test_safe_load_yaml_corrupt(self):  # Test safe_load_yaml pada file YAML corrupt (robust error handling)
        path = os.path.join(self.test_dir, "corrupt.yaml")
        with open(path, 'w') as f:
            f.write("foo: [this is not valid yaml")  # Corrupt YAML
        self.assertIsNone(safe_load_yaml(path))  # Harus None

if __name__ == '__main__':  # Jika file dijalankan langsung (CLI)
    unittest.main()  # Jalankan semua test (entry point unittest)

# Penjelasan:
# - Semua fungsi utility yang diuji sudah terhubung dengan baik ke node utama, file YAML, dan pipeline workspace (fusion, calibrator, dsb).
# - Test ini tidak tergantung node ROS2, bisa dijalankan langsung dengan python3 -m unittest test/test_calibration.py (robust, portable).
# - Output YAML yang dihasilkan dan divalidasi langsung dipakai node kalibrasi dan pipeline lain di workspace (integrasi end-to-end).
# - Sudah aman untuk ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson Orin + Arducam IMX477 + Velodyne VLP32-C).
# - Komentar penjelasan di setiap baris agar mudah dipahami siapapun (best practice, wajib untuk riset kolaboratif).

# Saran peningkatan (langsung diimplementasikan jika memungkinkan):
# - Tambahkan test untuk logger ROS2 (opsional, jika ingin coverage log error ke node, bisa pakai mock logger).
# - Tambahkan test untuk permission error (opsional, sulit di CI, bisa monkeypatch open/permission).
# - Dokumentasikan cara menjalankan test di README (sudah ada di README, pastikan user baru mudah menjalankan test).
# - Jika workspace bertambah sensor baru, tambahkan test validasi field baru di YAML.
# - Jika ingin coverage lebih tinggi, tambahkan test edge case (file kosong, file besar, dsb).