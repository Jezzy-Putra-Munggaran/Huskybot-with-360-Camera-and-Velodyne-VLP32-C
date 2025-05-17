# Copyright 2015 Open Source Robotics Foundation, Inc.  # Copyright wajib untuk ROS2 package, sesuai standar open source
#
# Licensed under the Apache License, Version 2.0 (the "License");  # Lisensi wajib untuk ROS2 package
# you may not use this file except in compliance with the License.  # Penjelasan lisensi
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0  # Link lisensi
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_pep257.main import main  # Import fungsi utama pep257 checker dari ament (otomatis scan seluruh workspace)
import pytest  # Import pytest untuk integrasi dengan framework test ROS2 (pipeline CI/CD)

@pytest.mark.linter  # Tag test ini sebagai linter check (untuk pipeline ROS2/CI/CD)
@pytest.mark.pep257  # Tag test ini sebagai pep257 check (untuk style docstring Python)
def test_pep257():  # Fungsi utama test pep257
    rc = main(argv=['.', 'test'])  # Jalankan pep257 pada seluruh workspace (default: . dan test/)
    # rc: return code (0 jika tidak ada error/warning docstring), error jika ada pelanggaran
    assert rc == 0, 'Found code style errors / warnings'  # Test gagal jika ada error/warning pep257

# Penjelasan:
# - File ini adalah test linter otomatis untuk memastikan semua source code Python di package ini sudah sesuai PEP257 (docstring style).
# - Sangat penting untuk ROS2, CI/CD, dan menjaga kualitas dokumentasi kode di seluruh pipeline (termasuk node, script, test).
# - Test ini akan dijalankan otomatis oleh colcon test atau pytest.
# - Tidak ada dependency ke node, topic, atau file lain, tapi wajib ada di semua package ROS2 yang ingin lolos CI/CD.
# - Sudah siap untuk ROS2 Humble, Gazebo, dan robot real.
# - Tidak perlu OOP di file test linter ini, karena hanya satu fungsi test.

# Saran peningkatan:
# - Pastikan semua file Python di huskybot_calibration/ dan scripts/ sudah lolos pep257 (tidak ada error/warning docstring).
# - Jalankan test ini secara otomatis di pipeline CI/CD sebelum merge/publish.
# - Dokumentasikan di README bahwa linter ini wajib lolos sebelum push ke repo.
# - Jika ingin pengecualian file/folder tertentu, tambahkan konfigurasi pep257 di setup.cfg atau pyproject.toml.
# - Untuk workspace besar, pastikan semua package punya test_pep257.py seperti ini agar kualitas dokumentasi kode terjaga.
