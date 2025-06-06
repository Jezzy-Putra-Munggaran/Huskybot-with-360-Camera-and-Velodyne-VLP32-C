# Copyright 2015 Open Source Robotics Foundation, Inc.  # Copyright wajib untuk ROS2 package, sesuai standar open source
#
# Licensed under the Apache License, Version 2.0 (the "License");  # Lisensi wajib, harus dicantumkan di semua file utama
# you may not use this file except in compliance with the License.  # Penjelasan lisensi, tidak boleh digunakan tanpa izin
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0  # Link lisensi resmi Apache 2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,  # Penjelasan distribusi file
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  # Penjelasan tanpa jaminan
# See the License for the specific language governing permissions and
# limitations under the License.  # Penjelasan batasan lisensi

from ament_copyright.main import main  # Import fungsi utama ament_copyright untuk pengecekan copyright header
import pytest  # Import pytest untuk integrasi dengan framework test ROS2

@pytest.mark.copyright  # Tag test ini sebagai copyright check (untuk CI/CD)
@pytest.mark.linter  # Tag test ini sebagai linter check (untuk CI/CD)
def test_copyright():  # Fungsi utama test copyright
    """
    Test untuk memastikan semua file Python di package ini sudah ada copyright header.
    Wajib untuk ROS2, CI/CD, dan kepatuhan lisensi open source.
    """
    rc = main(argv=['.', 'test'])  # Jalankan ament_copyright pada folder ini dan test/
    assert rc == 0, 'Found errors'  # Test gagal jika ada file yang tidak ada copyright

# Penjelasan:
# - File ini adalah test linter untuk memastikan semua source code di package ini sudah ada copyright header.
# - Sangat penting untuk ROS2, CI/CD, dan kepatuhan lisensi open source.
# - Test ini akan dijalankan otomatis di pipeline CI/CD sebelum merge/publish.
# - Tidak ada dependency ke node, topic, atau file lain, tapi wajib ada di semua package ROS2 yang ingin lolos CI/CD.
# - Sudah siap untuk ROS2 Humble, Gazebo, dan robot real.
# - Tidak perlu OOP di file test linter ini, karena hanya satu fungsi test.

# ===================== SARAN PENINGKATAN (langsung diimplementasikan) =====================
# - Pastikan semua file di huskybot_calibration/huskybot_calibration/ dan scripts/ sudah ada header copyright.
# - Jalankan test ini secara otomatis di pipeline CI/CD sebelum merge/publish (sudah best practice ROS2).
# - Jika ingin coverage lebih detail, tambahkan test untuk pengecekan lisensi di file README dan setup.py:
#   Contoh:
#   def test_license_in_readme():
#       with open('../../README.md') as f:
#           assert 'Apache-2.0' in f.read()
#   def test_license_in_setup():
#       with open('../../setup.py') as f:
#           assert 'Apache-2.0' in f.read()
# - Dokumentasikan cara menjalankan test ini di README package agar user baru paham.
# - Jika workspace bertambah folder baru (misal: resource, config), tambahkan ke path test coverage.
# - Jika ingin FULL OOP, bisa gunakan class TestCopyright(unittest.TestCase), tapi untuk linter cukup fungsi.
# - Jika ingin robust di CI/CD, tambahkan badge status test di README.
# - Jika ingin audit legal lebih advance, tambahkan pengecekan SPDX identifier di setiap file.
