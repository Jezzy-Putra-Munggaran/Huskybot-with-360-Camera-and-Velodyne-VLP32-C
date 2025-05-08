# Copyright 2017 Open Source Robotics Foundation, Inc.
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

from ament_flake8.main import main_with_errors  # Import fungsi utama flake8 checker dari ament
import pytest  # Import pytest untuk integrasi dengan framework test ROS2

@pytest.mark.flake8  # Tag test ini sebagai flake8 check (untuk CI/CD dan linter)
@pytest.mark.linter  # Tag test ini sebagai linter check (untuk pipeline ROS2)
def test_flake8():
    rc, errors = main_with_errors(argv=[])  # Jalankan flake8 pada seluruh workspace (default: .)
    # rc: return code (0 jika tidak ada error), errors: list error style
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)  # Test gagal jika ada error/warning flake8

# Penjelasan:
# - File ini adalah test linter otomatis untuk memastikan semua source code Python di package ini sudah sesuai PEP8.
# - Sangat penting untuk ROS2, CI/CD, dan menjaga kualitas kode di seluruh pipeline (termasuk node, script, test).
# - Test ini akan dijalankan otomatis oleh colcon test atau pytest.
# - Tidak ada dependency ke node, topic, atau file lain, tapi wajib ada di semua package ROS2 yang ingin lolos CI/CD.
# - Sudah siap untuk ROS2 Humble, Gazebo, dan robot real.

# Saran peningkatan:
# - Pastikan semua file Python di huskybot_calibration/ dan scripts/ sudah lolos flake8 (tidak ada error/warning).
# - Jalankan test ini secara otomatis di pipeline CI/CD sebelum merge/publish.
# - Jika ingin pengecualian file/folder tertentu, tambahkan konfigurasi flake8 di setup.cfg atau pyproject.toml.
# - Dokumentasikan di README bahwa linter ini wajib lolos sebelum push ke repo.
