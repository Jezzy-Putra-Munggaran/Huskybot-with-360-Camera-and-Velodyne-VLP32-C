# Copyright 2015 Open Source Robotics Foundation, Inc.
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

from ament_copyright.main import main  # Import fungsi utama ament_copyright untuk pengecekan copyright header
import pytest  # Import pytest untuk integrasi dengan framework test ROS2

# Remove the `skip` decorator once the source file(s) have a copyright header
@pytest.mark.skip(reason='No copyright header has been placed in the generated source file.')  # Skip test jika belum ada header copyright di semua file
@pytest.mark.copyright  # Tag test ini sebagai copyright check
@pytest.mark.linter  # Tag test ini sebagai linter check (untuk CI/CD)
def test_copyright():
    rc = main(argv=['.', 'test'])  # Jalankan ament_copyright pada folder ini dan test/
    assert rc == 0, 'Found errors'  # Test gagal jika ada file yang tidak ada copyright

# Penjelasan:
# - File ini adalah test linter untuk memastikan semua source code di package ini sudah ada copyright header.
# - Sangat penting untuk ROS2, CI/CD, dan kepatuhan lisensi open source.
# - Test ini akan di-skip sampai semua file Python utama sudah diberi header copyright.
# - Tidak ada dependency ke node, topic, atau file lain, tapi wajib ada di semua package ROS2 yang ingin lolos CI/CD.
# - Sudah siap untuk ROS2 Humble, Gazebo, dan robot real.

# Saran peningkatan:
# - Setelah semua file Python utama sudah diberi header copyright, hapus/deaktivasi @pytest.mark.skip.
# - Pastikan semua file di huskybot_calibration/huskybot_calibration/ dan scripts/ sudah ada header copyright.
# - Jalankan test ini secara otomatis di pipeline CI/CD sebelum merge/publish.
