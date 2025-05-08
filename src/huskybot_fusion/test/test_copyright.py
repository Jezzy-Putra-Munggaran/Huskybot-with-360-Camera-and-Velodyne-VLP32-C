# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");  # Lisensi file, wajib untuk open source compliance
# you may not use this file except in compliance with the License.  # Penjelasan lisensi
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0  # Link ke lisensi Apache-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_copyright.main import main  # Import fungsi utama untuk cek copyright header
import pytest  # Import pytest untuk framework unit test

# Remove the `skip` decorator once the source file(s) have a copyright header
@pytest.mark.skip(reason='No copyright header has been placed in the generated source file.')  # Skip test jika belum ada copyright
@pytest.mark.copyright  # Mark test ini sebagai copyright check
@pytest.mark.linter  # Mark test ini sebagai linter check (untuk CI/CD)
def test_copyright():
    rc = main(argv=['.', 'test'])  # Jalankan ament_copyright di folder ini
    assert rc == 0, 'Found errors'  # Assert hasilnya harus 0 (tidak ada error)

# ===================== REVIEW & SARAN =====================
# - File ini adalah test otomatis untuk memastikan semua file source di package ini memiliki copyright header.
# - Sudah terhubung dengan sistem linting ROS2 (ament_lint_auto) dan bisa dijalankan via colcon test.
# - Sudah best practice untuk CI/CD di ROS2 workspace.
# - FULL OOP tidak relevan di file test ini, karena hanya test linting.
# - Error handling: test otomatis akan gagal jika ada file tanpa copyright.
# - Saran peningkatan:
#   1. Setelah semua file source (Python, CMake, dsb) diberi copyright header, hapus/deaktivasi @pytest.mark.skip.
#   2. Tambahkan test lain di folder test/ untuk flake8, pep257, dan unit test fungsi utama (lihat test_flake8.py, test_pep257.py).
#   3. Pastikan test ini dijalankan di pipeline CI (GitHub Actions, dsb).
#   4. Untuk audit, tambahkan badge CI di README.md agar status test selalu terlihat.
# - Semua baris sudah diberi komentar penjelasan.
# - Tidak ada bug/error, sudah best practice ROS2 Python package.