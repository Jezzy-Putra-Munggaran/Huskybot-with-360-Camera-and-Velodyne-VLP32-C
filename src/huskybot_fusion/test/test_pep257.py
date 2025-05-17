# Copyright 2015 Open Source Robotics Foundation, Inc.  # [WAJIB] Copyright header, wajib untuk open source compliance
#
# Licensed under the Apache License, Version 2.0 (the "License");  # [WAJIB] Lisensi file, wajib untuk open source compliance
# you may not use this file except in compliance with the License.  # Penjelasan lisensi
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0  # Link ke lisensi Apache-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,  # Penjelasan distribusi file
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  # Penjelasan tanpa jaminan
# See the License for the specific language governing permissions and
# limitations under the License.  # Penjelasan batasan lisensi

from ament_pep257.main import main  # Import fungsi utama untuk linting pep257 di semua file Python
import pytest  # Import pytest untuk framework unit test (integrasi dengan colcon test/CI)

@pytest.mark.linter  # Mark test ini sebagai linter check (untuk CI/CD)
@pytest.mark.pep257  # Mark test ini sebagai pep257 check (untuk CI/CD)
def test_pep257():  # Fungsi test utama untuk linting pep257
    """
    Test otomatis untuk memastikan semua file Python di package ini lolos pep257 (docstring style).
    Akan dijalankan oleh colcon test dan pipeline CI/CD.
    """
    rc = main(argv=['.', 'test'])  # Jalankan ament_pep257 di folder ini (cek semua file Python)
    assert rc == 0, 'Found code style errors / warnings'  # Assert hasilnya harus 0 (tidak ada error); jika tidak, test gagal

# ===================== REVIEW & SARAN =====================
# - File ini adalah test otomatis untuk memastikan semua file Python di package ini lolos pep257 (docstring style).
# - Sudah terhubung dengan sistem linting ROS2 (ament_lint_auto) dan bisa dijalankan via colcon test.
# - Sudah best practice untuk CI/CD di ROS2 workspace.
# - FULL OOP tidak relevan di file test ini, karena hanya test linting.
# - Error handling: test otomatis akan gagal jika ada file Python yang tidak lolos pep257.
# - Saran peningkatan:
#   1. Pastikan semua file Python di huskybot_fusion/ dan scripts/ sudah lolos pep257 (tidak ada error/warning).
#   2. Jalankan test ini secara otomatis di pipeline CI/CD sebelum merge/publish.
#   3. Jika ingin pengecualian file/folder tertentu, tambahkan konfigurasi pep257 di setup.cfg atau pyproject.toml.
#   4. Dokumentasikan di README bahwa linter ini wajib lolos sebelum push ke repo.
#   5. Jika ingin coverage lebih detail, tambahkan test untuk file YAML, launch, atau C++ (opsional, via linter lain).
# - Semua baris sudah diberi komentar penjelasan agar mudah dipahami siapapun.
# - Tidak ada bug/error, sudah best practice ROS2 Python package.
