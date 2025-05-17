import pytest  # Untuk framework testing Python dan fixture tmp_path (pytest bawaan ROS2 linter/test)
import yaml  # Untuk baca/tulis file YAML (validasi output sinkronisasi)
import csv  # Untuk baca/tulis file CSV (validasi output sinkronisasi)
from huskybot_calibration.huskybot_calibration.sync_sensor_time import SensorTimeSynchronizer  # Import class utama yang akan diuji (OOP, pipeline sinkronisasi sensor)

def test_validate_output_files(tmp_path):  # Test validasi file output YAML+CSV yang valid
    yaml_path = tmp_path / "sync_data.yaml"  # Path file YAML dummy (temp)
    csv_path = tmp_path / "sync_data.csv"  # Path file CSV dummy (temp)
    with open(yaml_path, "w") as f:  # Simpan file YAML dummy
        yaml.dump([{"camera_stamp": {"sec": 1, "nanosec": 2}}], f)  # Data minimal agar validator tidak error
    with open(csv_path, "w", newline="") as f:  # Simpan file CSV dummy
        writer = csv.writer(f)
        writer.writerow(["camera_sec", "camera_nanosec", "lidar_sec", "lidar_nanosec", "imu_sec", "imu_nanosec"])  # Header sesuai pipeline
        writer.writerow([1, 2, 3, 4, 5, 6])  # Satu baris data dummy
    node = SensorTimeSynchronizer()  # Buat instance class yang akan diuji (OOP, tidak perlu ROS2 spin)
    node.output_file = str(yaml_path)  # Set path file YAML ke dummy
    node.output_csv = str(csv_path)  # Set path file CSV ke dummy
    assert node.validate_output_files() is True  # Test lolos jika validator return True

def test_validate_output_files_yaml_corrupt(tmp_path):  # Test error handling jika file YAML corrupt
    yaml_path = tmp_path / "corrupt.yaml"  # Path file YAML corrupt
    csv_path = tmp_path / "sync_data.csv"  # Path file CSV dummy
    with open(yaml_path, "w") as f:
        f.write("camera_stamp: [this is not valid yaml")  # Simpan YAML corrupt
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["camera_sec", "camera_nanosec", "lidar_sec", "lidar_nanosec", "imu_sec", "imu_nanosec"])
        writer.writerow([1, 2, 3, 4, 5, 6])
    node = SensorTimeSynchronizer()  # Buat instance class yang akan diuji
    node.output_file = str(yaml_path)  # Set path file YAML corrupt
    node.output_csv = str(csv_path)  # Set path file CSV dummy
    assert node.validate_output_files() is False  # Harus False jika YAML corrupt

def test_validate_output_files_csv_corrupt(tmp_path):  # Test error handling jika file CSV corrupt (header salah)
    yaml_path = tmp_path / "sync_data.yaml"  # Path file YAML dummy
    csv_path = tmp_path / "corrupt.csv"  # Path file CSV corrupt
    with open(yaml_path, "w") as f:
        yaml.dump([{"camera_stamp": {"sec": 1, "nanosec": 2}}], f)
    with open(csv_path, "w", newline="") as f:
        f.write("not,a,valid,csv\n")  # Header salah
    node = SensorTimeSynchronizer()  # Buat instance class yang akan diuji
    node.output_file = str(yaml_path)  # Set path file YAML dummy
    node.output_csv = str(csv_path)  # Set path file CSV corrupt
    assert node.validate_output_files() is False  # Harus False jika CSV corrupt

def test_validate_output_files_missing_file(tmp_path):  # Test error handling jika file tidak ada
    yaml_path = tmp_path / "not_exist.yaml"  # Path file YAML tidak ada
    csv_path = tmp_path / "not_exist.csv"  # Path file CSV tidak ada
    node = SensorTimeSynchronizer()  # Buat instance class yang akan diuji
    node.output_file = str(yaml_path)  # Set path file YAML tidak ada
    node.output_csv = str(csv_path)  # Set path file CSV tidak ada
    assert node.validate_output_files() is False  # Harus False jika file tidak ada

def test_validate_output_files_yaml_missing_field(tmp_path):  # Test error handling jika field wajib di YAML kurang
    yaml_path = tmp_path / "missing_field.yaml"
    csv_path = tmp_path / "sync_data.csv"
    with open(yaml_path, "w") as f:
        yaml.dump([{"wrong_field": 123}], f)  # Field wajib hilang
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["camera_sec", "camera_nanosec", "lidar_sec", "lidar_nanosec", "imu_sec", "imu_nanosec"])
        writer.writerow([1, 2, 3, 4, 5, 6])
    node = SensorTimeSynchronizer()
    node.output_file = str(yaml_path)
    node.output_csv = str(csv_path)
    assert node.validate_output_files() is False  # Harus False jika field kurang

def test_validate_output_files_csv_missing_header(tmp_path):  # Test error handling jika header CSV kurang
    yaml_path = tmp_path / "sync_data.yaml"
    csv_path = tmp_path / "missing_header.csv"
    with open(yaml_path, "w") as f:
        yaml.dump([{"camera_stamp": {"sec": 1, "nanosec": 2}}], f)
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["camera_sec", "camera_nanosec"])  # Header kurang
        writer.writerow([1, 2])
    node = SensorTimeSynchronizer()
    node.output_file = str(yaml_path)
    node.output_csv = str(csv_path)
    assert node.validate_output_files() is False  # Harus False jika header kurang

# Penjelasan:
# - Setiap test di atas memastikan fungsi validate_output_files() di SensorTimeSynchronizer robust terhadap file corrupt, field kurang, header salah, dan file tidak ada.
# - Test ini tidak tergantung node ROS2 aktif, hanya test fungsi OOP dan file I/O.
# - Sudah siap untuk pipeline ROS2 Humble, simulasi Gazebo, dan robot real.
# - Test ini penting agar pipeline sinkronisasi sensor tidak error saat file output dipakai node lain.

# Saran peningkatan:
# - Tambahkan test untuk permission error (opsional, sulit di CI, tapi bisa dengan monkeypatch open).
# - Dokumentasikan format file output di README agar user lain tahu struktur file yang divalidasi.
# - Tambahkan test untuk validasi isi data (misal: nilai timestamp harus positif, dsb) jika pipeline makin kompleks.
# - Jika ingin coverage lebih tinggi, tambahkan test untuk edge case (file kosong, file besar, dsb).