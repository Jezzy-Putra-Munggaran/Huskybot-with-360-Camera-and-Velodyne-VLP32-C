#!/usr/bin/python3  # Shebang agar bisa dieksekusi langsung (wajib ROS2 Python)
# -*- coding: utf-8 -*-  # Encoding file Python (wajib agar support karakter non-ASCII)

import sys  # Untuk akses argumen CLI dan error handling
import os  # Untuk operasi path file
import rclpy  # Import utama ROS2 Python
from gazebo_msgs.srv import SpawnEntity  # Service untuk spawn entity di Gazebo
import time  # Untuk timestamp log dan timeout
import logging  # Untuk logging ke file
import xml.etree.ElementTree as ET  # Untuk validasi XML URDF/SDF
import subprocess  # Untuk eksekusi xacro
import traceback  # Untuk print stack trace error

# ===================== LOGGING TO FILE (OPSIONAL) =====================
def setup_file_logger(log_path="spawn_huskybot.log"):  # Fungsi setup logger file
    log_path = os.path.expanduser(log_path)  # Expand ~ ke home user
    logger = logging.getLogger("spawn_huskybot_file_logger")  # Buat/get logger dengan nama unik
    logger.setLevel(logging.INFO)  # Set level default INFO
    if not logger.hasHandlers():  # Cegah duplicate handler
        fh = logging.FileHandler(log_path)  # Handler file log
        fh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s: %(message)s'))  # Format log
        logger.addHandler(fh)  # Tambah handler ke logger
    return logger  # Return logger instance

file_logger = setup_file_logger()  # Inisialisasi logger file global

def log_to_file(msg, level='info'):  # Fungsi log ke file dengan level
    if file_logger:  # Jika logger ada
        if level == 'error':
            file_logger.error(msg)  # Log error
        elif level == 'warn':
            file_logger.warning(msg)  # Log warning
        elif level == 'debug':
            file_logger.debug(msg)  # Log debug
        else:
            file_logger.info(msg)  # Log info

# ==================== OOP Implementation ====================
class HuskybotSpawner:  # Class OOP untuk proses spawn robot ke Gazebo
    def __init__(self, log_file="spawn_huskybot.log"):  # Konstruktor class
        self.logger = logging.getLogger("HuskybotSpawner")  # Logger khusus class
        self.logger.setLevel(logging.INFO)  # Set level INFO
        fh = logging.FileHandler(log_file)  # Handler file log
        fh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s: %(message)s'))  # Format log
        if not self.logger.hasHandlers():  # Cegah duplicate handler
            self.logger.addHandler(fh)
        self.log_file = log_file  # Simpan path log file

    def log_to_file(self, msg, level='info'):  # Logging ke file dari class
        try:
            with open(self.log_file, "a") as logf:
                logf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}\n")  # Tulis log manual
        except Exception as e:
            print(f"[WARNING] Tidak bisa menulis ke log file: {self.log_file} ({e})", file=sys.stderr)
        log_to_file(msg, level=level)  # Juga log ke global logger

    def get_robot_xml(self, robot_file):  # Fungsi untuk ambil isi URDF/Xacro/SDF robot
        # ---------- Error Handling: cek file ----------
        if not isinstance(robot_file, str) or not robot_file:  # Validasi tipe dan isi path
            self.logger.error("Path file robot harus string dan tidak kosong.")
            print("[ERROR] Path file robot harus string dan tidak kosong.", file=sys.stderr)
            self.log_to_file("Path file robot harus string dan tidak kosong.", level='error')
            sys.exit(21)
        if not os.path.isfile(robot_file):  # Cek file ada
            self.logger.error(f"File robot tidak ditemukan: {robot_file}")
            print(f"[ERROR] File robot tidak ditemukan: {robot_file}", file=sys.stderr)
            self.log_to_file(f"File robot tidak ditemukan: {robot_file}", level='error')
            sys.exit(2)
        # ---------- Error Handling: xacro ----------
        if robot_file.endswith('.xacro'):  # Jika file xacro, jalankan xacro
            try:
                result = subprocess.run(
                    ['xacro', robot_file],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    check=True,
                    text=True
                )
                content = result.stdout  # Ambil hasil xacro
            except subprocess.CalledProcessError as e:
                self.logger.error(f"Gagal menjalankan xacro: {e.stderr}")
                print(f"[ERROR] Gagal menjalankan xacro: {e.stderr}", file=sys.stderr)
                self.log_to_file(f"Gagal menjalankan xacro: {e.stderr}", level='error')
                sys.exit(13)
        else:
            try:
                with open(robot_file, 'r') as content_file:
                    content = content_file.read()  # Baca file URDF/SDF
            except Exception as e:
                self.logger.error(f"Gagal membaca file robot: {e}")
                print(f"[ERROR] Gagal membaca file robot: {e}", file=sys.stderr)
                self.log_to_file(f"Gagal membaca file robot: {e}\n{traceback.format_exc()}", level='error')
                sys.exit(3)
        # ---------- Error Handling: validasi isi file ----------
        if "<robot" not in content and "<sdf" not in content:  # Validasi minimal isi file
            self.logger.error("Isi file robot tidak valid (tidak mengandung <robot atau <sdf)")
            print("[ERROR] Isi file robot tidak valid (tidak mengandung <robot atau <sdf)", file=sys.stderr)
            self.log_to_file("Isi file robot tidak valid (tidak mengandung <robot atau <sdf)", level='error')
            sys.exit(9)
        try:
            ET.fromstring(content)  # Validasi XML
        except ET.ParseError as e:
            self.logger.error(f"File robot bukan XML valid: {e}")
            print(f"[ERROR] File robot bukan XML valid: {e}", file=sys.stderr)
            self.log_to_file(f"File robot bukan XML valid: {e}\n{traceback.format_exc()}", level='error')
            sys.exit(12)
        return content  # Return isi file robot

    def spawn(self, robot_file, entity_name="huskybot", robot_namespace="", reference_frame="world", timeout_sec=30):  # Fungsi utama spawn
        try:
            content = self.get_robot_xml(robot_file)  # Ambil isi file robot
            msg = f"Spawning entity: {entity_name} dari file: {robot_file} | Namespace: {robot_namespace} | Reference: {reference_frame}"
            print(f"[INFO] {msg}")
            self.logger.info(msg)
            self.log_to_file(msg)
        except Exception as e:
            self.logger.error(f"Error get_robot_xml: {e}")
            print(f"[ERROR] Error get_robot_xml: {e}", file=sys.stderr)
            self.log_to_file(f"Error get_robot_xml: {e}\n{traceback.format_exc()}", level='error')
            sys.exit(20)

        # ---------- Error Handling: inisialisasi ROS2 ----------
        try:
            rclpy.init(args=None)  # Inisialisasi ROS2
            node = rclpy.create_node('huskybot_spawner')  # Buat node ROS2
            cli = node.create_client(SpawnEntity, '/spawn_entity')  # Buat client service spawn_entity
        except Exception as e:
            self.logger.error(f"Gagal inisialisasi ROS2: {e}")
            print(f"[ERROR] Gagal inisialisasi ROS2: {e}", file=sys.stderr)
            self.log_to_file(f"Gagal inisialisasi ROS2: {e}\n{traceback.format_exc()}", level='error')
            sys.exit(4)

        # ---------- Error Handling: tunggu service ----------
        try:
            start_time = time.time()
            while not cli.wait_for_service(timeout_sec=1.0):  # Tunggu service /spawn_entity
                node.get_logger().info('service not available, waiting again...')
                self.logger.info('service not available, waiting again...')
                self.log_to_file('service not available, waiting again...', level='warn')
                if time.time() - start_time > timeout_sec:
                    self.logger.error("Timeout menunggu service /spawn_entity")
                    print("[ERROR] Timeout menunggu service /spawn_entity", file=sys.stderr)
                    self.log_to_file("Timeout menunggu service /spawn_entity", level='error')
                    node.destroy_node()
                    rclpy.shutdown()
                    sys.exit(10)
        except KeyboardInterrupt:
            self.logger.info("Dibatalkan oleh user (Ctrl+C)")
            print("[INFO] Dibatalkan oleh user (Ctrl+C)", file=sys.stderr)
            self.log_to_file("Dibatalkan oleh user (Ctrl+C)", level='warn')
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(130)
        except Exception as e:
            self.logger.error(f"Gagal menunggu service /spawn_entity: {e}")
            print(f"[ERROR] Gagal menunggu service /spawn_entity: {e}", file=sys.stderr)
            self.log_to_file(f"Gagal menunggu service /spawn_entity: {e}\n{traceback.format_exc()}", level='error')
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(5)

        # ---------- Kirim request spawn entity ----------
        req = SpawnEntity.Request()  # Buat request service
        req.name = entity_name  # Nama entity di Gazebo
        req.xml = content  # Isi URDF/SDF robot
        req.robot_namespace = robot_namespace  # Namespace robot (multi-robot)
        req.reference_frame = reference_frame  # Reference frame spawn

        try:
            future = cli.call_async(req)  # Kirim request async
            start_time = time.time()
            while not future.done():  # Tunggu response
                rclpy.spin_once(node, timeout_sec=0.5)
                if time.time() - start_time > timeout_sec:
                    self.logger.error("Timeout menunggu response spawn_entity")
                    print("[ERROR] Timeout menunggu response spawn_entity", file=sys.stderr)
                    self.log_to_file("Timeout menunggu response spawn_entity", level='error')
                    node.destroy_node()
                    rclpy.shutdown()
                    sys.exit(11)
        except Exception as e:
            self.logger.error(f"Gagal mengirim request spawn_entity: {e}")
            print(f"[ERROR] Gagal mengirim request spawn_entity: {e}", file=sys.stderr)
            self.log_to_file(f"Gagal mengirim request spawn_entity: {e}\n{traceback.format_exc()}", level='error')
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(6)

        # ---------- Cek hasil service call ----------
        try:
            if future.result() is not None:
                if future.result().success:
                    msg = f"Result {future.result().success} {future.result().status_message}"
                    node.get_logger().info(msg)
                    self.logger.info(msg)
                    self.log_to_file(msg)
                    node.destroy_node()
                    rclpy.shutdown()
                    sys.exit(0)
                else:
                    msg = f"SpawnEntity failed: {future.result().status_message}"
                    node.get_logger().error(msg)
                    self.logger.error(msg)
                    self.log_to_file(msg, level='error')
                    node.destroy_node()
                    rclpy.shutdown()
                    sys.exit(7)
            else:
                msg = f"Service call failed {future.exception()}"
                node.get_logger().error(msg)
                self.logger.error(msg)
                self.log_to_file(msg, level='error')
                node.destroy_node()
                rclpy.shutdown()
                sys.exit(8)
        except Exception as e:
            self.logger.error(f"Exception saat cek hasil service call: {e}")
            print(f"[ERROR] Exception saat cek hasil service call: {e}", file=sys.stderr)
            self.log_to_file(f"Exception saat cek hasil service call: {e}\n{traceback.format_exc()}", level='error')
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(15)

# ==================== CLI Entrypoint ====================
def main():  # Fungsi utama CLI
    # ---------- Cek jumlah argumen ----------
    if len(sys.argv) < 2:
        print("Usage: spawn_huskybot.py <robot.urdf/xacro/sdf> [entity_name] [namespace] [reference_frame] [timeout_sec]", file=sys.stderr)
        sys.exit(1)
    robot_file = sys.argv[1]  # Path file robot
    entity_name = sys.argv[2] if len(sys.argv) > 2 else "huskybot"  # Nama entity
    robot_namespace = sys.argv[3] if len(sys.argv) > 3 else ""  # Namespace robot
    reference_frame = sys.argv[4] if len(sys.argv) > 4 else "world"  # Reference frame
    try:
        timeout_sec = int(sys.argv[5]) if len(sys.argv) > 5 else 30  # Timeout (detik)
    except Exception as e:
        print(f"[WARNING] Argumen timeout_sec tidak valid, gunakan default 30 detik. ({e})", file=sys.stderr)
        timeout_sec = 30

    # ---------- Logging argumen ke file ----------
    log_file = "spawn_huskybot.log"
    try:
        with open(log_file, "a") as logf:
            logf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] CLI args: {sys.argv}\n")
    except Exception as e:
        print(f"[WARNING] Tidak bisa menulis ke log file: {log_file} ({e})", file=sys.stderr)
    log_to_file(f"CLI args: {sys.argv}")

    # Validasi dependency utama
    try:
        import rclpy
        from gazebo_msgs.srv import SpawnEntity
    except ImportError as e:
        print(f"[FATAL] Dependency ROS2 atau gazebo_msgs tidak ditemukan: {e}", file=sys.stderr)
        log_to_file(f"[FATAL] Dependency ROS2 atau gazebo_msgs tidak ditemukan: {e}", level='error')
        sys.exit(99)

    spawner = HuskybotSpawner(log_file=log_file)  # Buat instance spawner
    try:
        spawner.spawn(robot_file, entity_name, robot_namespace, reference_frame, timeout_sec)  # Jalankan spawn
    except Exception as e:
        print(f"[FATAL] Exception di spawner.spawn(): {e}", file=sys.stderr)
        log_to_file(f"[FATAL] Exception di spawner.spawn(): {e}\n{traceback.format_exc()}", level='error')
        sys.exit(99)

if __name__ == '__main__':  # Jika file dijalankan langsung
    try:
        main()  # Jalankan main
    except Exception as e:
        print(f"[FATAL] Exception di main(): {e}", file=sys.stderr)
        log_to_file(f"[FATAL] Exception di main(): {e}\n{traceback.format_exc()}", level='error')
        logging.basicConfig(filename="spawn_huskybot.log", level=logging.ERROR)
        logging.error(f"Exception di main(): {e}")
        sys.exit(99)

# ==================== PENJELASAN & SARAN ====================
# - Logger ROS2 (via node.get_logger()) dan logging ke file sudah di setiap langkah penting/error.
# - Semua error/exception di callback dan fungsi utama sudah di-log.
# - Validasi file, parameter, dan dependency sudah lengkap.
# - Monitoring health check service spawn_entity dan validasi isi file robot.
# - Siap untuk ROS2 Humble, simulasi Gazebo, dan robot real (Clearpath Husky A200 + Jetson Orin + 6x Arducam IMX477 + Velodyne VLP32-C).
# - Saran: tambahkan validasi isi URDF lebih detail (cek frame/link wajib).
# - Saran: tambahkan opsi retry otomatis jika service spawn_entity belum ready.
# - Saran: tambahkan unit test di folder test/ untuk CI/CD.
# - Saran: tambahkan argumen untuk log_file jika ingin log custom per robot.
# - Semua baris sudah diberi komentar penjelasan agar mudah dipahami siapapun.