#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import os
import rclpy
from gazebo_msgs.srv import SpawnEntity
import time
import logging
import xml.etree.ElementTree as ET
import subprocess
import traceback

# ===================== LOGGING TO FILE (OPSIONAL) =====================
def setup_file_logger(log_path="spawn_huskybot.log"):
    log_path = os.path.expanduser(log_path)
    logger = logging.getLogger("spawn_huskybot_file_logger")
    logger.setLevel(logging.INFO)
    if not logger.hasHandlers():
        fh = logging.FileHandler(log_path)
        fh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s: %(message)s'))
        logger.addHandler(fh)
    return logger

file_logger = setup_file_logger()

def log_to_file(msg, level='info'):
    if file_logger:
        if level == 'error':
            file_logger.error(msg)
        elif level == 'warn':
            file_logger.warning(msg)
        elif level == 'debug':
            file_logger.debug(msg)
        else:
            file_logger.info(msg)

# ==================== OOP Implementation ====================
class HuskybotSpawner:
    def __init__(self, log_file="spawn_huskybot.log"):
        self.logger = logging.getLogger("HuskybotSpawner")
        self.logger.setLevel(logging.INFO)
        fh = logging.FileHandler(log_file)
        fh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s: %(message)s'))
        if not self.logger.hasHandlers():
            self.logger.addHandler(fh)
        self.log_file = log_file

    def log_to_file(self, msg, level='info'):
        try:
            with open(self.log_file, "a") as logf:
                logf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}\n")
        except Exception as e:
            print(f"[WARNING] Tidak bisa menulis ke log file: {self.log_file} ({e})", file=sys.stderr)
        log_to_file(msg, level=level)

    def get_robot_xml(self, robot_file):
        # ---------- Error Handling: cek file ----------
        if not isinstance(robot_file, str) or not robot_file:
            self.logger.error("Path file robot harus string dan tidak kosong.")
            print("[ERROR] Path file robot harus string dan tidak kosong.", file=sys.stderr)
            self.log_to_file("Path file robot harus string dan tidak kosong.", level='error')
            sys.exit(21)
        if not os.path.isfile(robot_file):
            self.logger.error(f"File robot tidak ditemukan: {robot_file}")
            print(f"[ERROR] File robot tidak ditemukan: {robot_file}", file=sys.stderr)
            self.log_to_file(f"File robot tidak ditemukan: {robot_file}", level='error')
            sys.exit(2)
        # ---------- Error Handling: xacro ----------
        if robot_file.endswith('.xacro'):
            try:
                result = subprocess.run(
                    ['xacro', robot_file],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    check=True,
                    text=True
                )
                content = result.stdout
            except subprocess.CalledProcessError as e:
                self.logger.error(f"Gagal menjalankan xacro: {e.stderr}")
                print(f"[ERROR] Gagal menjalankan xacro: {e.stderr}", file=sys.stderr)
                self.log_to_file(f"Gagal menjalankan xacro: {e.stderr}", level='error')
                sys.exit(13)
        else:
            try:
                with open(robot_file, 'r') as content_file:
                    content = content_file.read()
            except Exception as e:
                self.logger.error(f"Gagal membaca file robot: {e}")
                print(f"[ERROR] Gagal membaca file robot: {e}", file=sys.stderr)
                self.log_to_file(f"Gagal membaca file robot: {e}\n{traceback.format_exc()}", level='error')
                sys.exit(3)
        # ---------- Error Handling: validasi isi file ----------
        if "<robot" not in content and "<sdf" not in content:
            self.logger.error("Isi file robot tidak valid (tidak mengandung <robot atau <sdf)")
            print("[ERROR] Isi file robot tidak valid (tidak mengandung <robot atau <sdf)", file=sys.stderr)
            self.log_to_file("Isi file robot tidak valid (tidak mengandung <robot atau <sdf)", level='error')
            sys.exit(9)
        try:
            ET.fromstring(content)
        except ET.ParseError as e:
            self.logger.error(f"File robot bukan XML valid: {e}")
            print(f"[ERROR] File robot bukan XML valid: {e}", file=sys.stderr)
            self.log_to_file(f"File robot bukan XML valid: {e}\n{traceback.format_exc()}", level='error')
            sys.exit(12)
        return content

    def spawn(self, robot_file, entity_name="huskybot", robot_namespace="", reference_frame="world", timeout_sec=30):
        try:
            content = self.get_robot_xml(robot_file)
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
            rclpy.init(args=None)
            node = rclpy.create_node('huskybot_spawner')
            cli = node.create_client(SpawnEntity, '/spawn_entity')
        except Exception as e:
            self.logger.error(f"Gagal inisialisasi ROS2: {e}")
            print(f"[ERROR] Gagal inisialisasi ROS2: {e}", file=sys.stderr)
            self.log_to_file(f"Gagal inisialisasi ROS2: {e}\n{traceback.format_exc()}", level='error')
            sys.exit(4)

        # ---------- Error Handling: tunggu service ----------
        try:
            start_time = time.time()
            while not cli.wait_for_service(timeout_sec=1.0):
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
        req = SpawnEntity.Request()
        req.name = entity_name
        req.xml = content
        req.robot_namespace = robot_namespace
        req.reference_frame = reference_frame

        try:
            future = cli.call_async(req)
            start_time = time.time()
            while not future.done():
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
def main():
    # ---------- Cek jumlah argumen ----------
    if len(sys.argv) < 2:
        print("Usage: spawn_huskybot.py <robot.urdf/xacro/sdf> [entity_name] [namespace] [reference_frame] [timeout_sec]", file=sys.stderr)
        sys.exit(1)
    robot_file = sys.argv[1]
    entity_name = sys.argv[2] if len(sys.argv) > 2 else "huskybot"
    robot_namespace = sys.argv[3] if len(sys.argv) > 3 else ""
    reference_frame = sys.argv[4] if len(sys.argv) > 4 else "world"
    try:
        timeout_sec = int(sys.argv[5]) if len(sys.argv) > 5 else 30
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

    spawner = HuskybotSpawner(log_file=log_file)
    try:
        spawner.spawn(robot_file, entity_name, robot_namespace, reference_frame, timeout_sec)
    except Exception as e:
        print(f"[FATAL] Exception di spawner.spawn(): {e}", file=sys.stderr)
        log_to_file(f"[FATAL] Exception di spawner.spawn(): {e}\n{traceback.format_exc()}", level='error')
        sys.exit(99)

if __name__ == '__main__':
    try:
        main()
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
# - Siap untuk ROS2 Humble, simulasi Gazebo, dan robot real.
# - Saran: tambahkan validasi isi URDF lebih detail (cek frame/link wajib).
# - Saran: tambahkan opsi retry otomatis jika service spawn_entity belum ready.
# - Saran: tambahkan unit test di folder test/ untuk CI/CD.
# - Saran: tambahkan argumen untuk log_file jika ingin log custom per robot.