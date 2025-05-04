#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys                                 # Modul sys untuk akses argumen command line dan error output
import rclpy                               # Modul utama ROS2 Python
from gazebo_msgs.srv import SpawnEntity    # Import service untuk spawn entity di Gazebo

def main(args=None):                       # Fungsi utama script
    # ---------- Cek jumlah argumen ----------
    if len(sys.argv) < 2:                  # Jika argumen kurang dari 2 (script + file robot)
        print("Usage: spawn_huskybot.py <robot.urdf/xacro/sdf> [entity_name] [namespace] [reference_frame]", file=sys.stderr)  # Print cara pakai
        sys.exit(1)                        # Exit dengan error

    robot_file = sys.argv[1]               # Argumen 1: path file URDF/Xacro/SDF robot
    entity_name = sys.argv[2] if len(sys.argv) > 2 else "huskybot"         # Argumen 2: nama entity di Gazebo (opsional, default "huskybot")
    robot_namespace = sys.argv[3] if len(sys.argv) > 3 else ""             # Argumen 3: namespace robot (opsional, default "")
    reference_frame = sys.argv[4] if len(sys.argv) > 4 else "world"        # Argumen 4: reference frame (opsional, default "world")

    rclpy.init(args=args)                  # Inisialisasi ROS2 Python
    node = rclpy.create_node('minimal_client')  # Membuat node ROS2 dengan nama 'minimal_client'
    cli = node.create_client(SpawnEntity, '/spawn_entity')  # Membuat client service ke /spawn_entity

    # ---------- Baca isi file robot ----------
    with open(robot_file, 'r') as content_file:   # Buka file robot
        content = content_file.read()             # Baca seluruh isi file ke variabel content

    req = SpawnEntity.Request()           # Membuat request untuk service SpawnEntity
    req.name = entity_name                # Nama entity di Gazebo (dari argumen)
    req.xml = content                     # Isi URDF/Xacro/SDFormat robot
    req.robot_namespace = robot_namespace # Namespace robot (dari argumen)
    req.reference_frame = reference_frame # Frame referensi spawn (dari argumen)

    while not cli.wait_for_service(timeout_sec=1.0):  # Tunggu service /spawn_entity siap
        node.get_logger().info('service not available, waiting again...')  # Logging info jika service belum siap

    future = cli.call_async(req)          # Kirim request spawn entity secara async
    rclpy.spin_until_future_complete(node, future)  # Tunggu hingga service selesai

    if future.result() is not None:       # Jika ada hasil dari service
        node.get_logger().info(
            'Result ' + str(future.result().success) + " " + future.result().status_message)  # Print status spawn
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))  # Print error jika gagal

    node.destroy_node()                   # Destroy node setelah selesai
    rclpy.shutdown()                      # Shutdown ROS2

if __name__ == '__main__':                # Jika script dijalankan langsung
    main()                                # Panggil fungsi main