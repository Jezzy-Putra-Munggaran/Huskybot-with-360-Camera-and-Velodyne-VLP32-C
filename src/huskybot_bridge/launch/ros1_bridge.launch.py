from launch import LaunchDescription
from launch_ros.actions import Node
import os
import sys

def generate_launch_description():
    # Cari path absolut bridge.yaml (robust untuk semua lokasi run)
    config_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'config', 'bridge.yaml'
    ))

    # Robust: cek file config ada
    if not os.path.exists(config_path):
        print(f"[FATAL] bridge.yaml tidak ditemukan di {config_path}", file=sys.stderr)
        sys.exit(1)

    # Robust: cek package ros1_bridge sudah terinstall
    try:
        import importlib.util
        if importlib.util.find_spec("ros1_bridge") is None:
            print("[FATAL] Package ros1_bridge tidak ditemukan di environment ROS2!", file=sys.stderr)
            sys.exit(2)
    except Exception as e:
        print(f"[FATAL] Error saat cek ros1_bridge: {e}", file=sys.stderr)
        sys.exit(3)

    # Robust: cek dependency message type (geometry_msgs, nav_msgs, sensor_msgs)
    missing_msgs = []
    for pkg in ["geometry_msgs", "nav_msgs", "sensor_msgs"]:
        try:
            __import__(pkg)
        except ImportError:
            missing_msgs.append(pkg)
    if missing_msgs:
        print(f"[FATAL] Dependency message type tidak ditemukan: {', '.join(missing_msgs)}", file=sys.stderr)
        sys.exit(4)

    # Robust: node ros1_bridge dengan parameter config
    return LaunchDescription([
        Node(
            package='ros1_bridge',
            executable='dynamic_bridge',
            name='ros1_bridge',
            output='screen',
            parameters=[config_path],
            # remappings=[ ... ] # Tambahkan remap jika perlu
        )
    ])