from setuptools import setup, find_packages                # Import setup tools untuk instalasi package Python

package_name = 'huskybot_control'                          # Nama package, harus sama dengan folder dan package.xml

setup(
    name=package_name,                                     # Nama package
    version='0.1.0',                                       # Versi package, update jika ada perubahan besar
    packages=find_packages(),                              # Temukan semua sub-package Python (jika ada)
    data_files=[
        ('share/ament_index/resource_index/packages',      # File resource index ROS2 (wajib agar dikenali colcon/ament)
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),        # Install package.xml ke share/package_name
        ('share/' + package_name + '/launch', ['launch/huskybot_control.launch.py']),  # Install launch file ke share/package/launch
        # Tambahkan file lain jika perlu (misal: YAML config, README)
    ],
    install_requires=['setuptools'],                       # Dependency Python wajib
    zip_safe=True,                                         # Package aman untuk di-zip (standar ROS2)
    maintainer='Jezzy Putra Munggaran',                    # Nama maintainer
    maintainer_email='mungguran.jezzy.putra@gmail.com',    # Email maintainer
    description='Node kontrol dan monitoring Huskybot (teleop/manual/autonomous) untuk simulasi dan real robot.',  # Deskripsi singkat
    license='Apache-2.0',                                  # Lisensi package (harus sama dengan package.xml)
    tests_require=['pytest'],                              # Dependency test Python (opsional, best practice)
    entry_points={
        'console_scripts': [
            # Daftarkan semua script yang ingin bisa dijalankan via ros2 run
            'robot_control.py = scripts.robot_control:main',           # Entry point robot_control.py
            'safety_monitor.py = scripts.safety_monitor:main',         # Entry point safety_monitor.py
            'logger.py = scripts.logger:main',                         # Entry point logger.py
            # Tambahkan script lain jika ada
        ],
    },
)