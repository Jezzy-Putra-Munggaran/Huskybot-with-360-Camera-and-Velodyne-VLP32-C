[![](https://github.com/ros-drivers/velodyne/workflows/Basic%20Build%20Workflow/badge.svg?branch=ros2)](https://github.com/ros-drivers/velodyne/actions)

Overview
========

Velodyne<sup>1</sup> is a collection of ROS<sup>2</sup> packages supporting `Velodyne high
definition 3D LIDARs`<sup>3</sup>.

**Warning**:

  The `<ros_distro>-devel` branch normally contains code being tested for the next
  ROS release.  It will not always work with every previous release.
  To check out the source for the most recent release, check out the
  tag `ros2-<version>` with the highest version number.

The current ``dashing-devel`` branch works with ROS Dashing.

- <sup>1</sup>Velodyne: http://www.ros.org/wiki/velodyne
- <sup>2</sup>ROS: http://www.ros.org
- <sup>3</sup>`Velodyne high definition 3D LIDARs`: http://www.velodynelidar.com/lidar/lidar.aspx

# velodyne

[![Build Status](https://github.com/ros-drivers/velodyne/actions/workflows/Basic%20Build%20Workflow/badge.svg?branch=ros2)](https://github.com/ros-drivers/velodyne/actions)

Driver official Velodyne untuk ROS2 (VLP-32C dan tipe lain).

---

## Fitur
- Driver untuk hardware Velodyne.
- Konversi raw packet ke point cloud (`/velodyne_points`).
- Support berbagai model Velodyne.

---

## Struktur Folder
- `velodyne_driver/` : Driver hardware.
- `velodyne_pointcloud/` : Konversi point cloud.
- `launch/` : Launch file driver.

---

## Cara Pakai

**Jalankan driver untuk VLP-32C:**
```sh
ros2 launch velodyne_driver velodyne_driver_node-VLP32C-launch.py device_ip:=192.168.1.201 frame_id:=velodyne rpm:=600 port:=2368
```

---

## Saran CI
- Ikuti workflow CI dari upstream [ros-drivers/velodyne](https://github.com/ros-drivers/velodyne).

---

## Catatan
Pastikan branch yang digunakan adalah `ros2` dan sudah build dengan benar.

## Diagram Driver

```
[Velodyne Hardware] --> [velodyne_driver_node] --> /velodyne_packets
                                    |
                                    v
                        [velodyne_convert_node] --> /velodyne_points
```

## Contoh Parameter Driver

```yaml
device_ip: 192.168.1.201
model: 32C
rpm: 600
port: 2368
frame_id: velodyne
```

## Troubleshooting

- Jika tidak ada data, cek IP dan koneksi hardware.
- Jika point cloud kosong, cek parameter model dan calibration.