std_msgs/Header header              # Header ROS2 (timestamp dan frame_id, untuk sinkronisasi waktu dan referensi frame)
InferenceResult[] yolov12_inference # Array hasil deteksi (list bounding box dan class, satu frame bisa banyak deteksi)
string camera_name                  # Nama kamera sumber deteksi (misal: "camera_front", "panorama", dsb)