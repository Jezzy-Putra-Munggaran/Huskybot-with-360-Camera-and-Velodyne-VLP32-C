std_msgs/Header header
string label
float32[3] center    # [x, y, z] center of bounding box (meter)
float32[3] size      # [dx, dy, dz] size of bounding box (meter)
float32[4] orientation  # quaternion [x, y, z, w]
float32 confidence