#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node

def main():
    """Create RViz2 configuration for 3D visualization"""
    
    # ✅ FIXED: Use more general paths that work across systems
    config_dir = os.path.expanduser('~/huskybot/install/huskybot_perception/share/huskybot_perception/rviz')
    if not os.path.exists(config_dir):
        try:
            # Try alternative path
            alt_path = '/home/kmp-orin/jezzy/huskybot/install/huskybot_perception/share/huskybot_perception/rviz'
            if os.path.exists(os.path.dirname(alt_path)):
                config_dir = alt_path
            else:
                # Create it if it doesn't exist
                os.makedirs(config_dir, exist_ok=True)
        except:
            # Final fallback
            config_dir = '/tmp'
    
    config_file = os.path.join(config_dir, 'huskybot_3d.rviz')
    
    print(f"✅ Creating RViz2 config at: {config_file}")
    
    # ✅ ENHANCED RViz2 config for 3D PointCloud visualization
    rviz_config = """Panels:
  - Class: rviz_common/Displays
    Help Height: 138
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /3D Objects PointCloud1
        - /Velodyne PointCloud1
        - /TF1
      Splitter Ratio: 0.5
    Tree Height: 719
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 20
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: rgb
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: RGB8
      Decay Time: 10
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: 3D Objects PointCloud
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 15
      Size (m): 0.5
      Style: Spheres
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /objects_3d_pointcloud
      Use Fixed Frame: true
      Use rainbow: false
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 60
  Name: root
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 25.0
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Field of View: 0.7853981852531433
      Focal Point:
        X: 5.0
        Y: 0
        Z: 2.0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.5853981971740723
      Target Frame: <Fixed Frame>
      Yaw: 0.785398006439209
    Saved: ~"""
    
    # Write config file
    with open(config_file, 'w') as f:
        f.write(rviz_config)
    
    print(f"✅ RViz2 config created: {config_file}")
    return True

if __name__ == '__main__':
    main()