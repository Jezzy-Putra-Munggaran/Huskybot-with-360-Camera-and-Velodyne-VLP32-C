#!/bin/bash
set -e

# Display header and usage information
echo "======================================================"
echo "Huskybot Launcher - ROS2 Humble (2025)"
echo "======================================================"
echo "Usage: ./run_huskybot.sh [option]"
echo "Options:"
echo "  --sim        Run the Gazebo simulation"
echo "  --real       Run on the real hardware"
echo "  --detection  Use yolo12x.engine for detection" 
echo "  --segmentation  Use yolo11x-seg.engine for segmentation"
echo "  --debug      Run with verbose debug output"
echo "  --help       Display this help message"
echo "------------------------------------------------------"

# Default options
SIM_MODE="sim"
PERCEPTION_TYPE="detection"
DEBUG_MODE="false"

# Parse arguments
for arg in "$@"; do
  case $arg in
    --sim)
      SIM_MODE="sim"
      shift
      ;;
    --real)
      SIM_MODE="real"
      shift
      ;;
    --detection)
      PERCEPTION_TYPE="detection"
      shift
      ;;
    --segmentation)
      PERCEPTION_TYPE="segmentation"
      shift
      ;;
    --debug)
      DEBUG_MODE="true"
      shift
      ;;
    --help)
      exit 0
      ;;
    *)
      echo "Unknown option: $arg"
      exit 1
      ;;
  esac
done

# Clean up any stale processes
echo "Cleaning up stale processes..."
pkill -f "gzserver" 2>/dev/null || true
pkill -f "gzclient" 2>/dev/null || true
sleep 1

# Source ROS 2 environment
echo "Sourcing ROS 2 environment..."
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Running in $SIM_MODE mode with $PERCEPTION_TYPE perception..."

# Launch the appropriate configuration
if [ "$SIM_MODE" = "sim" ]; then
  # Simulation mode
  echo "Launching Gazebo simulation..."
  
  # Check if important directories exist
  if [ ! -d "src/huskybot_description" ]; then
    echo "ERROR: huskybot_description package not found"
    exit 1
  fi
  
  if [ ! -d "src/huskybot_gazebo" ]; then
    echo "ERROR: huskybot_gazebo package not found"
    exit 1
  fi
  
  # Force rebuild if needed
  if [ "$DEBUG_MODE" = "true" ]; then
    echo "Rebuilding workspace in debug mode..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    source install/setup.bash
  fi

  # Launch with model_type parameter
  ros2 launch huskybot_perception simulation.launch.py model_type:=$PERCEPTION_TYPE use_sim_time:=true

else
  # Real hardware mode
  echo "Launching on real hardware..."
  if [ "$PERCEPTION_TYPE" = "detection" ]; then
    ros2 launch huskybot_perception main.launch.py model_type:=detection
  else
    ros2 launch huskybot_perception main.launch.py model_type:=segmentation
  fi
fi