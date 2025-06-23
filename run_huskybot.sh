#!/bin/bash
# filepath: /home/jezzy/huskybot/run_huskybot.sh

# Display help function
show_help() {
    echo "Huskybot Pipeline Launcher"
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  --detection     Use the detection pipeline (YOLOv12)"
    echo "  --segmentation  Use the segmentation pipeline (YOLOv11-seg)"
    echo "  --sim           Run in simulation mode (Gazebo)"
    echo "  --real          Run on real hardware"
    echo "  --help          Show this help message"
}

# Default values
MODEL_TYPE="detection"
RUN_MODE="sim"

# Parse arguments
while [ "$1" != "" ]; do
    case $1 in
        --detection)
            MODEL_TYPE="detection"
            ;;
        --segmentation)
            MODEL_TYPE="segmentation"
            ;;
        --sim)
            RUN_MODE="sim"
            ;;
        --real)
            RUN_MODE="real"
            ;;
        --help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
    shift
done

# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/huskybot/install/setup.bash

# Print configuration
echo "=== Huskybot Launch ==="
echo "Model: $MODEL_TYPE"
echo "Mode: $RUN_MODE"
echo "======================="

# Run depending on mode
if [ "$RUN_MODE" = "sim" ]; then
    echo "Starting Gazebo simulation..."
    # Launch Gazebo + robot + perception
    ros2 launch huskybot_gazebo huskybot_launch.py &
    GAZEBO_PID=$!
    
    # Wait for Gazebo to start
    sleep 5
    
    # Launch perception with selected model
    echo "Starting perception with $MODEL_TYPE model..."
    ros2 launch huskybot_perception main.launch.py model_type:=$MODEL_TYPE
    
    # Clean up
    kill $GAZEBO_PID
else
    # Launch on real hardware
    echo "Starting on real hardware with $MODEL_TYPE model..."
    ros2 launch huskybot_perception main.launch.py model_type:=$MODEL_TYPE
fi