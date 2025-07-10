#!/usr/bin/env python3  # Python interpreter specification (required for ROS2 launch files)
# -*- coding: utf-8 -*-  # Encoding declaration for proper Unicode support

"""
Main Launch File for Huskybot Perception

This is the primary launch file for huskybot_perception package,
supporting multiple YOLOv12 models (detection, segmentation, obb, tracking).
Designed for ROS2 Humble, Gazebo simulation, and Huskybot hardware
(Clearpath Husky A200 + Jetson AGX Orin + 6x Arducam IMX477 + Velodyne VLP32-C).

Author: Jezzy Putra Munggaran
License: Apache-2.0
"""

import os  # For path manipulation and environment variables (accessing files, directories, env vars)
import sys  # For system-level operations like exit codes and stderr output
import yaml  # For YAML file parsing (config validation and parameter loading)
import time  # For time-related functions like sleep and delays
import subprocess  # For running external commands to check system dependencies
import traceback  # For detailed stack traces in error logs (crucial for debugging)
from typing import List, Dict, Optional, Union, Tuple, Any  # Type hints for better code documentation and IDE support
import logging  # For structured logging to files (audit trail and debugging)

try:  # Try to import ROS-related packages with comprehensive error handling
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError  # Package path utilities for ROS2
    from launch import LaunchDescription  # Main class for ROS2 launch files - defines what to launch
    from launch.actions import (  # Launch actions for various launch functionalities
        IncludeLaunchDescription,  # For including other launch files
        DeclareLaunchArgument,  # For declaring launch arguments that can be passed when launching
        LogInfo,  # For logging informational messages during launch
        OpaqueFunction,  # For context-dependent operations that need the launch context
        RegisterEventHandler,  # For registering event handlers (like process exit)
        EmitEvent,  # For emitting events (like shutdown)
        TimerAction,  # For delayed actions
        GroupAction,  # For grouping actions together
        ExecuteProcess,  # For executing external processes
        LogError  # For logging error messages during launch
    )
    from launch.conditions import IfCondition, UnlessCondition  # For conditional execution based on parameter values
    from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown  # Event handlers for various process events
    from launch.events import Shutdown, ProcessExited  # Events that can be handled or emitted
    from launch.launch_description_sources import PythonLaunchDescriptionSource  # For including other Python launch files
    from launch.substitutions import (  # For parameter substitution in launch descriptions
        LaunchConfiguration,  # Access launch arguments 
        PythonExpression,  # Evaluate Python expressions during launch
        PathJoinSubstitution,  # Join paths with proper separators
        AndSubstitution,  # Logical AND of multiple substitutions
        OrSubstitution,  # Logical OR of multiple substitutions
        NotSubstitution  # Logical NOT of a substitution
    )
    from launch_ros.actions import Node, ComposableNodeContainer  # For launching ROS2 nodes and node containers
    from launch_ros.descriptions import ComposableNode  # For defining composable nodes
except ImportError as e:  # Handle import errors for ROS packages (critical for troubleshooting)
    print(f"[FATAL] Missing ROS2 dependencies: {e}", file=sys.stderr)
    print("[HINT] Make sure you've properly sourced your ROS2 installation:", file=sys.stderr) 
    print("       source /opt/ros/humble/setup.bash", file=sys.stderr)
    print("       Then ensure you have all required packages:", file=sys.stderr)
    print("       sudo apt install ros-humble-launch-ros python3-yaml", file=sys.stderr)
    sys.exit(1)  # Exit with error code 1 to indicate failure to the system

# Configure logging to file for this launch file - crucial for debugging and auditing
LOG_DIR = os.path.expanduser("~/huskybot_logs")  # Default log directory in user's home
try:
    os.makedirs(LOG_DIR, exist_ok=True)  # Create log directory if it doesn't exist
    logging.basicConfig(
        filename=os.path.join(LOG_DIR, "huskybot_perception_launch.log"),  # Log file path
        level=logging.INFO,  # Default log level (info and above)
        format='%(asctime)s - %(levelname)s - %(message)s',  # Include timestamp, level and message
        filemode='a'  # Append to existing file rather than overwrite
    )
    logging.info("=" * 80)  # Visual separator for new launch session
    logging.info("Starting huskybot_perception launch file execution")
except Exception as e:  # Handle errors creating log directory or setting up logging
    print(f"[WARNING] Could not set up logging to {LOG_DIR}: {e}", file=sys.stderr)
    print(f"[WARNING] Launch will continue but no logs will be saved", file=sys.stderr)

# ===================== CONSTANTS AND GLOBALS =====================

# Define valid model types for validation - must match available model files
VALID_MODEL_TYPES = ['detection', 'segmentation', 'obb', 'tracking']  # Supported YOLOv12 model types

# Paths for model files by type - used for validation
MODEL_FILE_PATTERNS = {  # Dictionary mapping model types to expected file patterns
    'detection': ['yolo12x.engine', 'yolo12x.engine', 'yolo12x.onnx'],  # Detection model file patterns
    'segmentation': ['yolo11x-seg.engine', 'yolo11x-seg.engine', 'yolo11x-seg.onnx'],  # Segmentation model file patterns
    'obb': ['yolo12x-obb.engine', 'yolo12x-obb.engine', 'yolo12x-obb.onnx'],  # OBB model file patterns
    'tracking': ['yolo12x-track.engine', 'yolo12x-track.engine', 'yolo12x-track.onnx']  # Tracking model file patterns
}

# Topic mapping for different model types - for validation and parameter passing
TOPIC_MAPPING = {  # Dictionary mapping model types to primary topics
    'detection': '/detection',  # Main topic for detection results
    'segmentation': '/segmentation',  # Main topic for segmentation results
    'obb': '/obb',  # Main topic for oriented bounding box results
    'tracking': '/tracking'  # Main topic for tracking results
}

# Configuration for automatic retry when packages are missing
MAX_RETRIES = 3  # Maximum number of retry attempts for finding packages
RETRY_DELAY = 2.0  # Delay between retries in seconds (allows for package system to update)

# Camera names in hexagonal configuration - used for validation and parameter passing
CAMERA_NAMES = ['front', 'front_left', 'left', 'rear', 'rear_right', 'right']  # Camera positions in 360° setup

# ===================== ERROR HANDLING & UTILITY FUNCTIONS =====================

def check_package_exists(package_name: str, retry: bool = True, retries: int = 0) -> bool:
    """
    Check if a ROS2 package exists and is accessible.
    Includes retry mechanism for packages that might be installing.
    
    Args:
        package_name: Name of the package to check
        retry: Whether to retry if package not found
        retries: Current retry count (for recursion)
        
    Returns:
        bool: True if package exists, False otherwise
    """
    try:
        get_package_share_directory(package_name)  # Try to get the package directory (throws if not found)
        logging.info(f"Package found: {package_name}")  # Log success
        return True  # Return True if package found
    except PackageNotFoundError:  # Package not found
        if retry and retries < MAX_RETRIES:  # If retries enabled and within limit
            logging.warning(f"Package '{package_name}' not found, retrying ({retries+1}/{MAX_RETRIES})...")
            print(f"[WARNING] Package '{package_name}' not found, retrying ({retries+1}/{MAX_RETRIES})...", 
                  file=sys.stderr)  # Print warning
            time.sleep(RETRY_DELAY)  # Wait before retry
            return check_package_exists(package_name, retry, retries + 1)  # Recursive retry
        logging.error(f"Package '{package_name}' not found after {retries} retries")
        return False  # Return False if package not found and retry disabled or limit reached
    except Exception as e:  # Other exceptions (file permissions, etc.)
        logging.error(f"Exception checking package {package_name}: {e}\n{traceback.format_exc()}")
        print(f"[ERROR] Exception checking package {package_name}: {e}", file=sys.stderr)  # Print error
        return False  # Return False for any other exception

def check_environment_variable(var_name: str, fallback: Optional[str] = None) -> Optional[str]:
    """
    Check if an environment variable exists and return its value.
    
    Args:
        var_name: Name of the environment variable
        fallback: Fallback value if variable doesn't exist
        
    Returns:
        str: Value of the environment variable or fallback
    """
    value = os.environ.get(var_name, fallback)  # Get environment variable with fallback
    if value is None:  # If no value and no fallback provided
        logging.warning(f"Environment variable '{var_name}' not set")
        print(f"[WARNING] Environment variable '{var_name}' not set", file=sys.stderr)  # Print warning
    return value  # Return value or fallback or None

def validate_launch_file(context, file_path: str) -> bool:
    """
    Validate that a launch file exists.
    
    Args:
        context: Launch context
        file_path: Full path to launch file
        
    Returns:
        bool: True if file exists, False otherwise
    """
    if not os.path.exists(file_path):  # Check if file exists
        logging.error(f"Launch file not found: {file_path}")
        print(f"[ERROR] Launch file not found: {file_path}", file=sys.stderr)  # Print error if not
        return False  # Return False if file not found
    
    # Additional check for read permission
    if not os.access(file_path, os.R_OK):  # Check read permission
        logging.error(f"Launch file exists but is not readable: {file_path}")
        print(f"[ERROR] Launch file exists but is not readable: {file_path}", file=sys.stderr)
        return False  # Return False if file not readable
    
    logging.info(f"Launch file validated: {file_path}")
    return True  # Return True if file exists and is readable

def check_model_type(context, model_type: str) -> bool:
    """
    Validate that model_type is one of the supported types.
    
    Args:
        context: Launch context
        model_type: Model type to validate
        
    Returns:
        bool: True if valid model type, False otherwise
    """
    if model_type not in VALID_MODEL_TYPES:  # Check if model type is valid
        logging.error(f"Invalid model_type '{model_type}'. Must be one of: {', '.join(VALID_MODEL_TYPES)}")
        print(f"[ERROR] Invalid model_type '{model_type}'. Must be one of: {', '.join(VALID_MODEL_TYPES)}", 
              file=sys.stderr)  # Print error with list of valid types
        return False  # Return False if invalid
    
    logging.info(f"Model type validated: {model_type}")
    return True  # Return True if valid

def check_hardware_compatibility() -> Dict[str, bool]:
    """
    Check hardware compatibility for specific Huskybot components.
    
    Returns:
        Dict[str, bool]: Dictionary of hardware components and their compatibility status
    """
    status = {
        'jetson': False,  # Jetson platform status (NVIDIA Jetson AGX Orin)
        'nvidia_gpu': False,  # NVIDIA GPU status (for non-Jetson NVIDIA GPUs)
        'tensorrt': False,  # TensorRT status (acceleration library)
        'velodyne': False,  # Velodyne driver status (for VLP32-C)
        'realtime_kernel': False,  # Realtime kernel status (optional for better timing)
        'cuda': False,  # CUDA availability status
    }
    
    # Check for Jetson platform via different indicators
    jetson_indicators = [
        '/etc/nv_tegra_release',  # Official Jetson indicator file
        '/proc/device-tree/model',  # Device tree model may contain Jetson information
        '/sys/devices/soc0/family'  # SOC family identifier
    ]
    
    for indicator in jetson_indicators:
        if os.path.exists(indicator):
            try:
                with open(indicator, 'r') as f:
                    content = f.read()
                    if 'jetson' in content.lower() or 'tegra' in content.lower() or 'xavier' in content.lower() or 'orin' in content.lower():
                        status['jetson'] = True
                        logging.info(f"Detected NVIDIA Jetson platform via {indicator}")
                        print(f"[INFO] Detected NVIDIA Jetson platform - optimizing for Jetson AGX Orin")
                        break
            except Exception as e:
                logging.warning(f"Could not read Jetson indicator file {indicator}: {e}")
    
    # Check for NVIDIA GPU
    try:
        result = subprocess.run(['nvidia-smi'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        if result.returncode == 0:  # If command successful
            status['nvidia_gpu'] = True  # Set NVIDIA GPU status to True
            status['cuda'] = True  # NVIDIA GPUs typically support CUDA
            logging.info("NVIDIA GPU detected via nvidia-smi - enabling CUDA acceleration")
            print("[INFO] NVIDIA GPU detected - enabling CUDA acceleration")  # Print info message
    except (subprocess.SubprocessError, FileNotFoundError):
        # Check for CUDA another way if nvidia-smi fails
        try:
            with open('/proc/driver/nvidia/version', 'r') as f:
                if 'NVIDIA' in f.read():
                    status['nvidia_gpu'] = True
                    status['cuda'] = True
                    logging.info("NVIDIA GPU detected via driver version file")
                    print("[INFO] NVIDIA GPU detected via driver version")
        except (FileNotFoundError, PermissionError):
            pass  # Skip silently if file doesn't exist or can't be read
    
    # Check for TensorRT - with proper error handling
    try:
        # Dynamically check for TensorRT without import error
        tensorrt_spec = subprocess.run(['python3', '-c', 'import tensorrt; print("TensorRT Found")'],
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        if tensorrt_spec.returncode == 0 and b"TensorRT Found" in tensorrt_spec.stdout:
            status['tensorrt'] = True  # Set TensorRT status to True if import successful
            logging.info("TensorRT detected - enabling TensorRT acceleration for YOLOv12")
            print("[INFO] TensorRT detected - enabling TensorRT acceleration for YOLOv12")
    except subprocess.SubprocessError:
        logging.info("TensorRT not found or not properly installed")
    
    # Check for Velodyne driver
    if check_package_exists('velodyne', False):  # Check for velodyne package (no retry)
        status['velodyne'] = True  # Set Velodyne status to True if package found
        logging.info("Velodyne driver detected - enabling VLP32-C support")
        print("[INFO] Velodyne driver detected - enabling VLP32-C support")
    
    # Check for realtime kernel
    try:
        uname_output = subprocess.check_output(['uname', '-a'], text=True)
        if 'PREEMPT_RT' in uname_output or 'preempt_rt' in uname_output:
            status['realtime_kernel'] = True
            logging.info("PREEMPT_RT realtime kernel detected - enabling optimized timing")
            print("[INFO] PREEMPT_RT realtime kernel detected - enabling optimized timing")
    except subprocess.SubprocessError:
        pass  # Skip silently if command fails
    
    return status  # Return hardware status dictionary

def check_for_yaml_params(context, model_type: str) -> Optional[str]:
    """
    Check for YAML parameter file for the specified model type.
    
    Args:
        context: Launch context
        model_type: Type of model to check parameters for
        
    Returns:
        Optional[str]: Path to YAML file if found, None otherwise
    """
    try:
        # Get package path
        package_path = get_package_share_directory('huskybot_perception')  # Get package path
        
        # Search paths in priority order (more specific to more general)
        yaml_paths = [
            os.path.join(package_path, 'config', f'{model_type}_params.yaml'),  # Type-specific config
            os.path.join(package_path, 'config', 'model_params.yaml'),  # Generic model config
            os.path.join(package_path, 'config', 'default_params.yaml')  # Default fallback config
        ]
        
        # Check for existence of each file in priority order
        for yaml_path in yaml_paths:  # For each potential path
            if os.path.exists(yaml_path) and os.access(yaml_path, os.R_OK):  # If file exists and is readable
                logging.info(f"Using parameter file: {yaml_path}")
                print(f"[INFO] Using parameter file: {yaml_path}")  # Print info message
                return yaml_path  # Return the path
        
        # Try to create a default config if none exists
        config_dir = os.path.join(package_path, 'config')
        if not os.path.exists(config_dir):
            try:
                os.makedirs(config_dir, exist_ok=True)
                logging.info(f"Created config directory: {config_dir}")
            except Exception as e:
                logging.warning(f"Failed to create config directory: {e}")
        
        # Create a default config file if allowed
        default_config_path = os.path.join(config_dir, 'default_params.yaml')
        if not os.path.exists(default_config_path):
            try:
                with open(default_config_path, 'w') as f:
                    f.write(f"# Default parameters for {model_type}\n")
                    f.write("visualizer_node:\n")
                    f.write("  ros__parameters:\n")
                    f.write("    window_width: 1280\n")
                    f.write("    window_height: 720\n")
                    f.write("    verbose_logging: false\n")
                logging.info(f"Created default parameter file: {default_config_path}")
                print(f"[INFO] Created default parameter file: {default_config_path}")
                return default_config_path
            except Exception as e:
                logging.warning(f"Failed to create default parameter file: {e}")
        
        # Log when no parameter file is found
        logging.info("No parameter file found, using launch arguments")
        print("[INFO] No parameter file found, using launch arguments")  # Print info if no file found
        return None  # Return None if no file found or created
        
    except Exception as e:  # Handle any exceptions
        logging.warning(f"Error checking for YAML parameters: {e}\n{traceback.format_exc()}")
        print(f"[WARNING] Error checking for YAML parameters: {e}", file=sys.stderr)  # Print warning
        return None  # Return None on error

def log_launch_info(context, *args, **kwargs) -> str:
    """
    Log launch configuration information.
    
    Args:
        context: Launch context
        
    Returns:
        str: Log message
    """
    # Get parameters from context with robust error handling
    try:
        model_type = LaunchConfiguration('model_type').perform(context)  # Get model_type parameter
        namespace = LaunchConfiguration('namespace').perform(context)  # Get namespace parameter
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'  # Get use_sim_time parameter as bool
        log_level = LaunchConfiguration('log_level').perform(context)  # Get log_level parameter
    except Exception as e:  # Handle any parameter access errors
        logging.error(f"Failed to access launch parameters: {e}\n{traceback.format_exc()}")
        print(f"[ERROR] Failed to access launch parameters: {e}", file=sys.stderr)  # Print error
        model_type = "unknown"  # Set default value
        namespace = ""  # Set default value
        use_sim_time = False  # Set default value
        log_level = "info"  # Set default log level
    
    # Check hardware compatibility for better info
    hw_status = check_hardware_compatibility()  # Get hardware status
    
    # Check if we have parameter file
    param_file = check_for_yaml_params(context, model_type)  # Get parameter file path if exists
    
    # Check ROS domain ID (important for multi-robot setups)
    ros_domain_id = os.environ.get('ROS_DOMAIN_ID', 'Default (0)')
    
    # Check for specific topics based on model type
    main_topic = TOPIC_MAPPING.get(model_type, f"/{model_type}")
    
    # Construct log message
    header_footer = "="*50  # Header/footer decoration
    msg = [
        header_footer,
        f"HUSKYBOT PERCEPTION - {model_type.upper()} PIPELINE",
        header_footer,
        f"Model Type    : {model_type}",
        f"Namespace     : {'/' if not namespace else '/' + namespace}",
        f"Simulation    : {use_sim_time}",
        f"Launch File   : simple_{model_type}.launch.py",
        f"Params File   : {param_file if param_file else 'Using defaults'}",
        f"Hardware      : {'Jetson AGX Orin' if hw_status['jetson'] else 'Standard PC'}",
        f"GPU Support   : {'Yes (CUDA)' if hw_status['cuda'] else 'No'}",
        f"TensorRT      : {'Enabled' if hw_status['tensorrt'] else 'Disabled'}",
        f"Velodyne      : {'VLP32-C Detected' if hw_status['velodyne'] else 'Not Detected'}",
        f"ROS Domain ID : {ros_domain_id}",
        f"Main Topic    : {main_topic}",
        f"Log Level     : {log_level}",
        header_footer
    ]
    
    # Print the message to console and log file
    for line in msg:  # For each line in message
        print(line)  # Print the line
        logging.info(line)  # Log the line
    
    return "\n".join(msg)  # Return message as single string with newlines

def validate_model_files(context, model_type: str) -> bool:
    """
    Validate that model files exist for the specified model type.
    
    Args:
        context: Launch context
        model_type: Type of model to validate
        
    Returns:
        bool: True if model files exist, False otherwise
    """
    if model_type not in MODEL_FILE_PATTERNS:  # Check if model type has defined patterns
        logging.error(f"No model file patterns defined for model type '{model_type}'")
        print(f"[ERROR] No model file patterns defined for model type '{model_type}'", file=sys.stderr)  # Print error
        return False  # Return False if no patterns defined
    
    try:
        # Determine package to look in based on model type
        package_map = {  # Map model types to packages
            'detection': 'huskybot_detection',  # Detection models in huskybot_detection
            'segmentation': 'huskybot_segmentation',  # Segmentation models in huskybot_segmentation
            'obb': 'huskybot_obb',  # OBB models in huskybot_obb
            'tracking': 'huskybot_tracking'  # Tracking models in huskybot_tracking
        }
        package = package_map.get(model_type)  # Get package for model type
        
        if not package or not check_package_exists(package, False):  # If package not found
            logging.warning(f"Package '{package}' for model type '{model_type}' not found")
            print(f"[WARNING] Package '{package}' for model type '{model_type}' not found", file=sys.stderr)  # Print warning
            
            # Try huskybot_recognition as a fallback
            if check_package_exists('huskybot_recognition', False):
                package = 'huskybot_recognition'
                logging.info(f"Using huskybot_recognition package as fallback for model files")
                print(f"[INFO] Using huskybot_recognition package as fallback for model files")
            else:
                return True  # Return True anyway to allow launch to continue (might be installing)
        
        # Get package path
        package_path = get_package_share_directory(package)  # Get package path
        
        # Check model files in several potential locations
        model_dirs = [
            os.path.join(package_path, 'models'),  # Standard models directory
            os.path.join(package_path, 'weights'),  # Alternative weights directory
            os.path.join(package_path, 'model'),   # Singular model directory
            os.path.join(package_path)             # Root package directory
        ]
        
        model_found = False
        for model_dir in model_dirs:
            if not os.path.exists(model_dir):
                continue
                
            # Check for any of the model files
            for pattern in MODEL_FILE_PATTERNS[model_type]:  # For each file pattern
                model_path = os.path.join(model_dir, pattern)  # Construct model path
                if os.path.exists(model_path):  # If model file exists
                    model_found = True
                    logging.info(f"Found model file: {model_path}")
                    print(f"[INFO] Found model file: {model_path}")  # Print info message
                    break  # Exit the pattern loop
            
            if model_found:
                break  # Exit the directory loop
                
        # Also search in the workspace models directory
        if not model_found:
            try:
                workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))
                workspace_models = os.path.join(workspace_dir, 'models')
                if os.path.exists(workspace_models):
                    for pattern in MODEL_FILE_PATTERNS[model_type]:
                        model_path = os.path.join(workspace_models, pattern)
                        if os.path.exists(model_path):
                            model_found = True
                            logging.info(f"Found model file in workspace: {model_path}")
                            print(f"[INFO] Found model file in workspace: {model_path}")
                            break
            except Exception as e:
                logging.warning(f"Error checking for models in workspace: {e}")
                
        # If we get here and no model files were found
        if not model_found:
            logging.warning(f"No model files found for '{model_type}' in any expected location")
            print(f"[WARNING] No model files found for '{model_type}'", file=sys.stderr)  # Print warning
            print(f"[WARNING] Expected one of: {', '.join(MODEL_FILE_PATTERNS[model_type])}", file=sys.stderr)  # Print expected files
            print(f"[HINT] Place model files in: {model_dirs[0]}", file=sys.stderr)
        
        return True  # Return True anyway to allow launch to continue (user might add models later)
        
    except Exception as e:  # Handle any exceptions
        logging.error(f"Error validating model files: {e}\n{traceback.format_exc()}")
        print(f"[WARNING] Error validating model files: {e}", file=sys.stderr)  # Print warning
        return True  # Return True anyway to allow launch to continue

def check_system_requirements(context) -> List[Any]:
    """
    Check system requirements for running the perception stack.
    
    Args:
        context: Launch context
        
    Returns:
        List: Actions to include in the launch description or error actions
    """
    # Get model type from context
    try:
        model_type = LaunchConfiguration('model_type').perform(context)  # Get model_type parameter
    except Exception as e:
        model_type = 'detection'  # Default to detection if parameter error
        logging.error(f"Error getting model_type, defaulting to 'detection': {e}")
    
    # System requirements by model type
    requirements = {  # Define requirements by model
        'detection': {  # Detection model requirements
            'ram': 4,  # RAM in GB
            'disk': 1,  # Disk space in GB
            'cpu_cores': 2,  # Minimum CPU cores
        },
        'segmentation': {  # Segmentation model requirements
            'ram': 8,  # RAM in GB
            'disk': 2,  # Disk space in GB
            'cpu_cores': 4,  # Minimum CPU cores
        },
        'obb': {  # OBB model requirements
            'ram': 6,  # RAM in GB
            'disk': 1.5,  # Disk space in GB
            'cpu_cores': 4,  # Minimum CPU cores
        },
        'tracking': {  # Tracking model requirements
            'ram': 8,  # RAM in GB
            'disk': 2,  # Disk space in GB
            'cpu_cores': 4,  # Minimum CPU cores
        }
    }
    
    if model_type not in requirements:  # If model type not in requirements
        return []  # Return empty list (no checks)
    
    req = requirements[model_type]  # Get requirements for model type
    warnings = []  # List to collect all warnings
    
    # Check RAM
    try:
        import psutil  # Import psutil for system info
        ram_gb = psutil.virtual_memory().total / (1024**3)  # Get total RAM in GB
        if ram_gb < req['ram']:  # If RAM less than required
            warn_msg = f"System has {ram_gb:.1f}GB RAM, {model_type} recommends at least {req['ram']}GB"
            warnings.append(warn_msg)
            logging.warning(warn_msg)
            print(f"[WARNING] {warn_msg}", file=sys.stderr)
            
        # Check CPU cores
        cpu_count = psutil.cpu_count(logical=False) or psutil.cpu_count()  # Physical cores, fallback to logical
        if cpu_count < req['cpu_cores']:
            warn_msg = f"System has {cpu_count} CPU cores, {model_type} recommends at least {req['cpu_cores']}"
            warnings.append(warn_msg)
            logging.warning(warn_msg)
            print(f"[WARNING] {warn_msg}", file=sys.stderr)
    except ImportError:  # If psutil not available
        logging.warning("psutil not available, skipping RAM and CPU checks")
        print("[WARNING] psutil not available, skipping RAM and CPU checks", file=sys.stderr)
    
    # Check disk space
    try:
        import shutil  # Import shutil for disk info
        disk_gb = shutil.disk_usage('/').free / (1024**3)  # Get free disk space in GB
        if disk_gb < req['disk']:  # If disk space less than required
            warn_msg = f"System has {disk_gb:.1f}GB free disk space, {model_type} recommends at least {req['disk']}GB"
            warnings.append(warn_msg)
            logging.warning(warn_msg)
            print(f"[WARNING] {warn_msg}", file=sys.stderr)
    except Exception as e:  # If disk check fails
        logging.warning(f"Could not check free disk space: {e}")
        print("[WARNING] Could not check free disk space", file=sys.stderr)
    
    # Return a list of LogInfo actions for the warnings (if any)
    return [LogInfo(msg=f"[WARNING] {warning}") for warning in warnings]

def create_config_directories(context) -> List[Any]:
    """
    Create necessary configuration directories if they don't exist.
    
    Args:
        context: Launch context
        
    Returns:
        List: Actions after directories are created
    """
    try:
        package_path = get_package_share_directory('huskybot_perception')
        
        # List of directories to create
        dirs_to_create = [
            os.path.join(package_path, 'config'),
            os.path.join(package_path, 'launch'),
            os.path.join(package_path, 'rviz'),
            os.path.expanduser("~/huskybot_logs"),
            os.path.expanduser("~/huskybot_screenshots")
        ]
        
        # Create directories
        for directory in dirs_to_create:
            if not os.path.exists(directory):
                try:
                    os.makedirs(directory, exist_ok=True)
                    logging.info(f"Created directory: {directory}")
                    print(f"[INFO] Created directory: {directory}")
                except Exception as e:
                    logging.warning(f"Failed to create directory {directory}: {e}")
                    print(f"[WARNING] Failed to create directory {directory}: {e}", file=sys.stderr)
    
    except Exception as e:
        logging.error(f"Error creating config directories: {e}\n{traceback.format_exc()}")
        print(f"[WARNING] Error creating config directories: {e}", file=sys.stderr)
    
    return []  # Return empty list (no actions to add)

def validate_and_include_launch(context, *args, **kwargs) -> List[Any]:
    """
    Validate and include the appropriate launch file based on model_type.
    Includes comprehensive error checking and recovery mechanisms.
    
    Args:
        context: Launch context
        
    Returns:
        List: Actions to include in the launch description
    """
    # Get parameters from context with error handling
    try:
        model_type = LaunchConfiguration('model_type').perform(context)  # Get model_type parameter
        namespace = LaunchConfiguration('namespace').perform(context)  # Get namespace parameter
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context)  # Get use_sim_time parameter
        log_level = LaunchConfiguration('log_level').perform(context)  # Get log_level parameter
        auto_detect_hardware = LaunchConfiguration('auto_detect_hardware').perform(context) == 'true'  # Get auto_detect_hardware as bool
    except Exception as e:  # Handle any parameter access errors
        logging.error(f"Failed to access launch parameters: {e}\n{traceback.format_exc()}")
        print(f"[ERROR] Failed to access launch parameters: {e}", file=sys.stderr)  # Print error
        return [EmitEvent(event=Shutdown(reason=f"Failed to access launch parameters: {e}"))]  # Emit shutdown event
    
    # Get package paths with error handling and retry
    try:
        huskybot_perception_dir = get_package_share_directory('huskybot_perception')  # Get huskybot_perception path
    except PackageNotFoundError:  # Handle package not found
        logging.error("Package 'huskybot_perception' not found. Did you source your workspace?")
        print("[FATAL] Package 'huskybot_perception' not found. Did you source your workspace?", 
              file=sys.stderr)  # Print error message
        print("[HINT] Try: source ~/huskybot/install/setup.bash", file=sys.stderr)  # Print hint
        return [EmitEvent(event=Shutdown(reason="Package 'huskybot_perception' not found"))]  # Emit shutdown event
    except Exception as e:  # Handle other exceptions
        logging.error(f"Failed to get package path: {e}\n{traceback.format_exc()}")
        print(f"[ERROR] Failed to get package path: {e}", file=sys.stderr)  # Print error
        return [EmitEvent(event=Shutdown(reason=f"Failed to get package path: {e}"))]  # Emit shutdown event
    
    # Check if model_type is valid
    if not check_model_type(context, model_type):  # Validate model type
        return [EmitEvent(event=Shutdown(reason=f"Invalid model_type: {model_type}"))]  # Emit shutdown event if invalid
    
    # Validate model files
    if not validate_model_files(context, model_type):  # Validate model files
        logging.warning(f"Proceeding without validated model files for '{model_type}'")
        print(f"[WARNING] Proceeding without validated model files for '{model_type}'", file=sys.stderr)  # Print warning
    
    # Construct launch file path
    launch_file = os.path.join(huskybot_perception_dir, 'launch', f'simple_{model_type}.launch.py')  # Construct path
    
    # Validate launch file exists with retries
    launch_file_valid = False
    for attempt in range(MAX_RETRIES):
        if validate_launch_file(context, launch_file):  # Validate launch file
            launch_file_valid = True
            break
        else:
            # If not the last attempt, wait and retry
            if attempt < MAX_RETRIES - 1:
                print(f"[WARNING] Retrying launch file validation ({attempt+1}/{MAX_RETRIES})...", file=sys.stderr)
                time.sleep(RETRY_DELAY)
    
    # If launch file validation failed after all retries
    if not launch_file_valid:
        print("[ERROR] Launch file not found after retries. Creating a fallback launch...", file=sys.stderr)
        logging.error(f"Could not find launch file for {model_type} after {MAX_RETRIES} retries")
        
        # Create a fallback visualization node directly
        fallback_actions = [  # Define fallback actions
            LogInfo(msg=f"[ERROR] Could not find launch file for {model_type}. Launching fallback visualizer."),  # Log error
            
            # Launch a visualizer node directly as fallback
            Node(
                package='huskybot_perception',  # Use perception package
                executable='visualizer_node',  # Use visualizer node
                name='fallback_visualizer',  # Name the node
                namespace=namespace if namespace else '',  # Use provided namespace or default
                parameters=[{  # Set parameters
                    'use_sim_time': use_sim_time == 'true',  # Convert to bool
                    'log_level': log_level,  # Set log level
                    'window_width': 1280,  # Default window width
                    'window_height': 720,  # Default window height
                    'enable_display': True,  # Enable display
                    'topics': [f'/{model_type}'],  # Set topic to subscribe based on model_type
                    'verbose_logging': True,  # Enable verbose for debugging
                }],
                remappings=[],  # No remappings
                output='screen'  # Show output on screen
            )
        ]
        return fallback_actions  # Return fallback actions
    
    # Check for parameter file
    param_file = check_for_yaml_params(context, model_type)  # Get parameter file path
    
    # Determine hardware configuration for optimization parameters
    hw_status = {}
    if auto_detect_hardware:
        hw_status = check_hardware_compatibility()
    
    # Construct launch arguments
    launch_args = {  # Define launch arguments
        'namespace': namespace,  # Pass namespace parameter
        'use_sim_time': use_sim_time,  # Pass use_sim_time parameter
        'log_level': log_level,  # Pass log_level parameter
        'auto_detect_hardware': str(auto_detect_hardware).lower()  # Pass auto_detect_hardware parameter
    }
    
    # Add hardware-specific parameters if detected
    if hw_status.get('jetson', False):
        launch_args['jetson_device'] = 'true'
    if hw_status.get('tensorrt', False):
        launch_args['use_tensorrt'] = 'true'
    
    # Add parameter file if found
    if param_file:  # If parameter file found
        launch_args['params_file'] = param_file  # Add to launch arguments
    
    # Create successful launch with event handler for errors
    try:
        launch_actions = [  # Define launch actions
            # Include the appropriate launch file
            IncludeLaunchDescription(  # Include launch description
                PythonLaunchDescriptionSource([launch_file]),  # Source is the validated launch file
                launch_arguments=launch_args.items()  # Pass all the launch arguments
            )
        ]
    except Exception as e:
        logging.error(f"Error including launch file: {e}\n{traceback.format_exc()}")
        print(f"[ERROR] Error including launch file: {e}", file=sys.stderr)
        
        # Fallback to direct node launch if include fails
        return [
            LogError(msg=f"[ERROR] Failed to include launch file: {e}"),
            Node(
                package='huskybot_perception',
                executable='visualizer_node',
                name='emergency_visualizer',
                parameters=[{'use_sim_time': use_sim_time == 'true'}],
                output='screen'
            )
        ]
    
    # Add event handler for errors
    launch_actions.append(  # Add to launch actions
        RegisterEventHandler(  # Register event handler
            OnProcessExit(  # For process exit event
                target_action=launch_actions[0],  # Target the included launch
                on_exit=[  # On exit actions
                    LogInfo(msg=f"Launch file {os.path.basename(launch_file)} exited")  # Log info message
                ]
            )
        )
    )
    
    return launch_actions  # Return the launch actions

# ===================== MAIN LAUNCH DESCRIPTION =====================

def generate_launch_description() -> LaunchDescription:
    """
    Generate the main launch description for huskybot_perception.
    
    Provides a flexible entry point for different perception models:
    - detection: YOLOv12 object detection
    - segmentation: YOLOv11 semantic segmentation
    - obb: YOLOv12 oriented bounding box detection
    - tracking: YOLOv12 object tracking
    
    Handles parameter validation, error recovery, and hardware adaptation.
    
    Returns:
        LaunchDescription: The complete launch description
    """
    try:
        # Declare launch arguments with documentation and default values
        model_type_arg = DeclareLaunchArgument(
            'model_type',  # Parameter name
            default_value='detection',  # Default to detection model
            description='Type of YOLOv12 model to use: detection (yolo12x.engine), segmentation (yolo11x-seg.engine), ' +
                        'obb (yolo12x-obb.engine), or tracking (yolo12x-track.engine)'  # Detailed description
        )
        
        namespace_arg = DeclareLaunchArgument(
            'namespace',  # Parameter name
            default_value='',  # Default empty namespace
            description='Namespace for nodes (required for multi-robot setups)'  # Description
        )
        
        use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',  # Parameter name
            default_value='false',  # Default to real robot mode
            description='Use simulation time (true for Gazebo simulation, false for real robot)'  # Description
        )
        
        log_level_arg = DeclareLaunchArgument(
            'log_level',  # Parameter name
            default_value='info',  # Default to info level
            description='Logging level: debug, info, warn, error, fatal'  # Description
        )
        
        auto_detect_hardware_arg = DeclareLaunchArgument(
            'auto_detect_hardware',  # Parameter name
            default_value='true',  # Default to auto-detect
            description='Automatically detect and optimize for hardware (Jetson, GPU, etc.)'  # Description
        )
        
        params_file_arg = DeclareLaunchArgument(
            'params_file',  # Parameter name
            default_value='',  # Default to empty (auto-detect)
            description='Path to YAML parameter file (empty for auto-detect)'  # Description
        )
        
        # Create directories for logs, configs, etc.
        create_dirs = OpaqueFunction(function=create_config_directories)
        
        # Create system check action
        system_check = OpaqueFunction(function=check_system_requirements)  # Create system check action
        
        # Create LogInfo action for debugging
        log_info = OpaqueFunction(function=log_launch_info)  # Create log info action
        
        # Validate and include the appropriate launch file
        include_launch = OpaqueFunction(function=validate_and_include_launch)  # Create validate and include action
        
        # Combine all actions into the final launch description
        return LaunchDescription([
            # Launch arguments
            model_type_arg,      # Define model type argument (detection, segmentation, etc.)
            namespace_arg,       # Define namespace argument (for multi-robot)
            use_sim_time_arg,    # Define use_sim_time argument (true for simulation)
            log_level_arg,       # Define log level argument (debug, info, etc.)
            auto_detect_hardware_arg,  # Define auto-detect hardware argument
            params_file_arg,     # Define params file argument
            
            # Create necessary directories
            create_dirs,         # Create config and log directories
            
            # System checks
            system_check,        # Check system requirements
            
            # Log launch configuration for debugging
            log_info,            # Log launch configuration
            
            # Include the appropriate launch file based on model_type
            include_launch,      # Include the appropriate launch file
            
            # Register shutdown handler
            RegisterEventHandler(  # Register event handler
                OnShutdown(  # For shutdown event
                    on_shutdown=[  # Actions on shutdown
                        LogInfo(msg="Shutting down huskybot_perception launch")  # Log shutdown message
                    ]
                )
            ),
        ])
    except Exception as e:
        # Last resort error handling at the launch description level
        logging.critical(f"Fatal error generating launch description: {e}\n{traceback.format_exc()}")
        print(f"[FATAL] Error generating launch description: {e}", file=sys.stderr)
        
        # Return minimal launch description with error message
        return LaunchDescription([
            LogError(msg=f"[FATAL] Failed to generate launch description: {e}"),
            ExecuteProcess(
                cmd=["echo", f"FATAL ERROR: {e}"],
                output="screen"
            )
        ])