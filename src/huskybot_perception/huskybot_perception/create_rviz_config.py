#!/usr/bin/env python3

import os
import yaml

def create_rviz_config():
    """Create RViz2 configuration for 3D object visualization"""
    
    rviz_config = {
        'Panels': [
            {
                'Class': 'rviz_common/Displays',
                'Help Height': 78,
                'Name': 'Displays',
                'Property Tree Widget': {
                    'Expanded': [
                        '/Global Options1',
                        '/Status1',
                        '/PointCloud21',
                        '/TF1'
                    ],
                    'Splitter Ratio': 0.5
                },
                'Tree Height': 787
            },
            {
                'Class': 'rviz_common/Selection',
                'Name': 'Selection'
            },
            {
                'Class': 'rviz_common/Tool Properties',
                'Expanded': [
                    '/2D Goal Pose1',
                    '/2D Nav Goal1',
                    '/Publish Point1'
                ],
                'Name': 'Tool Properties',
                'Splitter Ratio': 0.5885416865348816
            },
            {
                'Class': 'rviz_common/Views',
                'Expanded': [
                    '/Current View1'
                ],
                'Name': 'Views',
                'Splitter Ratio': 0.5
            }
        ],
        'Preferences': {
            'PromptSaveOnExit': True
        },
        'Toolbars': 'toolbars',
        'Visualization Manager': {
            'Class': '',
            'Displays': [
                {
                    'Alpha': 0.5,
                    'Cell Size': 1.0,
                    'Class': 'rviz_default_plugins/Grid',
                    'Color': '50; 50; 50',
                    'Enabled': True,
                    'Line Style': {
                        'Line Width': 0.029999999329447746,
                        'Value': 'Lines'
                    },
                    'Name': 'Grid',
                    'Normal Cell Count': 0,
                    'Offset': {
                        'X': 0.0,
                        'Y': 0.0,
                        'Z': 0.0
                    },
                    'Plane': 'XY',
                    'Plane Cell Count': 20,
                    'Reference Frame': '<Fixed Frame>',
                    'Value': True
                },
                {
                    'Class': 'rviz_default_plugins/TF',
                    'Enabled': True,
                    'Filter (blacklist)': '',
                    'Filter (whitelist)': '',
                    'Frame Timeout': 15.0,
                    'Frames': {
                        'All Enabled': True,
                        'base_link': {
                            'Value': True
                        },
                        'base_footprint': {
                            'Value': True
                        }
                    },
                    'Marker Scale': 1.0,
                    'Name': 'TF',
                    'Show Arrows': True,
                    'Show Axes': True,
                    'Show Names': True,
                    'Tree': {
                        'base_link': {
                            'base_footprint': {}
                        }
                    },
                    'Update Interval': 0.0,
                    'Value': True
                },
                {
                    'Alpha': 1.0,
                    'Autocompute Intensity Bounds': True,
                    'Autocompute Value Bounds': {
                        'Max Value': 10.0,
                        'Min Value': -10.0,
                        'Value': True
                    },
                    'Axis': 'Z',
                    'Channel Name': 'intensity',
                    'Class': 'rviz_default_plugins/PointCloud2',
                    'Color': '255; 255; 255',
                    'Color Transformer': 'Intensity',
                    'Decay Time': 0.0,
                    'Enabled': True,
                    'Invert Rainbow': False,
                    'Max Color': '255; 255; 255',
                    'Max Intensity': 100.0,
                    'Min Color': '0; 0; 0',
                    'Min Intensity': 0.0,
                    'Name': '3D Objects',
                    'Position Transformer': 'XYZ',
                    'Selectable': True,
                    'Size (Pixels)': 5.0,
                    'Size (m)': 0.1,
                    'Style': 'Spheres',
                    'Topic': {
                        'Depth': 5,
                        'Durability Policy': 'Volatile',
                        'Filter size': 10,
                        'History Policy': 'Keep Last',
                        'Reliability Policy': 'Reliable',
                        'Value': '/objects_3d_pointcloud'
                    },
                    'Use Fixed Frame': True,
                    'Use rainbow': True,
                    'Value': True
                },
                {
                    'Class': 'rviz_default_plugins/Axes',
                    'Enabled': True,
                    'Length': 1.0,
                    'Name': 'Robot Base',
                    'Reference Frame': 'base_link',
                    'Radius': 0.1,
                    'Value': True
                }
            ],
            'Enabled': True,
            'Global Options': {
                'Background Color': '48; 48; 48',
                'Default Light': True,
                'Fixed Frame': 'base_link',
                'Frame Rate': 30
            },
            'Name': 'root',
            'Tools': [
                {
                    'Class': 'rviz_default_plugins/Interact',
                    'Hide Inactive Objects': True
                },
                {
                    'Class': 'rviz_default_plugins/MoveCamera'
                },
                {
                    'Class': 'rviz_default_plugins/Select'
                },
                {
                    'Class': 'rviz_default_plugins/FocusCamera'
                },
                {
                    'Class': 'rviz_default_plugins/Measure'
                },
                {
                    'Class': 'rviz_default_plugins/SetInitialPose',
                    'Covariance x': 0.25,
                    'Covariance y': 0.25,
                    'Covariance yaw': 0.06853891909122467,
                    'Topic': {
                        'Depth': 5,
                        'Durability Policy': 'Volatile',
                        'History Policy': 'Keep Last',
                        'Reliability Policy': 'Reliable',
                        'Value': '/initialpose'
                    }
                },
                {
                    'Class': 'rviz_default_plugins/SetGoal',
                    'Topic': {
                        'Depth': 5,
                        'Durability Policy': 'Volatile',
                        'History Policy': 'Keep Last',
                        'Reliability Policy': 'Reliable',
                        'Value': '/goal_pose'
                    }
                },
                {
                    'Class': 'rviz_default_plugins/PublishPoint',
                    'Single click': True,
                    'Topic': {
                        'Depth': 5,
                        'Durability Policy': 'Volatile',
                        'History Policy': 'Keep Last',
                        'Reliability Policy': 'Reliable',
                        'Value': '/clicked_point'
                    }
                }
            ],
            'Transformation': {
                'Current': {
                    'Class': 'rviz_default_plugins/TF'
                }
            },
            'Value': True,
            'Views': {
                'Current': {
                    'Class': 'rviz_default_plugins/Orbit',
                    'Distance': 10.0,
                    'Enable Stereo Rendering': {
                        'Stereo Eye Separation': 0.06000000238418579,
                        'Stereo Focal Distance': 1.0,
                        'Swap Stereo Eyes': False,
                        'Value': False
                    },
                    'Focal Point': {
                        'X': 0.0,
                        'Y': 0.0,
                        'Z': 0.0
                    },
                    'Focal Shape Fixed Size': True,
                    'Focal Shape Size': 0.05000000074505806,
                    'Invert Z Axis': False,
                    'Name': 'Current View',
                    'Near Clip Distance': 0.009999999776482582,
                    'Pitch': 0.4853981733322144,
                    'Target Frame': '<Fixed Frame>',
                    'Value': 'Orbit (rviz_default_plugins)',
                    'Yaw': 0.7853981852531433
                },
                'Saved': {}
            }
        }
    }
    
    # Create directory
    config_dir = '/home/kmp-orin/jezzy/huskybot/install/huskybot_perception/share/huskybot_perception/rviz'
    os.makedirs(config_dir, exist_ok=True)
    
    # Write config file
    config_path = os.path.join(config_dir, 'huskybot_3d.rviz')
    with open(config_path, 'w') as f:
        yaml.dump(rviz_config, f, default_flow_style=False, indent=2)
    
    print(f"✅ RViz2 config created: {config_path}")

if __name__ == '__main__':
    create_rviz_config()