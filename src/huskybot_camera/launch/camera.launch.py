from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # ===================== ARGUMEN LAUNCH =====================
    args = []
    # Mapping device ke topic sesuai urutan fisik dan pipeline Huskybot
    camera_remap = [
        # (device, topic)
        ('csi://0', '/camera_front/image_raw'),
        ('csi://1', '/camera_front_left/image_raw'),
        ('csi://2', '/camera_left/image_raw'),
        ('csi://3', '/camera_rear/image_raw'),
        ('csi://4', '/camera_rear_right/image_raw'),
        ('csi://5', '/camera_right/image_raw'),
    ]
    for i, (dev, topic) in enumerate(camera_remap, start=1):
        args.append(DeclareLaunchArgument(
            f'camera{i}_device',
            default_value=dev,
            description=f'Device kamera {i} (misal: {dev})'
        ))
        args.append(DeclareLaunchArgument(
            f'camera{i}_topic',
            default_value=topic,
            description=f'Topic output kamera {i} (misal: {topic})'
        ))
        args.append(DeclareLaunchArgument(
            f'camera{i}_width',
            default_value='1920',
            description=f'Resolution width kamera {i}'
        ))
        args.append(DeclareLaunchArgument(
            f'camera{i}_height',
            default_value='1080',
            description=f'Resolution height kamera {i}'
        ))
        args.append(DeclareLaunchArgument(
            f'camera{i}_framerate',
            default_value='30.0', 
            description=f'Framerate kamera {i}'
        ))

    # ===================== NODE VIDEO_SOURCE UNTUK 6 KAMERA =====================
    nodes = []
    for i in range(1, 7):
        nodes.append(
            Node(
                package='ros_deep_learning',
                executable='video_source',
                name=f'video_source_{i}',
                output='screen',
                parameters=[{
                    'resource': LaunchConfiguration(f'camera{i}_device'),
                    'width': LaunchConfiguration(f'camera{i}_width'),
                    'height': LaunchConfiguration(f'camera{i}_height'),
                    'framerate': LaunchConfiguration(f'camera{i}_framerate'),
                    'codec': 'unknown',
                    'loop': 0,
                    'latency': 2000,
                }],
                remappings=[
                    ('/video_source/raw', LaunchConfiguration(f'camera{i}_topic'))
                ]
            )
        )

    return LaunchDescription(args + nodes)