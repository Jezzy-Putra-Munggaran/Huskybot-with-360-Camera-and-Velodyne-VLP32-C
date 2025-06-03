from visualization_msgs.msg import Marker

def create_object_marker(obj, idx, frame_id="base_link"):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = obj.header.stamp
    marker.ns = "objects3d"
    marker.id = idx
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = obj.center[0]
    marker.pose.position.y = obj.center[1]
    marker.pose.position.z = obj.center[2] + 0.5
    marker.scale.z = 0.4
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.text = f"[{obj.label}]\nJarak: {sum([x**2 for x in obj.center])**0.5:.2f}m\nPosisi: ({obj.center[0]:.2f}, {obj.center[1]:.2f}, {obj.center[2]:.2f})"
    marker.lifetime.sec = 1
    return marker