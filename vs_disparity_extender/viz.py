import math
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

LIFETIME = Duration(sec=0, nanosec=500_000_000)


def _point(distance, angle):
    return Point(
        x=float(distance * math.cos(angle)),
        y=float(distance * math.sin(angle)),
        z=0.0,
    )


def scan_msg(scan, ranges, angles):
    out = LaserScan()
    out.header = scan.header
    out.angle_min = float(angles[0])
    out.angle_max = float(angles[-1])
    out.angle_increment = scan.angle_increment
    out.range_min = scan.range_min
    out.range_max = scan.range_max
    out.ranges = [float(r) for r in ranges]
    return out


def closest_marker(scan, distance, angle):
    marker = Marker()
    marker.header = scan.header
    marker.ns = 'closest'
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position = _point(distance, angle)
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.20
    marker.scale.y = 0.20
    marker.scale.z = 0.20
    marker.color = ColorRGBA(r=0.9, g=0.1, b=0.1, a=1.0)
    marker.lifetime = LIFETIME
    return marker


def target_marker(scan, distance, angle):
    marker = Marker()
    marker.header = scan.header
    marker.ns = 'target'
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.06
    marker.scale.y = 0.14
    marker.scale.z = 0.18
    marker.color = ColorRGBA(r=0.1, g=0.85, b=0.2, a=1.0)
    marker.lifetime = LIFETIME
    marker.points = [Point(), _point(distance, angle)]
    return marker
