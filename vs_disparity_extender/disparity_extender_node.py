import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

from vs_disparity_extender import viz


class DisparityExtenderNode(Node):
    def __init__(self):
        super().__init__('disparity_extender_node')

        self.declare_parameter('speed', 2.0)
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('bubble_radius', 0.25)
        self.declare_parameter('clearance', 0.22)
        self.declare_parameter('far_ratio', 0.95)
        self.declare_parameter('disparity_threshold', 0.1)
        self.declare_parameter('max_steer_rad', 0.34)
        self.declare_parameter('fov_deg', 180.0)
        self.declare_parameter('publish_debug_scans', True)

        self.speed = float(self.get_parameter('speed').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.bubble_radius = float(self.get_parameter('bubble_radius').value)
        self.clearance = float(self.get_parameter('clearance').value)
        self.far_ratio = float(self.get_parameter('far_ratio').value)
        self.disparity = float(self.get_parameter('disparity_threshold').value)
        self.max_steer = float(self.get_parameter('max_steer_rad').value)
        self.fov = math.radians(float(self.get_parameter('fov_deg').value))
        self.publish_debug_scans = bool(self.get_parameter('publish_debug_scans').value)

        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        self.proc_pub = self.create_publisher(LaserScan, 'scan_proc', 10)
        self.extended_pub = self.create_publisher(LaserScan, 'scan_extended', 10)
        self.free_pub = self.create_publisher(LaserScan, 'scan_free', 10)
        self.closest_pub = self.create_publisher(Marker, 'closest', 10)
        self.target_pub = self.create_publisher(Marker, 'target', 10)

    def read_scan(self, scan):
        ranges = np.array(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        return ranges, angles

    def preprocess_lidar(self, ranges, angles):
        front = np.abs(angles) <= (self.fov / 2.0)
        ranges = ranges[front]
        angles = angles[front]
        ranges = np.nan_to_num(ranges, nan=self.max_range, posinf=self.max_range)
        ranges = np.minimum(ranges, self.max_range)
        return ranges, angles

    def extend_disparities(self, ranges, angle_increment):
        extended = ranges.copy()
        n = len(ranges)

        for i in range(n - 1):
            jump = ranges[i + 1] - ranges[i]
            if abs(jump) <= self.disparity:
                continue

            closer_dist = max(min(ranges[i], ranges[i + 1]), 0.05)
            width = math.ceil(self.clearance / (closer_dist * angle_increment))

            if jump > 0:
                start = i + 1
                end = min(i + 1 + width, n)
            else:
                start = max(i - width + 1, 0)
                end = i + 1
            extended[start:end] = np.minimum(extended[start:end], closer_dist)

        return extended

    def apply_safety_bubble(self, ranges, angle_increment):
        closest = int(np.argmin(ranges))
        dist = max(ranges[closest], 0.05)
        width = math.ceil(self.bubble_radius / (dist * angle_increment))

        free = ranges.copy()
        start = max(closest - width, 0)
        end = min(closest + width + 1, len(ranges))
        free[start:end] = 0.0
        return free, closest

    def find_best_point(self, free):
        nonzero = np.where(free > 0.0)[0]
        if nonzero.size == 0:
            return len(free) // 2

        breaks = np.where(np.diff(nonzero) != 1)[0] + 1
        gaps = np.split(nonzero, breaks)
        max_gap = max(gaps, key=len)

        depth = free[max_gap]
        threshold = self.far_ratio * depth.max()
        far_beams = max_gap[depth >= threshold]

        far_breaks = np.where(np.diff(far_beams) != 1)[0] + 1
        widest_far = max(np.split(far_beams, far_breaks), key=len)
        return widest_far[len(widest_far) // 2]

    def drive(self, target_angle):
        steer = float(np.clip(target_angle, -self.max_steer, self.max_steer))

        if abs(steer) > 0.2:
            speed = self.speed * 0.5
        elif abs(steer) > 0.1:
            speed = self.speed * 0.75
        else:
            speed = self.speed

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = steer
        msg.drive.speed = speed
        self.drive_pub.publish(msg)
        return steer, speed

    def lidar_callback(self, scan):
        ranges, angles = self.read_scan(scan)
        ranges, angles = self.preprocess_lidar(ranges, angles)
        extended = self.extend_disparities(ranges, scan.angle_increment)
        free, closest = self.apply_safety_bubble(extended, scan.angle_increment)
        best_idx = self.find_best_point(free)
        self.drive(angles[best_idx])

        if self.publish_debug_scans:
            self.proc_pub.publish(viz.scan_msg(scan, ranges, angles))
            self.extended_pub.publish(viz.scan_msg(scan, extended, angles))
            self.free_pub.publish(viz.scan_msg(scan, free, angles))
            self.closest_pub.publish(viz.closest_marker(scan, extended[closest], angles[closest]))
            self.target_pub.publish(viz.target_marker(scan, max(free[best_idx], 1.0), angles[best_idx]))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DisparityExtenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.drive(0.0)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
