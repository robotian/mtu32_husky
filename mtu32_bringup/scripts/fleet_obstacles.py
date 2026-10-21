#!/usr/bin/env python3
"""The other robots, as obstacles for this robot's global costmap (its fleet_layer, config/nav2_robots.yaml).

The global costmap plans on the farm map only (a scan there marks the lavender beside the robot and the planner
finds no way into the lanes), so on its own the planner drives straight at another robot and the controller stops
in front of it for good. Every robot's Nav2 publishes its footprint in the map frame
(/<ns>/global_costmap/published_footprint, 1 Hz); this node follows the other robots' and publishes:

  fleet_obstacles/cloud  PointCloud2 in map, points every 5 cm filling each footprint (marking)
  fleet_obstacles/scan   LaserScan in base_link, every ray at range_max (clearing only)

The obstacle layer clears before it marks, so each update clears around the robot and marks where the others are
now: a robot that moved leaves no trace. A footprint not heard for `timeout` s is dropped. Only robots in the same
ROS graph are seen (none: the layer stays clear).
"""
import math
import re
import struct

import rclpy
from geometry_msgs.msg import PolygonStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

FOOTPRINT = re.compile(r'^/([^/]+)/global_costmap/published_footprint$')
STEP = 0.05  # m between cloud points, under the costmap resolution so every covered cell is marked


def inside(x, y, poly):
    """Point in polygon (even-odd rule)."""
    hit = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            hit = not hit
    return hit


def fill(poly):
    """Points STEP apart covering the polygon, its corners included."""
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    nx, ny = int((max(xs) - min(xs)) / STEP) + 1, int((max(ys) - min(ys)) / STEP) + 1
    grid = [(min(xs) + i * STEP, min(ys) + j * STEP) for i in range(nx + 1) for j in range(ny + 1)]
    return [p for p in grid if inside(*p, poly)] + list(poly)


class FleetObstacles(Node):
    def __init__(self):
        super().__init__('fleet_obstacles')
        self.rate = self.declare_parameter('rate', 2.0).value
        self.timeout = self.declare_parameter('timeout', 3.0).value
        self.range = self.declare_parameter('clear_range', 25.0).value  # m cleared around the robot
        self.rays = self.declare_parameter('rays', 3600).value  # 0.1 deg: gaps at 25 m stay under a 6 cm cell
        self.own = self.get_namespace().strip('/')
        self.footprints = {}  # namespace -> (received at, [(x, y), ...])
        self.subs = {}
        self.cloud_pub = self.create_publisher(PointCloud2, 'fleet_obstacles/cloud', 1)
        self.scan_pub = self.create_publisher(LaserScan, 'fleet_obstacles/scan', 1)
        self.create_timer(2.0, self.discover)
        self.create_timer(1.0 / self.rate, self.publish)
        self.discover()

    def discover(self):
        for topic, _ in self.get_topic_names_and_types():
            m = FOOTPRINT.match(topic)
            if m and m.group(1) != self.own and m.group(1) not in self.subs:
                ns = m.group(1)
                self.subs[ns] = self.create_subscription(PolygonStamped, topic, lambda msg, ns=ns: self.on_footprint(ns, msg), 1)
                self.get_logger().info(f'following {ns}')

    def on_footprint(self, ns, msg):
        if msg.header.frame_id.lstrip('/') == 'map' and len(msg.polygon.points) >= 3:
            self.footprints[ns] = (self.get_clock().now(), [(p.x, p.y) for p in msg.polygon.points])

    def publish(self):
        now = self.get_clock().now()
        stale = now - Duration(seconds=self.timeout)
        points = [p for t, poly in self.footprints.values() if t > stale for p in fill(poly)]

        cloud = PointCloud2()
        cloud.header.stamp, cloud.header.frame_id = now.to_msg(), 'map'
        cloud.height, cloud.width = 1, len(points)
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
        cloud.point_step, cloud.row_step, cloud.is_dense = 12, 12 * len(points), True
        cloud.data = b''.join(struct.pack('<fff', x, y, 0.1) for x, y in points)  # z inside the layer's height band
        self.cloud_pub.publish(cloud)

        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = now.to_msg(), 'base_link'
        scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / self.rays
        scan.angle_max = scan.angle_min + (self.rays - 1) * scan.angle_increment
        scan.range_min, scan.range_max = 0.0, self.range + 1.0
        scan.ranges = [self.range] * self.rays
        self.scan_pub.publish(scan)


def main():
    rclpy.init()
    node = FleetObstacles()
    # Not rclpy.spin(): under use_sim_time it never returns after SIGTERM (colcon_ws/CLAUDE.md).
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
