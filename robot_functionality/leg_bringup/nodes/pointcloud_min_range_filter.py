#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class PointCloudMinRangeFilter(Node):
    def __init__(self):
        super().__init__("pointcloud_min_range_filter")

        self.declare_parameter("input_topic", "/cloud_registered")
        self.declare_parameter("output_topic", "/cloud_registered_min_range_filtered")
        self.declare_parameter("odom_topic", "/aft_mapped_to_init")
        self.declare_parameter("min_range", 1.0)
        self.declare_parameter("use_odom_origin", True)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        self.min_range = float(self.get_parameter("min_range").value)
        self.use_odom_origin = bool(self.get_parameter("use_odom_origin").value)
        self.origin = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.has_odom = False

        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.pub = self.create_publisher(PointCloud2, output_topic, qos)
        self.sub = self.create_subscription(PointCloud2, input_topic, self.cloud_callback, qos)

        if self.use_odom_origin:
            self.odom_sub = self.create_subscription(
                Odometry, odom_topic, self.odom_callback, QoSProfile(depth=20)
            )
        else:
            self.odom_sub = None

        self.get_logger().info(
            f"Filtering {input_topic} -> {output_topic}, min_range={self.min_range:.2f} m, "
            f"use_odom_origin={self.use_odom_origin}, odom_topic={odom_topic}"
        )

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.origin = np.array([p.x, p.y, p.z], dtype=np.float32)
        self.has_odom = True

    def cloud_callback(self, msg: PointCloud2):
        if not msg.fields:
            self.pub.publish(msg)
            return

        points = pc2.read_points(msg, skip_nans=True)
        if points.size == 0:
            self.pub.publish(msg)
            return

        names = points.dtype.names
        if not names or not {"x", "y", "z"}.issubset(names):
            self.get_logger().warn("Input PointCloud2 has no x/y/z fields; publishing unchanged.")
            self.pub.publish(msg)
            return

        origin = self.origin if (self.use_odom_origin and self.has_odom) else np.zeros(3, dtype=np.float32)
        min_range_sq = self.min_range * self.min_range
        dx = points["x"] - origin[0]
        dy = points["y"] - origin[1]
        dz = points["z"] - origin[2]
        keep = (dx * dx + dy * dy + dz * dz) >= min_range_sq
        filtered_points = points[keep]

        out = pc2.create_cloud(msg.header, msg.fields, filtered_points)
        out.header = msg.header
        self.pub.publish(out)

        if self.get_clock().now().nanoseconds % int(5e9) < int(1e8):
            removed = int(points.size - filtered_points.size)
            if removed > 0:
                self.get_logger().debug(
                    f"Removed {removed}/{points.size} points within {self.min_range:.2f} m"
                )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMinRangeFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
