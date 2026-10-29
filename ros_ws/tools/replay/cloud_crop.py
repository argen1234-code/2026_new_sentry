#!/usr/bin/env python3
"""Crop replayed terrain clouds to what the *simulated* robot could see.

A bag records what the real robot saw from where it was. Replaying that cloud
verbatim gives the local costmap a god's-eye view: obstacles stay visible at
their recorded positions even after the simulated robot moves elsewhere, so the
local planner looks better than it is. Cropping to a radius around the simulated
pose restores "only what is nearby is known" without re-running terrain_analysis.

Usage: play the bag with remaps, e.g.
    ros2 bag play <bag> --clock --topics /terrain_map /terrain_map_ext \\
        --ros-args -r /terrain_map:=/terrain_map_raw \\
                   -r /terrain_map_ext:=/terrain_map_ext_raw
"""

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class CloudCrop(Node):
    def __init__(self):
        super().__init__("cloud_crop")
        d = self.declare_parameter
        d("local_input", "terrain_map_raw")
        d("local_output", "terrain_map")
        d("local_radius", 6.0)
        d("ext_input", "terrain_map_ext_raw")
        d("ext_output", "terrain_map_ext")
        d("ext_radius", 12.0)
        d("odom_topic", "odometry")

        cfg = [
            (self.get_parameter("local_input").value,
             self.get_parameter("local_output").value,
             float(self.get_parameter("local_radius").value)),
            (self.get_parameter("ext_input").value,
             self.get_parameter("ext_output").value,
             float(self.get_parameter("ext_radius").value)),
        ]

        self.pose = None
        self.kept = 0
        self.dropped = 0
        for src, dst, radius in cfg:
            pub = self.create_publisher(PointCloud2, dst, qos_profile_sensor_data)
            self.create_subscription(
                PointCloud2, src,
                lambda msg, p=pub, r=radius, s=src: self.crop(msg, p, r, s),
                qos_profile_sensor_data)
            self.get_logger().info(f"{src} -> {dst} radius {radius} m")
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value,
                                 self.odom_cb, 10)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def crop(self, msg, pub, radius, name):
        if self.pose is None:
            return
        if msg.width * msg.height == 0:
            pub.publish(msg)
            return

        x_off = y_off = None
        for f in msg.fields:
            if f.name == "x":
                x_off = f.offset
            elif f.name == "y":
                y_off = f.offset
        if x_off is None or y_off is None:
            pub.publish(msg)
            return

        n = msg.width * msg.height
        step = msg.point_step
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < n * step:
            return
        x = np.ndarray(shape=(n,), dtype=np.float32, buffer=msg.data, offset=x_off, strides=(step,))
        y = np.ndarray(shape=(n,), dtype=np.float32, buffer=msg.data, offset=y_off, strides=(step,))
        keep = np.hypot(x - self.pose[0], y - self.pose[1]) <= radius
        self.kept += int(keep.sum())
        self.dropped += int((~keep).sum())

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(keep.sum())
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = step
        out.row_step = out.width * step
        out.is_dense = msg.is_dense
        out.data = raw.reshape(n, step)[keep].tobytes()
        pub.publish(out)


def main():
    rclpy.init()
    node = CloudCrop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"cloud_crop: kept {node.kept} / dropped {node.dropped} points")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
