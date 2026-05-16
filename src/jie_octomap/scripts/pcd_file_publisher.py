#!/usr/bin/env python3
# pcd_file_publisher.py  —  ROS 1 port of the original ROS 2 implementation

import struct

import numpy as np
import open3d as o3d
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def rgb_to_float(r: int, g: int, b: int) -> float:
    rgb_uint32 = (int(r) << 16) | (int(g) << 8) | int(b)
    return struct.unpack("f", struct.pack("I", rgb_uint32))[0]


def load_pcd(pcd_path: str, frame_id: str) -> PointCloud2:
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points, dtype=np.float32)
    if points.size == 0:
        raise RuntimeError(f"PCD is empty: {pcd_path}")

    header = Header()
    header.frame_id = frame_id

    if pcd.has_colors():
        colors = np.asarray(pcd.colors, dtype=np.float32)
        cloud_data = []
        for i in range(points.shape[0]):
            r = int(np.clip(colors[i, 0] * 255.0, 0, 255))
            g = int(np.clip(colors[i, 1] * 255.0, 0, 255))
            b = int(np.clip(colors[i, 2] * 255.0, 0, 255))
            cloud_data.append((float(points[i, 0]), float(points[i, 1]),
                               float(points[i, 2]), rgb_to_float(r, g, b)))
        fields = [
            PointField(name="x",   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        return pc2.create_cloud(header, fields, cloud_data)

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    return pc2.create_cloud(header, fields, points.tolist())


def main() -> None:
    rospy.init_node("pcd_file_publisher")

    pcd_path = rospy.get_param("~pcd_path",    "")
    topic = rospy.get_param("~topic",       "/pcd_points")
    frame_id = rospy.get_param("~frame_id",    "map")
    publish_hz = float(rospy.get_param("~publish_hz", 1.0))

    if not pcd_path:
        rospy.logfatal("pcd_path parameter is empty")
        return

    pub = rospy.Publisher(topic, PointCloud2, queue_size=10, latch=True)

    try:
        cloud_msg = load_pcd(pcd_path, frame_id)
    except Exception as exc:
        rospy.logfatal(f"Failed to load PCD: {exc}")
        return

    rospy.loginfo(f"Loaded PCD: {pcd_path}, publishing to {topic} at {publish_hz} Hz")

    rate = rospy.Rate(max(publish_hz, 1e-3))
    while not rospy.is_shutdown():
        cloud_msg.header.stamp = rospy.Time.now()
        pub.publish(cloud_msg)
        rate.sleep()


if __name__ == "__main__":
    main()
