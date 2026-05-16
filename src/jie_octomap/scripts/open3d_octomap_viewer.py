#!/usr/bin/env python3
# open3d_octomap_viewer.py  —  ROS 1 port of the original ROS 2 implementation

import time
from typing import Optional

import numpy as np
import open3d as o3d
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


# ── State shared between rospy subscriber thread and main thread ───────────────
_latest_points: Optional[np.ndarray] = None
_dirty = False
_cloud_count = 0


def _cloud_cb(msg: PointCloud2) -> None:
    global _latest_points, _dirty, _cloud_count
    _cloud_count += 1
    records = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if not records:
        return
    _latest_points = np.asarray(records, dtype=np.float64)
    _dirty = True
    if _cloud_count % 10 == 0:
        rospy.loginfo(f"Received cloud messages: {_cloud_count}, points={_latest_points.shape[0]}")


def main() -> None:
    global _latest_points, _dirty

    rospy.init_node("open3d_octomap_viewer")
    cloud_topic = rospy.get_param("~cloud_topic", "/octomap_points")
    rospy.Subscriber(cloud_topic, PointCloud2, _cloud_cb, queue_size=1)
    rospy.loginfo(f"Open3D viewer started. cloud_topic={cloud_topic}")

    vis = o3d.visualization.Visualizer()
    ok = vis.create_window(window_name="Open3D OctoMap Viewer", width=1280, height=800)
    if not ok:
        rospy.logerr("Open3D window create failed. Check DISPLAY/GL driver.")
        return

    pcd = o3d.geometry.PointCloud()
    geometry_added = False

    try:
        while not rospy.is_shutdown():
            if _dirty and _latest_points is not None:
                points = _latest_points
                _dirty = False
                pcd.points = o3d.utility.Vector3dVector(points)
                colors = np.zeros_like(points)
                colors[:, 0] = 1.0
                colors[:, 1] = 0.55
                colors[:, 2] = 0.2
                pcd.colors = o3d.utility.Vector3dVector(colors)
                if not geometry_added:
                    vis.add_geometry(pcd)
                    vis.get_view_control().set_zoom(0.35)
                    geometry_added = True
                else:
                    vis.update_geometry(pcd)

            keep_running = vis.poll_events()
            vis.update_renderer()
            if not keep_running:
                rospy.loginfo("Open3D window closed.")
                break
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        vis.destroy_window()


if __name__ == "__main__":
    main()
