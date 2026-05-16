#!/usr/bin/env python3
# map_package_manager.py  —  ROS 1 port of the original ROS 2 implementation
#
# 地图包格式（与 ROS2 版本完全相同，文件互通）：
# map_package/
# ├── meta.yaml          # 元数据
# ├── octomap_msg.npz    # Octomap 二进制数据
# └── layers.npz         # 各层栅格数据

from __future__ import annotations

import copy
import threading
from pathlib import Path
from typing import Optional

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import Point
from jie_map_msgs.srv import (
    ExportNavigationSnapshot, ExportNavigationSnapshotRequest,
    GetNavigationMapMeta,     GetNavigationMapMetaRequest,
    LoadNavigationMapPackage, LoadNavigationMapPackageResponse,
    SaveNavigationMapPackage, SaveNavigationMapPackageResponse,
)
from octomap_msgs.msg import Octomap
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker


class MapPackageManager:
    def __init__(self) -> None:
        # Parameters
        self._octomap_topic = rospy.get_param("~octomap_topic",         "/octomap")
        self._occupied_topic = rospy.get_param("~occupied_marker_topic", "/octomap_occupied_markers")
        self._preblocked_topic = rospy.get_param("~preblocked_topic",      "/preblocked_cells_markers")
        self._traversable_topic = rospy.get_param("~traversable_topic",     "/traversable_cells_markers")
        self._risk_cost_topic = rospy.get_param("~risk_cost_topic",       "/risk_cost_cells")
        self._meta_service = rospy.get_param("~planner_meta_service",  "/jie_path_node/get_meta")
        self._export_service = rospy.get_param("~planner_export_service", "/jie_path_node/export_snapshot")
        self._autoload_path = rospy.get_param("~autoload_package_path", "").strip()

        # Latest messages (protected by lock because callbacks run in threads)
        self._lock = threading.Lock()
        self._latest_octomap:     Optional[Octomap] = None
        self._latest_occupied:    Optional[Marker] = None
        self._latest_preblocked:  Optional[Marker] = None
        self._latest_traversable: Optional[Marker] = None
        self._latest_risk_cost:   Optional[PointCloud2] = None

        # Publishers (latch=True → transient_local equivalent)
        self._octomap_pub = rospy.Publisher(self._octomap_topic,     Octomap,     queue_size=1, latch=True)
        self._occupied_pub = rospy.Publisher(self._occupied_topic,    Marker,      queue_size=1, latch=True)
        self._preblocked_pub = rospy.Publisher(self._preblocked_topic,  Marker,      queue_size=1, latch=True)
        self._traversable_pub = rospy.Publisher(self._traversable_topic, Marker,      queue_size=1, latch=True)
        self._risk_cost_pub = rospy.Publisher(self._risk_cost_topic,   PointCloud2, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber(self._octomap_topic,     Octomap,     self._on_octomap,     queue_size=1)
        rospy.Subscriber(self._occupied_topic,    Marker,      self._on_occupied,    queue_size=1)
        rospy.Subscriber(self._preblocked_topic,  Marker,      self._on_preblocked,  queue_size=1)
        rospy.Subscriber(self._traversable_topic, Marker,      self._on_traversable, queue_size=1)
        rospy.Subscriber(self._risk_cost_topic,   PointCloud2, self._on_risk_cost,   queue_size=1)

        # Services
        rospy.Service("~save_package", SaveNavigationMapPackage, self._handle_save_package)
        rospy.Service("~load_package", LoadNavigationMapPackage, self._handle_load_package)

        # Autoload (1 s after startup)
        if self._autoload_path:
            threading.Timer(1.0, self._autoload_package_once).start()

        rospy.loginfo("Map Package Manager started. "
                      "save_service=~save_package load_service=~load_package")

    # ── subscribers ────────────────────────────────────────────────────────
    def _on_octomap(self, msg: Octomap) -> None:
        with self._lock:
            self._latest_octomap = copy.deepcopy(msg)

    def _on_occupied(self, msg: Marker) -> None:
        if msg.type == Marker.CUBE_LIST:
            with self._lock:
                self._latest_occupied = copy.deepcopy(msg)

    def _on_preblocked(self, msg: Marker) -> None:
        if msg.type == Marker.CUBE_LIST:
            with self._lock:
                self._latest_preblocked = copy.deepcopy(msg)

    def _on_traversable(self, msg: Marker) -> None:
        if msg.type == Marker.CUBE_LIST:
            with self._lock:
                self._latest_traversable = copy.deepcopy(msg)

    def _on_risk_cost(self, msg: PointCloud2) -> None:
        with self._lock:
            self._latest_risk_cost = copy.deepcopy(msg)

    # ── helpers ─────────────────────────────────────────────────────────────
    @staticmethod
    def _marker_points_to_numpy(marker: Marker) -> np.ndarray:
        return np.array([[p.x, p.y, p.z] for p in marker.points], dtype=np.float32)

    @staticmethod
    def _make_marker_from_points(
        frame_id: str,
        ns: str,
        scale: np.ndarray,
        points: np.ndarray,
        color: tuple,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        for xyz in points:
            p = Point()
            p.x = float(xyz[0])
            p.y = float(xyz[1])
            p.z = float(xyz[2])
            marker.points.append(p)
        return marker

    def _call_export_snapshot(self):
        """Call planner export service synchronously. Returns (ok, msg, response|None)."""
        try:
            rospy.wait_for_service(self._export_service, timeout=1.0)
        except rospy.ROSException:
            return False, "planner export service unavailable", None
        try:
            proxy = rospy.ServiceProxy(self._export_service, ExportNavigationSnapshot)
            req = ExportNavigationSnapshotRequest()
            req.recompute_layers = True
            resp = proxy(req)
            return resp.success, resp.message, resp
        except rospy.ServiceException as exc:
            return False, str(exc), None

    def _call_get_meta(self):
        """Call planner meta service synchronously. Returns (ok, msg, response|None)."""
        try:
            rospy.wait_for_service(self._meta_service, timeout=1.0)
        except rospy.ROSException:
            return False, "planner meta service unavailable", None
        try:
            proxy = rospy.ServiceProxy(self._meta_service, GetNavigationMapMeta)
            resp = proxy(GetNavigationMapMetaRequest())
            return resp.success, resp.message, resp
        except rospy.ServiceException as exc:
            return False, str(exc), None

    # ── save service handler ────────────────────────────────────────────────
    def _handle_save_package(self, request):
        response = SaveNavigationMapPackageResponse()

        export_ok, export_msg, export_result = self._call_export_snapshot()
        if not export_ok or export_result is None:
            response.success = False
            response.message = export_msg
            return response

        meta_ok, meta_msg, meta = self._call_get_meta()
        if not meta_ok or meta is None:
            response.success = False
            response.message = meta_msg
            return response

        with self._lock:
            octomap = copy.deepcopy(self._latest_octomap)
            preblocked = copy.deepcopy(self._latest_preblocked)
            traversable = copy.deepcopy(self._latest_traversable)
            risk_cost = copy.deepcopy(self._latest_risk_cost)

        for name, val in [("octomap",     octomap),
                          ("preblocked",  preblocked),
                          ("traversable", traversable),
                          ("risk_cost",   risk_cost)]:
            if val is None:
                response.success = False
                response.message = f"{name} message not received yet"
                return response

        package_dir = Path(request.package_path).expanduser()
        if package_dir.exists() and not request.overwrite:
            response.success = False
            response.message = f"package path already exists: {package_dir}"
            return response
        package_dir.mkdir(parents=True, exist_ok=True)

        octomap_file = package_dir / "octomap_msg.npz"
        layers_file = package_dir / "layers.npz"
        meta_file = package_dir / "meta.yaml"

        # Save octomap
        np.savez_compressed(
            octomap_file,
            binary=np.array([octomap.binary],     dtype=np.bool_),
            octomap_id=np.array([octomap.id]),
            resolution=np.array([octomap.resolution], dtype=np.float64),
            frame_id=np.array([octomap.header.frame_id]),
            data=np.array(octomap.data, dtype=np.int8),
        )

        # Save layers
        preblocked_points = self._marker_points_to_numpy(preblocked)
        traversable_points = self._marker_points_to_numpy(traversable)

        risk_records = list(pc2.read_points(
            risk_cost, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        risk_arr = (np.array([[r[0], r[1], r[2], r[3]] for r in risk_records], dtype=np.float32)
                    if risk_records else np.empty((0, 4), dtype=np.float32))

        np.savez_compressed(
            layers_file,
            preblocked_points=preblocked_points,
            preblocked_scale=np.array([preblocked.scale.x,  preblocked.scale.y,
                                      preblocked.scale.z],  dtype=np.float64),
            preblocked_frame_id=np.array([preblocked.header.frame_id]),
            traversable_points=traversable_points,
            traversable_scale=np.array([traversable.scale.x, traversable.scale.y,
                                       traversable.scale.z], dtype=np.float64),
            traversable_frame_id=np.array([traversable.header.frame_id]),
            risk_points=risk_arr[:, :3] if risk_arr.size else np.empty((0, 3), dtype=np.float32),
            risk_intensity=risk_arr[:, 3] if risk_arr.size else np.empty((0,),  dtype=np.float32),
            risk_frame_id=np.array([risk_cost.header.frame_id]),
        )

        # Save meta.yaml
        # ROS1 rospy.Time has .secs / .nsecs (not .sec / .nanosec)
        snap_stamp = export_result.snapshot_stamp
        meta_yaml = {
            "map_id":            meta.map_id,
            "frame_id":          meta.frame_id,
            "resolution":        meta.resolution,
            "octomap_file":      octomap_file.name,
            "layers_file":       layers_file.name,
            "source_world_file": meta.source_world_file,
            "snapshot_stamp": {
                "sec":     int(snap_stamp.secs),
                "nanosec": int(snap_stamp.nsecs),
            },
            "bounds": {
                "min": [meta.min_bound.x, meta.min_bound.y, meta.min_bound.z],
                "max": [meta.max_bound.x, meta.max_bound.y, meta.max_bound.z],
            },
            "planner": {
                "robot_radius":                   meta.robot_radius,
                "snap_search_radius_cells":        meta.snap_search_radius_cells,
                "require_ground_support":          meta.require_ground_support,
                "strict_direct_ground_support":    meta.strict_direct_ground_support,
                "ground_support_xy_radius_cells":  meta.ground_support_xy_radius_cells,
                "ground_support_depth_cells":      meta.ground_support_depth_cells,
                "enable_preblocked_costmap":       meta.enable_preblocked_costmap,
                "preblocked_costmap_radius_cells": meta.preblocked_costmap_radius_cells,
                "preblocked_costmap_weight":       meta.preblocked_costmap_weight,
            },
            "layers": {
                "preblocked_count":  int(preblocked_points.shape[0]),
                "traversable_count": int(traversable_points.shape[0]),
                "risk_cost_count":   int(risk_arr.shape[0]),
            },
        }
        with meta_file.open("w", encoding="utf-8") as f:
            yaml.safe_dump(meta_yaml, f, sort_keys=False, allow_unicode=True)

        response.success = True
        response.message = "map package saved"
        response.manifest_path = str(meta_file)
        return response

    # ── load service handler ────────────────────────────────────────────────
    def _handle_load_package(self, request):
        success, message, map_id = self._load_package(request.package_path)
        response = LoadNavigationMapPackageResponse()
        response.success = success
        response.message = message
        response.map_id = map_id
        return response

    def _autoload_package_once(self) -> None:
        if not self._autoload_path:
            rospy.loginfo(f"autoloaded map package path error")
            return
        success, message, map_id = self._load_package(self._autoload_path)
        if success:
            rospy.loginfo(f"autoloaded map package: {self._autoload_path} map_id={map_id}")
        else:
            rospy.logerr(f"failed to autoload map package {self._autoload_path}: {message}")

    def _load_package(self, package_path: str):
        """Returns (success, message, map_id)."""
        package_dir = Path(package_path).expanduser()
        meta_file = package_dir / "meta.yaml"
        if not meta_file.exists():
            return False, f"meta file not found: {meta_file}", ""

        with meta_file.open("r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)

        octomap_npz = np.load(package_dir / meta["octomap_file"], allow_pickle=False)
        layers_npz = np.load(package_dir / meta["layers_file"],  allow_pickle=False)

        # Rebuild Octomap message
        octomap_msg = Octomap()
        octomap_msg.header.frame_id = str(octomap_npz["frame_id"][0])
        octomap_msg.header.stamp = rospy.Time.now()
        octomap_msg.binary = bool(octomap_npz["binary"][0])
        octomap_msg.id = str(octomap_npz["octomap_id"][0])
        octomap_msg.resolution = float(octomap_npz["resolution"][0])
        octomap_msg.data = octomap_npz["data"].astype(np.int8).tolist()

        # Rebuild occupied marker (optional, may not be in older packages)
        occupied_msg = None
        if "occupied_points" in layers_npz:
            occupied_msg = self._make_marker_from_points(
                str(layers_npz["occupied_frame_id"][0]),
                "occupied_voxels",
                layers_npz["occupied_scale"],
                layers_npz["occupied_points"],
                (0.95, 0.45, 0.15, 0.95),
            )

        preblocked_msg = self._make_marker_from_points(
            str(layers_npz["preblocked_frame_id"][0]),
            "preblocked_cells",
            layers_npz["preblocked_scale"],
            layers_npz["preblocked_points"],
            (0.15, 0.35, 1.0, 0.95),
        )
        traversable_msg = self._make_marker_from_points(
            str(layers_npz["traversable_frame_id"][0]),
            "traversable_cells",
            layers_npz["traversable_scale"],
            layers_npz["traversable_points"],
            (0.20, 0.95, 0.55, 0.55),
        )

        # Rebuild risk PointCloud2
        risk_header = rospy.Header()
        risk_header.frame_id = str(layers_npz["risk_frame_id"][0])
        risk_header.stamp = rospy.Time.now()
        risk_points = layers_npz["risk_points"]
        risk_intensity = layers_npz["risk_intensity"]
        fields = [
            PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        risk_msg = pc2.create_cloud(
            risk_header,
            fields,
            [(float(p[0]), float(p[1]), float(p[2]), float(i))
             for p, i in zip(risk_points, risk_intensity)],
        )

        # Store and publish
        with self._lock:
            self._latest_octomap = copy.deepcopy(octomap_msg)
            self._latest_occupied = copy.deepcopy(occupied_msg) if occupied_msg else None
            self._latest_preblocked = copy.deepcopy(preblocked_msg)
            self._latest_traversable = copy.deepcopy(traversable_msg)
            self._latest_risk_cost = copy.deepcopy(risk_msg)

        self._octomap_pub.publish(octomap_msg)
        if occupied_msg is not None:
            self._occupied_pub.publish(occupied_msg)
        self._preblocked_pub.publish(preblocked_msg)
        self._traversable_pub.publish(traversable_msg)
        self._risk_cost_pub.publish(risk_msg)

        return True, "map package loaded", str(meta.get("map_id", ""))


def main() -> None:
    rospy.init_node("map_package_manager")
    node = MapPackageManager()
    rospy.spin()


if __name__ == "__main__":
    main()
