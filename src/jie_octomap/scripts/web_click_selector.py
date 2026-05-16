#!/usr/bin/env python3
# web_click_selector.py  —  ROS 1 port of the original ROS 2 implementation

import math
from typing import Optional, Set, Tuple

import rospy
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

GridIndex = Tuple[int, int, int]


class WebClickSelectorNode:
    def __init__(self) -> None:
        # Parameters
        self._occupied_marker_topic = rospy.get_param("~occupied_marker_topic",  "/octomap_occupied_markers")
        self._preblocked_marker_topic = rospy.get_param("~preblocked_marker_topic", "/preblocked_cells_markers")
        self._raw_click_topic = rospy.get_param("~raw_click_topic",         "/web_clicked_point")
        self._marker_topic = rospy.get_param("~marker_topic",            "/selection_markers")
        self._start_topic = rospy.get_param("~start_topic",             "/start_point")
        self._goal_topic = rospy.get_param("~goal_topic",              "/goal_point")
        self._status_topic = rospy.get_param("~status_topic",            "/web_selection_status")

        self._arrow_height = rospy.get_param("~arrow_height",            0.6)
        self._arrow_length = rospy.get_param("~arrow_length",            0.7)
        self._shaft_diameter = rospy.get_param("~shaft_diameter",          0.16)
        self._head_diameter = rospy.get_param("~head_diameter",           0.32)
        self._head_length = rospy.get_param("~head_length",             0.44)
        self._cube_size = rospy.get_param("~cube_size",               0.20)
        self._robot_radius = rospy.get_param("~robot_radius",            0.25)
        self._snap_search_radius = rospy.get_param("~snap_search_radius_cells", 12)
        self._require_ground = rospy.get_param("~require_ground_support",  True)
        self._strict_direct = rospy.get_param("~strict_direct_ground_support", False)
        self._support_xy_radius = rospy.get_param("~ground_support_xy_radius_cells", 1)
        self._support_depth = rospy.get_param("~ground_support_depth_cells", 1)

        # Publishers (latch=True mimics transient_local)
        self.marker_pub = rospy.Publisher(self._marker_topic, MarkerArray, queue_size=1, latch=True)
        self.start_pub = rospy.Publisher(self._start_topic,  PointStamped, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher(self._goal_topic,   PointStamped, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(self._status_topic, String,       queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber(self._occupied_marker_topic,   Marker,       self._on_occupied,  queue_size=1)
        rospy.Subscriber(self._preblocked_marker_topic, Marker,       self._on_preblocked, queue_size=1)
        rospy.Subscriber(self._raw_click_topic,         PointStamped, self._on_raw_click, queue_size=10)

        # State
        self.expect_start = True
        self.has_start = False
        self.has_goal = False
        self.start_point: Optional[PointStamped] = None
        self.goal_point:  Optional[PointStamped] = None

        self.map_frame = "map"
        self.resolution = 0.2
        self.occupied_cells: Set[GridIndex] = set()
        self.preblocked_cells: Set[GridIndex] = set()
        self.min_idx: Optional[GridIndex] = None
        self.max_idx: Optional[GridIndex] = None

        self._publish_status("等待占据栅格地图。")
        rospy.loginfo(
            f"web_click_selector started. "
            f"raw_click_topic={self._raw_click_topic} "
            f"occupied_marker_topic={self._occupied_marker_topic}"
        )

    # ---- helpers ----
    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _world_to_grid(self, x: float, y: float, z: float) -> GridIndex:
        r = self.resolution
        return (math.floor(x / r), math.floor(y / r), math.floor(z / r))

    def _grid_to_point(self, idx: GridIndex) -> Point:
        r = self.resolution
        p = Point()
        p.x = (idx[0] + 0.5) * r
        p.y = (idx[1] + 0.5) * r
        p.z = (idx[2] + 0.5) * r
        return p

    def _is_inside_bounds(self, idx: GridIndex) -> bool:
        if self.min_idx is None or self.max_idx is None:
            return False
        return (self.min_idx[0] <= idx[0] <= self.max_idx[0]
                and self.min_idx[1] <= idx[1] <= self.max_idx[1]
                and self.min_idx[2] <= idx[2] <= self.max_idx[2])

    def _is_occupied(self, idx: GridIndex) -> bool:
        return idx in self.occupied_cells

    def _has_ground_support(self, idx: GridIndex) -> bool:
        if self._strict_direct:
            below = (idx[0], idx[1], idx[2] - 1)
            return self._is_inside_bounds(below) and self._is_occupied(below)
        for dz in range(1, max(1, self._support_depth) + 1):
            for dx in range(-self._support_xy_radius, self._support_xy_radius + 1):
                for dy in range(-self._support_xy_radius, self._support_xy_radius + 1):
                    below = (idx[0] + dx, idx[1] + dy, idx[2] - dz)
                    if self._is_inside_bounds(below) and self._is_occupied(below):
                        return True
        return False

    def _is_traversable(self, idx: GridIndex) -> bool:
        if not self._is_inside_bounds(idx):
            return False
        if self._is_occupied(idx):
            return False
        if idx in self.preblocked_cells:
            return False
        if self._require_ground and not self._has_ground_support(idx):
            return False
        n = max(1, math.ceil(self._robot_radius / self.resolution))
        radius_sq = self._robot_radius * self._robot_radius
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                for dz in range(0, n + 1):
                    dist_sq = ((dx * self.resolution) ** 2
                               + (dy * self.resolution) ** 2
                               + (dz * self.resolution) ** 2)
                    if dist_sq > radius_sq:
                        continue
                    if self._is_occupied((idx[0] + dx, idx[1] + dy, idx[2] + dz)):
                        return False
        return True

    def _find_nearest_traversable(self, seed: GridIndex) -> Optional[GridIndex]:
        if self._is_traversable(seed):
            return seed
        for r in range(1, self._snap_search_radius + 1):
            for dz in range(0, r + 1):
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        if max(abs(dx), abs(dy), abs(dz)) != r:
                            continue
                        c1 = (seed[0] + dx, seed[1] + dy, seed[2] + dz)
                        if self._is_traversable(c1):
                            return c1
                        if dz > 0:
                            c2 = (seed[0] + dx, seed[1] + dy, seed[2] - dz)
                            if self._is_traversable(c2):
                                return c2
        return None

    # ---- subscribers ----
    def _on_occupied(self, msg: Marker) -> None:
        if msg.type != Marker.CUBE_LIST:
            return
        self.map_frame = msg.header.frame_id or self.map_frame
        self.resolution = max(1e-6, float(msg.scale.x))
        occupied: Set[GridIndex] = set()
        min_x = min_y = min_z = math.inf
        max_x = max_y = max_z = -math.inf
        for p in msg.points:
            idx = self._world_to_grid(p.x, p.y, p.z)
            occupied.add(idx)
            min_x = min(min_x, idx[0])
            min_y = min(min_y, idx[1])
            min_z = min(min_z, idx[2])
            max_x = max(max_x, idx[0])
            max_y = max(max_y, idx[1])
            max_z = max(max_z, idx[2])
        self.occupied_cells = occupied
        if occupied:
            self.min_idx = (int(min_x), int(min_y), int(min_z))
            self.max_idx = (int(max_x), int(max_y), int(max_z))
            self._publish_status(f"地图已就绪：{len(self.occupied_cells)} 个占据栅格。")

    def _on_preblocked(self, msg: Marker) -> None:
        if msg.type != Marker.CUBE_LIST:
            return
        self.preblocked_cells = {self._world_to_grid(p.x, p.y, p.z) for p in msg.points}

    def _on_raw_click(self, msg: PointStamped) -> None:
        if not self.occupied_cells or self.min_idx is None:
            self._publish_status("地图尚未就绪。")
            return
        seed = self._world_to_grid(msg.point.x, msg.point.y, msg.point.z)
        snapped = self._find_nearest_traversable(seed)
        if snapped is None:
            self._publish_status("点击位置附近没有找到具备地面支撑的可通行栅格。")
            rospy.logwarn(
                f"Raw click [{msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f}] "
                "could not be snapped to a traversable cell."
            )
            return

        point = self._grid_to_point(snapped)
        snapped_msg = PointStamped()
        snapped_msg.header.frame_id = self.map_frame
        snapped_msg.header.stamp = rospy.Time.now()
        snapped_msg.point = point

        if self.expect_start:
            self.start_point = snapped_msg
            self.has_start = True
            self.start_pub.publish(snapped_msg)
            self._publish_status(
                f"起点已设置为 [{point.x:.2f}, {point.y:.2f}, {point.z:.2f}]。再次点击可设置终点。"
            )
            rospy.loginfo(f"Set START point: [{point.x:.3f}, {point.y:.3f}, {point.z:.3f}]")
        else:
            self.goal_point = snapped_msg
            self.has_goal = True
            self.goal_pub.publish(snapped_msg)
            self._publish_status(
                f"终点已设置为 [{point.x:.2f}, {point.y:.2f}, {point.z:.2f}]。再次点击可设置起点。"
            )
            rospy.loginfo(f"Set GOAL point: [{point.x:.3f}, {point.y:.3f}, {point.z:.3f}]")

        self.expect_start = not self.expect_start
        self._publish_markers()

    # ---- marker helpers ----
    def _make_arrow(self, marker_id: int, p: PointStamped, rgb: Tuple[float, float, float]) -> Marker:
        m = Marker()
        m.header = p.header
        m.ns = "web_selector"
        m.id = marker_id
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.scale.x = self._shaft_diameter
        m.scale.y = self._head_diameter
        m.scale.z = self._head_length
        m.color.r = rgb[0]
        m.color.g = rgb[1]
        m.color.b = rgb[2]
        m.color.a = 1.0
        m.pose.orientation.w = 1.0
        base = Point()
        base.x = p.point.x
        base.y = p.point.y
        base.z = p.point.z + self._arrow_height
        tip = Point()
        tip.x = base.x
        tip.y = base.y
        tip.z = base.z - self._arrow_length
        m.points = [base, tip]
        return m

    def _make_cube(self, marker_id: int, p: PointStamped, rgb: Tuple[float, float, float]) -> Marker:
        m = Marker()
        m.header = p.header
        m.ns = "web_selector"
        m.id = marker_id
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position = p.point
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = self._cube_size
        m.color.r = rgb[0]
        m.color.g = rgb[1]
        m.color.b = rgb[2]
        m.color.a = 0.95
        return m

    def _publish_markers(self) -> None:
        markers = MarkerArray()
        if self.has_start and self.start_point is not None:
            markers.markers.append(self._make_arrow(0, self.start_point, (0.1, 0.95, 0.1)))
            markers.markers.append(self._make_cube(2, self.start_point, (0.1, 0.95, 0.1)))
        if self.has_goal and self.goal_point is not None:
            markers.markers.append(self._make_arrow(1, self.goal_point, (0.95, 0.1, 0.1)))
            markers.markers.append(self._make_cube(3, self.goal_point, (0.95, 0.1, 0.1)))
        self.marker_pub.publish(markers)


def main() -> None:
    rospy.init_node("web_click_selector")
    node = WebClickSelectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
