#!/usr/bin/env python3
# map_viewer_gui.py  —  ROS 1 port of the original ROS 2 implementation

from __future__ import annotations

import os
import threading
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rospy
import yaml
from jie_map_msgs.srv import (
    LoadNavigationMapPackage, LoadNavigationMapPackageRequest,
    SaveNavigationMapPackage, SaveNavigationMapPackageRequest,
)
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from PyQt5.QtCore import QEvent, Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtWidgets import (
    QApplication,
    QButtonGroup,
    QCheckBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QVBoxLayout,
    QWidget,
)
import tf2_ros
from visualization_msgs.msg import Marker
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk
from vtk.util import numpy_support


# ── Service workers ────────────────────────────────────────────────────────────
class SaveWorker(QObject):
    finished = pyqtSignal(bool, str)

    def __init__(self, package_path: str, overwrite: bool) -> None:
        super().__init__()
        self.package_path = package_path
        self.overwrite    = overwrite

    def run(self) -> None:
        service_name = "/map_package_manager/save_package"
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            proxy = rospy.ServiceProxy(service_name, SaveNavigationMapPackage)
            req = SaveNavigationMapPackageRequest()
            req.package_path = self.package_path
            req.overwrite    = self.overwrite
            resp = proxy(req)
            self.finished.emit(bool(resp.success), str(resp.message))
        except rospy.ROSException:
            self.finished.emit(False, f"保存服务 {service_name} 不可用。")
        except rospy.ServiceException as exc:
            self.finished.emit(False, f"保存地图失败：{exc}")


class LoadWorker(QObject):
    finished = pyqtSignal(bool, str, str)

    def __init__(self, package_path: str) -> None:
        super().__init__()
        self.package_path = package_path

    def run(self) -> None:
        service_name = "/map_package_manager/load_package"
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            proxy = rospy.ServiceProxy(service_name, LoadNavigationMapPackage)
            req = LoadNavigationMapPackageRequest()
            req.package_path = self.package_path
            resp = proxy(req)
            self.finished.emit(bool(resp.success), str(resp.message), self.package_path)
        except rospy.ROSException:
            self.finished.emit(False, f"读取服务 {service_name} 不可用。", self.package_path)
        except rospy.ServiceException as exc:
            self.finished.emit(False, f"读取地图失败：{exc}", self.package_path)


# ── ROS interface ──────────────────────────────────────────────────────────────
class MapViewerRosNode:
    def __init__(self) -> None:
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._tf_parent = rospy.get_param("~tf_parent_frame", "map")
        self._tf_child  = rospy.get_param("~tf_child_frame",  "base_footprint")

        # Publishers (latch=True → transient_local)
        self.start_pub              = rospy.Publisher("/start_point",                   PointStamped,             queue_size=1, latch=True)
        self.goal_pub               = rospy.Publisher("/goal_point",                    PointStamped,             queue_size=1, latch=True)
        self.goal_pose_pub          = rospy.Publisher("/goal_pose",                     PoseStamped,              queue_size=1, latch=True)
        self.initial_pose_pub       = rospy.Publisher("/initialpose",                   PoseWithCovarianceStamped,queue_size=10)
        self.external_preblocked_pub= rospy.Publisher("/edited_preblocked_cells_markers",Marker,                  queue_size=1, latch=True)
        self.edited_occupied_pub    = rospy.Publisher("/edited_occupied_markers",        Marker,                  queue_size=1, latch=True)
        self.occupied_pub           = rospy.Publisher("/octomap_occupied_markers",       Marker,                  queue_size=1, latch=True)
        self.preblocked_pub         = rospy.Publisher("/preblocked_cells_markers",       Marker,                  queue_size=1, latch=True)
        self.traversable_pub        = rospy.Publisher("/traversable_cells_markers",      Marker,                  queue_size=1, latch=True)
        self.risk_pub               = rospy.Publisher("/risk_cost_cells",                PointCloud2,             queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber("/planned_path",              PathMsg,     self._on_path,        queue_size=1)
        rospy.Subscriber("/octomap_occupied_markers",  Marker,      self._on_occupied,    queue_size=1)
        rospy.Subscriber("/preblocked_cells_markers",  Marker,      self._on_preblocked,  queue_size=1)
        rospy.Subscriber("/traversable_cells_markers", Marker,      self._on_traversable, queue_size=1)
        rospy.Subscriber("/risk_cost_cells",           PointCloud2, self._on_risk,        queue_size=1)

        # State (dirty flags; callbacks run in rospy threads)
        self._latest_path_points = []
        self._path_dirty         = False
        self._latest_occupied    = None
        self._occupied_dirty     = False
        self._latest_preblocked  = None
        self._preblocked_dirty   = False
        self._latest_traversable = None
        self._traversable_dirty  = False
        self._latest_risk        = None
        self._risk_dirty         = False

    # ── pub helpers ───────────────────────────────────────────────────────
    def _now(self): return rospy.Time.now()

    def publish_point(self, topic: str, frame_id: str, xyz) -> None:
        msg = PointStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp    = self._now()
        msg.point.x = float(xyz[0]); msg.point.y = float(xyz[1]); msg.point.z = float(xyz[2])
        (self.start_pub if topic == "start" else self.goal_pub).publish(msg)

    def publish_goal_pose(self, frame_id: str, xyz, yaw: float) -> None:
        msg = PoseStamped()
        msg.header.frame_id = frame_id; msg.header.stamp = self._now()
        msg.pose.position.x = float(xyz[0]); msg.pose.position.y = float(xyz[1]); msg.pose.position.z = float(xyz[2])
        half = float(yaw) * 0.5
        msg.pose.orientation.z = float(np.sin(half)); msg.pose.orientation.w = float(np.cos(half))
        self.goal_pose_pub.publish(msg)

    def publish_initial_pose(self, frame_id: str, xyz, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id; msg.header.stamp = self._now()
        msg.pose.pose.position.x = float(xyz[0]); msg.pose.pose.position.y = float(xyz[1]); msg.pose.pose.position.z = float(xyz[2])
        half = float(yaw) * 0.5
        msg.pose.pose.orientation.z = float(np.sin(half)); msg.pose.pose.orientation.w = float(np.cos(half))
        msg.pose.covariance[0] = 0.25; msg.pose.covariance[7] = 0.25; msg.pose.covariance[35] = 0.06853891909122467
        self.initial_pose_pub.publish(msg)

    def publish_external_preblocked(self, frame_id: str, points: np.ndarray, scale: np.ndarray) -> None:
        msg = self._make_marker(frame_id, "external_preblocked_cells", scale, points, (0.95, 0.10, 0.10, 0.95))
        self.external_preblocked_pub.publish(msg)

    def publish_edited_occupied(self, frame_id: str, points: np.ndarray, scale: np.ndarray) -> None:
        msg = self._make_marker(frame_id, "edited_occupied_cells", scale, points, (0.95, 0.45, 0.15, 1.0))
        self.edited_occupied_pub.publish(msg)

    def publish_voxel_marker(self, layer_name: str, frame_id: str,
                             points: np.ndarray, scale: np.ndarray,
                             color, opacity: float) -> None:
        rgba = (color[0], color[1], color[2], opacity)
        ns_map = {"occupied": "occupied_voxels", "preblocked": "preblocked_cells", "traversable": "traversable_cells"}
        ns = ns_map.get(layer_name, f"{layer_name}_cells")
        msg = self._make_marker(frame_id, ns, scale, points, rgba)
        pub_map = {"occupied": self.occupied_pub, "preblocked": self.preblocked_pub, "traversable": self.traversable_pub}
        pub = pub_map.get(layer_name)
        if pub: pub.publish(msg)

    def publish_risk_cloud(self, frame_id: str, points: np.ndarray, intensity: np.ndarray) -> None:
        header = Header(); header.frame_id = frame_id; header.stamp = self._now()
        fields = [
            PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = pc2.create_cloud(header, fields,
            [(float(p[0]), float(p[1]), float(p[2]), float(i)) for p, i in zip(points, intensity)])
        self.risk_pub.publish(cloud)

    @staticmethod
    def _make_marker(frame_id, ns, scale, points, rgba) -> Marker:
        msg = Marker()
        msg.header.frame_id = frame_id; msg.header.stamp = rospy.Time.now()
        msg.ns = ns; msg.id = 0
        msg.type = Marker.CUBE_LIST; msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        msg.scale.x = float(scale[0]); msg.scale.y = float(scale[1]); msg.scale.z = float(scale[2])
        msg.color.r = float(rgba[0]); msg.color.g = float(rgba[1]); msg.color.b = float(rgba[2]); msg.color.a = float(rgba[3])
        for pt in points:
            p = Point(); p.x = float(pt[0]); p.y = float(pt[1]); p.z = float(pt[2])
            msg.points.append(p)
        return msg

    # ── subscribers ───────────────────────────────────────────────────────
    def _on_path(self, msg: PathMsg) -> None:
        self._latest_path_points = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
        self._path_dirty = True

    def _on_occupied(self, msg: Marker) -> None:
        if msg.type != Marker.CUBE_LIST: return
        self._latest_occupied = (np.array([[p.x,p.y,p.z] for p in msg.points],dtype=np.float32),
                                  np.array([msg.scale.x,msg.scale.y,msg.scale.z],dtype=np.float32))
        self._occupied_dirty = True

    def _on_preblocked(self, msg: Marker) -> None:
        if msg.type != Marker.CUBE_LIST: return
        self._latest_preblocked = (np.array([[p.x,p.y,p.z] for p in msg.points],dtype=np.float32),
                                    np.array([msg.scale.x,msg.scale.y,msg.scale.z],dtype=np.float32))
        self._preblocked_dirty = True

    def _on_traversable(self, msg: Marker) -> None:
        if msg.type != Marker.CUBE_LIST: return
        self._latest_traversable = (np.array([[p.x,p.y,p.z] for p in msg.points],dtype=np.float32),
                                     np.array([msg.scale.x,msg.scale.y,msg.scale.z],dtype=np.float32))
        self._traversable_dirty = True

    def _on_risk(self, msg: PointCloud2) -> None:
        records = list(pc2.read_points(msg, field_names=("x","y","z","intensity"), skip_nans=True))
        if not records:
            xyz = np.empty((0,3),dtype=np.float32); intensity = np.empty((0,),dtype=np.float32)
        else:
            arr = np.array([[r[0],r[1],r[2],r[3]] for r in records],dtype=np.float32)
            xyz = arr[:,:3]; intensity = arr[:,3]
        scale = self._latest_occupied[1] if self._latest_occupied else np.array([0.2,0.2,0.2],dtype=np.float32)
        self._latest_risk = (xyz, scale, intensity)
        self._risk_dirty = True

    # ── consume ───────────────────────────────────────────────────────────
    def consume_path(self):
        if not self._path_dirty: return None
        self._path_dirty = False; return list(self._latest_path_points)

    def consume_occupied(self):
        if not self._occupied_dirty or self._latest_occupied is None: return None
        self._occupied_dirty = False; return self._latest_occupied

    def consume_preblocked(self):
        if not self._preblocked_dirty or self._latest_preblocked is None: return None
        self._preblocked_dirty = False; return self._latest_preblocked

    def consume_traversable(self):
        if not self._traversable_dirty or self._latest_traversable is None: return None
        self._traversable_dirty = False; return self._latest_traversable

    def consume_risk(self):
        if not self._risk_dirty or self._latest_risk is None: return None
        self._risk_dirty = False; return self._latest_risk

    def consume_robot_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(self._tf_parent, self._tf_child, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None
        t = tf.transform.translation; r = tf.transform.rotation
        yaw = float(np.arctan2(2.0*(r.w*r.z + r.x*r.y), 1.0 - 2.0*(r.y*r.y + r.z*r.z)))
        return ((float(t.x), float(t.y), float(t.z)), yaw)

    def shutdown(self) -> None:
        for pub in (self.start_pub, self.goal_pub, self.goal_pose_pub,
                    self.initial_pose_pub, self.external_preblocked_pub,
                    self.edited_occupied_pub, self.occupied_pub,
                    self.preblocked_pub, self.traversable_pub, self.risk_pub):
            pub.unregister()


# ── Main window ────────────────────────────────────────────────────────────────
class MapViewerWindow(QWidget):
    _ROBOT_DISPLAY_Z_OFFSET = -0.3
    _LAYER_STYLE = {
        "occupied":    ((0.95, 0.45, 0.15), 1.0,  "占据"),
        "preblocked":  ((1.0,  0.0,  0.0),  1.0,  "禁行"),
        "traversable": ((0.0,  1.0,  0.0),  0.4,  "可通行"),
        "risk":        ((0.15, 0.35, 1.0),  0.55, "风险代价"),
    }

    def __init__(self) -> None:
        super().__init__()
        default_map_package = Path(os.environ.get("MAP_VIEWER_DEFAULT_PACKAGE", "/home/robot/maps/map")).expanduser()
        self._default_root     = default_map_package.parent
        self._default_map_name = default_map_package.name
        self._suppress_next_load_dialog = False
        self._worker_thread: threading.Thread | None = None
        self._layer_actors: dict = {}
        self._layer_data:   dict = {}
        self._renderer  = vtk.vtkRenderer()
        self._ros_node  = MapViewerRosNode()
        self._frame_id  = "map"
        self._pick_mode: str | None = None
        self._start_actor:           vtk.vtkActor | None = None
        self._goal_actor:            vtk.vtkActor | None = None
        self._goal_arrow_actor:      vtk.vtkActor | None = None
        self._goal_pending_position                      = None
        self._goal_yaw: float = 0.0
        self._current_pose_arrow_actor: vtk.vtkActor | None = None
        self._path_actor:  vtk.vtkActor | None = None
        self._robot_actor: vtk.vtkProp3D | None = None
        self._latest_robot_pose = None
        self._edit_cursor_actor:      vtk.vtkActor | None = None
        self._edit_cursor_edge_actor: vtk.vtkActor | None = None
        self._edit_position   = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self._edit_size_cells = 1
        self._init_ui()
        QApplication.instance().installEventFilter(self)
        self._spin_timer = QTimer(self)
        self._spin_timer.setTimerType(Qt.PreciseTimer)
        self._spin_timer.timeout.connect(self._spin_ros_once)
        self._spin_timer.start(20)
        QTimer.singleShot(0, self._autoload_default_map)

    def _init_ui(self) -> None:
        self.setWindowTitle("地图查看")
        self.resize(1220, 820)
        layout = QVBoxLayout()
        control_row = QHBoxLayout()

        # Map group
        map_group  = QGroupBox("地图处理")
        map_layout = QVBoxLayout()
        map_root_row = QHBoxLayout()
        self.path_edit = QLineEdit(str(self._default_root))
        self.path_edit.setPlaceholderText("选择地图根目录")
        choose_root_btn = QPushButton("选择文件夹")
        choose_root_btn.clicked.connect(self._choose_root_directory)
        map_root_row.addWidget(QLabel("根目录")); map_root_row.addWidget(self.path_edit, 1); map_root_row.addWidget(choose_root_btn)
        map_name_row = QHBoxLayout()
        self.name_edit = QLineEdit(self._default_map_name)
        self.name_edit.setPlaceholderText("请输入地图名，例如 lv2")
        self.overwrite_checkbox = QCheckBox("允许覆盖"); self.overwrite_checkbox.setChecked(True)
        map_name_row.addWidget(QLabel("地图名")); map_name_row.addWidget(self.name_edit, 1); map_name_row.addWidget(self.overwrite_checkbox)
        map_button_row = QHBoxLayout()
        open_btn = QPushButton("打开地图"); open_btn.clicked.connect(self._choose_and_open)
        refresh_btn = QPushButton("刷新地图"); refresh_btn.clicked.connect(self._refresh_map_from_edited_occupied)
        save_btn = QPushButton("保存地图"); save_btn.clicked.connect(self._start_save)
        map_button_row.addWidget(open_btn); map_button_row.addWidget(refresh_btn); map_button_row.addWidget(save_btn)
        map_layout.addLayout(map_root_row); map_layout.addLayout(map_name_row); map_layout.addLayout(map_button_row)
        map_group.setLayout(map_layout)

        # Planning group
        planning_group  = QGroupBox("路径规划")
        planning_layout = QHBoxLayout()
        self.start_btn = QPushButton("起始点"); self.start_btn.clicked.connect(lambda: self._set_pick_mode("start"))
        self.goal_btn  = QPushButton("目标点"); self.goal_btn.clicked.connect(lambda: self._set_pick_mode("goal"))
        planning_layout.addWidget(self.start_btn); planning_layout.addWidget(self.goal_btn); planning_layout.addStretch(1)
        planning_group.setLayout(planning_layout)

        # Navigation group
        nav_group  = QGroupBox("导航")
        nav_layout = QHBoxLayout()
        self.current_pose_btn = QPushButton("当前姿态"); self.current_pose_btn.clicked.connect(lambda: self._set_pick_mode("current_pose"))
        self.navigate_btn     = QPushButton("导航目标"); self.navigate_btn.clicked.connect(lambda: self._set_pick_mode("navigate"))
        nav_layout.addWidget(self.current_pose_btn); nav_layout.addWidget(self.navigate_btn); nav_layout.addStretch(1)
        nav_group.setLayout(nav_layout)

        # Display group
        display_group = QGroupBox("地图显示选项")
        layer_row     = QHBoxLayout()
        self.occupied_checkbox    = QCheckBox("占据");    self.occupied_checkbox.setChecked(True)
        self.preblocked_checkbox  = QCheckBox("禁行");    self.preblocked_checkbox.setChecked(False)
        self.traversable_checkbox = QCheckBox("可通行");  self.traversable_checkbox.setChecked(False)
        self.risk_checkbox        = QCheckBox("风险代价"); self.risk_checkbox.setChecked(False)
        for cb in (self.occupied_checkbox, self.preblocked_checkbox, self.traversable_checkbox, self.risk_checkbox):
            cb.toggled.connect(self._refresh_layers); layer_row.addWidget(cb)
        layer_row.addStretch(1); display_group.setLayout(layer_row)

        # Edit group
        edit_group  = QGroupBox("栅格编辑")
        edit_layout = QVBoxLayout()
        edit_top_row = QHBoxLayout()
        self.edit_checkbox = QCheckBox("编辑栅格"); self.edit_checkbox.toggled.connect(self._toggle_edit_mode)
        self.enlarge_btn = QPushButton("扩大"); self.enlarge_btn.clicked.connect(self._increase_edit_size)
        self.shrink_btn  = QPushButton("缩小"); self.shrink_btn.clicked.connect(self._decrease_edit_size)
        edit_top_row.addWidget(self.edit_checkbox); edit_top_row.addWidget(self.enlarge_btn); edit_top_row.addWidget(self.shrink_btn); edit_top_row.addStretch(1)
        edit_radio_row = QHBoxLayout()
        self.edit_type_group   = QButtonGroup(self)
        self.edit_type_buttons: dict[str, QRadioButton] = {}
        for layer_name in ("occupied", "preblocked", "traversable", "clear"):
            label = self._LAYER_STYLE[layer_name][2] if layer_name in self._LAYER_STYLE else "清空"
            radio = QRadioButton(label)
            if layer_name == "occupied": radio.setChecked(True)
            radio.toggled.connect(self._refocus_view_if_editing)
            self.edit_type_group.addButton(radio)
            self.edit_type_buttons[layer_name] = radio
            edit_radio_row.addWidget(radio)
        edit_radio_row.addStretch(1)
        edit_layout.addLayout(edit_top_row); edit_layout.addLayout(edit_radio_row)
        edit_group.setLayout(edit_layout)

        nav_column = QVBoxLayout()
        nav_column.addWidget(planning_group); nav_column.addWidget(nav_group); nav_column.addStretch(1)
        control_row.addWidget(map_group, 2); control_row.addLayout(nav_column, 1)
        control_row.addWidget(display_group, 2); control_row.addWidget(edit_group, 2)

        self.info_label = QLabel("尚未加载地图。")
        self.info_label.setWordWrap(True)
        self.info_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        self.vtk_widget = QVTKRenderWindowInteractor(self)
        self.vtk_widget.GetRenderWindow().AddRenderer(self._renderer)
        self._renderer.SetBackground(0.04, 0.07, 0.09)
        self._renderer.GradientBackgroundOn(); self._renderer.SetBackground2(0.12, 0.16, 0.19)
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(1.5,1.5,1.5); axes.SetXAxisLabelText(""); axes.SetYAxisLabelText(""); axes.SetZAxisLabelText("")
        self._renderer.AddActor(axes)
        self._renderer.AddActor(self._make_ground_grid(24.0, 1.0))
        interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        interactor.Initialize()
        interactor.AddObserver("LeftButtonPressEvent", self._on_left_button_press, 1.0)
        interactor.AddObserver("MouseMoveEvent",       self._on_mouse_move,        1.0)
        self.setFocusPolicy(Qt.StrongFocus); self.vtk_widget.setFocusPolicy(Qt.StrongFocus)
        self.installEventFilter(self); self.vtk_widget.installEventFilter(self)

        layout.addLayout(control_row); layout.addWidget(self.info_label); layout.addWidget(self.vtk_widget, 1)
        self.setLayout(layout)

    def closeEvent(self, event) -> None:
        self._ros_node.shutdown(); super().closeEvent(event)

    # ── spin / consume ────────────────────────────────────────────────────
    def _spin_ros_once(self) -> None:
        for consume_fn, layer_name in [
            (self._ros_node.consume_occupied,    "occupied"),
            (self._ros_node.consume_preblocked,  "preblocked"),
            (self._ros_node.consume_traversable, "traversable"),
        ]:
            payload = consume_fn()
            if payload is not None:
                points, scale = payload
                color, opacity, _ = self._LAYER_STYLE[layer_name]
                self._layer_data[layer_name] = (points, scale, color, opacity)
                self._refresh_layers()

        risk = self._ros_node.consume_risk()
        if risk is not None:
            self._layer_data["risk"] = risk
            self._refresh_layers()

        robot_pose = self._ros_node.consume_robot_pose()
        if robot_pose is not None:
            self._latest_robot_pose = robot_pose
            self._update_robot_visual(*robot_pose)

        path_points = self._ros_node.consume_path()
        if path_points is not None:
            self._update_path(path_points)

    # ── map open / save ───────────────────────────────────────────────────
    def _choose_root_directory(self) -> None:
        selected = QFileDialog.getExistingDirectory(self, "选择地图根目录",
            self.path_edit.text().strip() or str(self._default_root),
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks)
        if selected: self.path_edit.setText(selected)

    def _autoload_default_map(self) -> None:
        package_path = self._default_root / self._default_map_name
        if not package_path.exists():
            self.info_label.setText(f"默认地图不存在：{package_path}"); return
        self.path_edit.setText(str(self._default_root))
        self.name_edit.setText(self._default_map_name)
        self._suppress_next_load_dialog = True
        self._start_load_for_package(package_path)

    def _build_package_path(self):
        root_dir = self.path_edit.text().strip(); map_name = self.name_edit.text().strip()
        if not root_dir: QMessageBox.warning(self, "地图目录", "请先选择地图根目录。"); return None
        if not map_name: QMessageBox.warning(self, "地图目录", "请输入地图名。");     return None
        return Path(root_dir).expanduser() / map_name

    def _set_busy(self, busy: bool) -> None:
        for w in (self.path_edit, self.name_edit, self.overwrite_checkbox,
                  self.start_btn, self.goal_btn, self.current_pose_btn, self.navigate_btn,
                  self.edit_checkbox, self.enlarge_btn, self.shrink_btn,
                  *self.edit_type_buttons.values()):
            w.setEnabled(not busy)

    def _start_save(self) -> None:
        package_path = self._build_package_path()
        if package_path is None: return
        if not self._publish_edited_occupied_for_cpp_refresh(): return
        self._set_busy(True)
        self.info_label.setText(f"正在保存地图到 {package_path}，请稍候。")
        worker = SaveWorker(str(package_path), self.overwrite_checkbox.isChecked())
        worker.finished.connect(self._on_save_finished)
        thread = threading.Thread(target=worker.run, daemon=True)
        self._worker_thread = thread; self._worker = worker; thread.start()

    def _on_save_finished(self, success: bool, message: str) -> None:
        self._set_busy(False); self.info_label.setText(message)
        if success: QMessageBox.information(self, "保存地图", "地图保存成功。")
        else:       QMessageBox.critical(self, "保存地图", message)

    def _choose_and_open(self) -> None:
        selected = QFileDialog.getExistingDirectory(self, "选择地图目录",
            self.path_edit.text().strip() or str(self._default_root),
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks)
        if not selected: return
        selected_path = Path(selected)
        self.path_edit.setText(str(selected_path.parent)); self.name_edit.setText(selected_path.name)
        self._start_load_for_package(selected_path)

    def _start_load_for_package(self, package_path: Path) -> None:
        self._set_busy(True)
        self.info_label.setText(f"正在加载地图 {package_path}，请稍候。")
        worker = LoadWorker(str(package_path))
        worker.finished.connect(self._on_load_finished)
        thread = threading.Thread(target=worker.run, daemon=True)
        self._worker_thread = thread; self._worker = worker; thread.start()

    def _on_load_finished(self, success: bool, message: str, package_path: str) -> None:
        self._set_busy(False)
        suppress = self._suppress_next_load_dialog; self._suppress_next_load_dialog = False
        if success:
            self._load_map_package(Path(package_path))
            if not suppress: QMessageBox.information(self, "加载地图", "地图加载成功。")
        else:
            self.info_label.setText(message)
            if not suppress: QMessageBox.critical(self, "加载地图", message)

    def _load_map_package(self, package_dir: Path) -> None:
        meta_path   = package_dir / "meta.yaml"
        layers_path = package_dir / "layers.npz"
        if not meta_path.exists() or not layers_path.exists():
            QMessageBox.critical(self, "打开地图", "目录中缺少 meta.yaml 或 layers.npz。"); return
        try:
            with meta_path.open("r", encoding="utf-8") as f:
                meta = yaml.safe_load(f)
            layers = np.load(layers_path, allow_pickle=False)
        except Exception as exc:
            QMessageBox.critical(self, "打开地图", f"读取地图失败：{exc}"); return

        occupied_layer = self._layer_data.get("occupied")
        self._layer_data.clear()
        for layer_name in ("occupied", "preblocked", "traversable"):
            pts_key = f"{layer_name}_points"; sc_key = f"{layer_name}_scale"
            if pts_key in layers:
                color, opacity, _ = self._LAYER_STYLE[layer_name]
                self._layer_data[layer_name] = (layers[pts_key], layers[sc_key], color, opacity)
        if "occupied" not in self._layer_data and occupied_layer is not None:
            self._layer_data["occupied"] = occupied_layer
        if "risk_points" in layers and "risk_intensity" in layers:
            risk_scale = (np.asarray(layers["traversable_scale"], dtype=np.float32)
                         if "traversable_scale" in layers
                         else np.array([0.2,0.2,0.2], dtype=np.float32))
            self._layer_data["risk"] = (
                np.asarray(layers["risk_points"],     dtype=np.float32),
                risk_scale,
                np.asarray(layers["risk_intensity"],  dtype=np.float32),
            )
        if not self._layer_data:
            QMessageBox.critical(self, "打开地图", "地图文件中没有可显示的体素层。"); return

        self._refresh_layers(reset_camera=True)
        self._sync_external_preblocked()
        if self.edit_checkbox.isChecked():
            self._initialize_edit_position(); self._update_edit_cursor()
        map_id     = meta.get("map_id", "")
        resolution = meta.get("resolution", 0.0)
        self._frame_id = meta.get("frame_id", "map")
        occupied_count = len(self._layer_data.get("occupied", (np.empty((0,3)),))[0])
        self.info_label.setText(f"地图：{map_id}    分辨率：{resolution:.2f} 米    占据体素：{occupied_count}")

    # ── pick / navigation ─────────────────────────────────────────────────
    def _set_pick_mode(self, mode: str) -> None:
        if mode == "navigate" and self._latest_robot_pose is None:
            self.info_label.setText("未收到机器人 TF，无法设置导航目标。"); return
        self._pick_mode = mode; self._goal_pending_position = None
        if self.edit_checkbox.isChecked(): self.edit_checkbox.setChecked(False)
        msgs = {"start": "点击 3D 视图设置起始点。",
                "current_pose": "点击 3D 视图设置当前姿态位置，再点击一次设置朝向。",
                "navigate": "点击 3D 视图设置导航目标位置，再点击一次设置目标朝向。",
                "goal": "点击 3D 视图设置目标点位置，再点击一次设置目标朝向。"}
        self.info_label.setText(msgs.get(mode, ""))

    def _on_left_button_press(self, obj, _event) -> None:
        if self._pick_mode is None: return
        actor_list = [a for a, _ in self._layer_actors.values()]
        if not actor_list: return
        cx, cy = obj.GetEventPosition()
        picker = vtk.vtkPropPicker(); picker.PickFromListOn()
        for a in actor_list: picker.AddPickList(a)
        if picker.Pick(cx, cy, 0, self._renderer) == 0:
            self.info_label.setText("没有选中栅格。"); return
        self.vtk_widget.setFocus()
        pos = picker.GetPickPosition()
        picked_xyz = self._snap_pick((float(pos[0]), float(pos[1]), float(pos[2])))
        mode = self._pick_mode

        if mode == "start":
            self._pick_mode = None
            self._ros_node.publish_point("start", self._frame_id, picked_xyz)
            self._update_point_actor("start", picked_xyz)
            self.info_label.setText(f"起始点已设置：[{picked_xyz[0]:.2f}, {picked_xyz[1]:.2f}, {picked_xyz[2]:.2f}]")
        elif mode in ("goal", "current_pose", "navigate"):
            self._goal_pending_position = picked_xyz
            self._pick_mode = mode + "_heading"
            if mode == "current_pose": self._update_current_pose_visual(picked_xyz, self._goal_yaw)
            else:                      self._update_goal_visual(picked_xyz, self._goal_yaw)
            self.info_label.setText("位置已设置。移动鼠标预览朝向，再点击一次确认。")
        elif mode == "goal_heading" and self._goal_pending_position is not None:
            yaw = self._compute_yaw(self._goal_pending_position, picked_xyz)
            self._goal_yaw = yaw; self._pick_mode = None
            goal_xyz = self._goal_pending_position; self._goal_pending_position = None
            self._ros_node.publish_point("goal", self._frame_id, goal_xyz)
            self._ros_node.publish_goal_pose(self._frame_id, goal_xyz, yaw)
            self._update_goal_visual(goal_xyz, yaw)
            self.info_label.setText(f"目标点已设置：[{goal_xyz[0]:.2f}, {goal_xyz[1]:.2f}, {goal_xyz[2]:.2f}]，朝向 {np.degrees(yaw):.1f}°")
        elif mode == "current_pose_heading" and self._goal_pending_position is not None:
            yaw = self._compute_yaw(self._goal_pending_position, picked_xyz)
            self._goal_yaw = yaw; self._pick_mode = None
            pose_xyz = self._goal_pending_position; self._goal_pending_position = None
            self._ros_node.publish_initial_pose(self._frame_id, pose_xyz, yaw)
            self._clear_current_pose_visual(); self._update_robot_visual(pose_xyz, yaw)
            self.info_label.setText(f"当前姿态已设置：[{pose_xyz[0]:.2f}, {pose_xyz[1]:.2f}, {pose_xyz[2]:.2f}]，朝向 {np.degrees(yaw):.1f}°")
        elif mode == "navigate_heading" and self._goal_pending_position is not None:
            if self._latest_robot_pose is None:
                self._pick_mode = None; self._goal_pending_position = None
                self.info_label.setText("未收到机器人 TF，导航起点不可用。"); return
            yaw = self._compute_yaw(self._goal_pending_position, picked_xyz)
            self._goal_yaw = yaw; self._pick_mode = None
            goal_xyz = self._goal_pending_position; self._goal_pending_position = None
            start_xyz, _ = self._latest_robot_pose
            self._ros_node.publish_point("start", self._frame_id, start_xyz)
            self._ros_node.publish_point("goal",  self._frame_id, goal_xyz)
            self._ros_node.publish_goal_pose(self._frame_id, goal_xyz, yaw)
            self._update_point_actor("start", start_xyz); self._update_goal_visual(goal_xyz, yaw)
            self.info_label.setText(f"导航目标已设置：[{goal_xyz[0]:.2f}, {goal_xyz[1]:.2f}, {goal_xyz[2]:.2f}]，朝向 {np.degrees(yaw):.1f}°")

    def _on_mouse_move(self, obj, _event) -> None:
        if self._pick_mode not in ("goal_heading", "navigate_heading", "current_pose_heading") \
                or self._goal_pending_position is None: return
        mx, my = obj.GetEventPosition()
        cursor_xyz = self._pick_on_height_plane(mx, my, float(self._goal_pending_position[2]))
        if cursor_xyz is None: return
        self._goal_yaw = self._compute_yaw(self._goal_pending_position, cursor_xyz)
        if self._pick_mode == "current_pose_heading":
            self._update_current_pose_visual(self._goal_pending_position, self._goal_yaw)
        else:
            self._update_goal_visual(self._goal_pending_position, self._goal_yaw)

    def _pick_on_height_plane(self, dx: int, dy: int, plane_z: float):
        self._renderer.SetDisplayPoint(float(dx), float(dy), 0.0); self._renderer.DisplayToWorld()
        nw = self._renderer.GetWorldPoint()
        self._renderer.SetDisplayPoint(float(dx), float(dy), 1.0); self._renderer.DisplayToWorld()
        fw = self._renderer.GetWorldPoint()
        if abs(nw[3]) < 1e-9 or abs(fw[3]) < 1e-9: return None
        p0 = np.array([nw[0]/nw[3], nw[1]/nw[3], nw[2]/nw[3]])
        p1 = np.array([fw[0]/fw[3], fw[1]/fw[3], fw[2]/fw[3]])
        d = p1 - p0
        if abs(d[2]) < 1e-9: return None
        t = (plane_z - p0[2]) / d[2]; i = p0 + d * t
        return (float(i[0]), float(i[1]), float(plane_z))

    def _compute_yaw(self, origin, target) -> float:
        dx = float(target[0]-origin[0]); dy = float(target[1]-origin[1])
        if abs(dx) < 1e-6 and abs(dy) < 1e-6: return self._goal_yaw
        return float(np.arctan2(dy, dx))

    def _snap_pick(self, xyz):
        layer = self._layer_data.get("traversable")
        if layer is None: return xyz
        points = layer[0]
        if points.size == 0: return xyz
        diffs = points - np.asarray(xyz, dtype=np.float32)
        nearest = int(np.argmin(np.einsum("ij,ij->i", diffs, diffs)))
        s = points[nearest]; return (float(s[0]), float(s[1]), float(s[2]))

    # ── edit mode ─────────────────────────────────────────────────────────
    def eventFilter(self, obj, event):
        if event.type() == QEvent.KeyPress and self.edit_checkbox.isChecked():
            if self._handle_edit_key(event): return True
        return super().eventFilter(obj, event)

    def _handle_edit_key(self, event) -> bool:
        key = event.key()
        move_map = {Qt.Key_W: np.array([1,0,0],dtype=np.float32), Qt.Key_S: np.array([-1,0,0],dtype=np.float32),
                    Qt.Key_A: np.array([0,1,0],dtype=np.float32), Qt.Key_D: np.array([0,-1,0],dtype=np.float32),
                    Qt.Key_Q: np.array([0,0,-1],dtype=np.float32),Qt.Key_E: np.array([0,0,1],dtype=np.float32)}
        if key in move_map:
            self._edit_position += move_map[key] * self._get_edit_scale()
            self._update_edit_cursor()
            self.info_label.setText(f"编辑栅格位置：[{self._edit_position[0]:.2f}, {self._edit_position[1]:.2f}, {self._edit_position[2]:.2f}]")
            return True
        if key == Qt.Key_Space: self._place_edit_voxel(); return True
        return False

    def _refocus_view_if_editing(self, checked=False) -> None:
        if self.edit_checkbox.isChecked(): self.activateWindow(); self.vtk_widget.setFocus()

    def _increase_edit_size(self) -> None:
        self._edit_size_cells += 1
        if self.edit_checkbox.isChecked(): self._update_edit_cursor(); self._refocus_view_if_editing()
        self.info_label.setText(f"编辑栅格尺寸：{self._edit_size_cells}x{self._edit_size_cells}x{self._edit_size_cells}")

    def _decrease_edit_size(self) -> None:
        self._edit_size_cells = max(1, self._edit_size_cells - 1)
        if self.edit_checkbox.isChecked(): self._update_edit_cursor(); self._refocus_view_if_editing()
        self.info_label.setText(f"编辑栅格尺寸：{self._edit_size_cells}x{self._edit_size_cells}x{self._edit_size_cells}")

    def _toggle_edit_mode(self, checked: bool) -> None:
        if checked:
            self._pick_mode = None
            self._initialize_edit_position(); self._update_edit_cursor(); self._refocus_view_if_editing()
            self.info_label.setText(f"编辑栅格已开启。当前尺寸 {self._edit_size_cells}x{self._edit_size_cells}x{self._edit_size_cells}。W/S X，A/D Y，Q/E Z，空格生成。")
        else:
            self._remove_edit_cursor(); self.vtk_widget.GetRenderWindow().Render()

    def _initialize_edit_position(self) -> None:
        occ = self._layer_data.get("occupied")
        if occ is not None and occ[0].size > 0:
            pts = occ[0]; sc = np.asarray(occ[1], dtype=np.float32)
            self._edit_position = np.array([(pts[:,0].min()+pts[:,0].max())*0.5,
                                             (pts[:,1].min()+pts[:,1].max())*0.5,
                                              pts[:,2].max() + 2.0*sc[2]], dtype=np.float32); return
        for ln in ("traversable","preblocked"):
            l = self._layer_data.get(ln)
            if l is not None and l[0].size > 0: self._edit_position = l[0][0].astype(np.float32).copy(); return
        self._edit_position = np.zeros(3, dtype=np.float32)

    def _get_edit_scale(self) -> np.ndarray:
        for ln in ("occupied","traversable","preblocked"):
            l = self._layer_data.get(ln)
            if l is not None: return np.asarray(l[1], dtype=np.float32)
        return np.array([0.2,0.2,0.2], dtype=np.float32)

    def _remove_edit_cursor(self) -> None:
        for attr in ("_edit_cursor_actor","_edit_cursor_edge_actor"):
            a = getattr(self, attr)
            if a is not None: self._renderer.RemoveActor(a); setattr(self, attr, None)

    def _update_edit_cursor(self) -> None:
        self._remove_edit_cursor()
        scale = self._get_edit_scale() * float(self._edit_size_cells)
        actor, edge_actor = self._build_voxel_actors(np.asarray([self._edit_position],dtype=np.float32), scale, (1.0,0.15,0.15), 0.45)
        edge_actor.GetProperty().SetColor(0.55,0.0,0.0); edge_actor.GetProperty().SetLineWidth(2.0)
        self._renderer.AddActor(actor); self._renderer.AddActor(edge_actor)
        self._edit_cursor_actor = actor; self._edit_cursor_edge_actor = edge_actor
        self.vtk_widget.GetRenderWindow().Render()

    def _selected_edit_layer(self) -> str:
        for ln, radio in self.edit_type_buttons.items():
            if radio.isChecked(): return ln
        return "occupied"

    def _edit_block_points(self, scale: np.ndarray) -> np.ndarray:
        half = (self._edit_size_cells - 1) / 2.0
        offsets = np.arange(self._edit_size_cells, dtype=np.float32) - half
        pts = [self._edit_position + np.array([ox*scale[0], oy*scale[1], oz*scale[2]], dtype=np.float32)
               for ox in offsets for oy in offsets for oz in offsets]
        return np.asarray(pts, dtype=np.float32)

    def _place_edit_voxel(self) -> None:
        layer_name = self._selected_edit_layer()
        if layer_name == "clear": self._clear_edit_voxel(); return
        scale = self._get_edit_scale()
        color, opacity, label = self._LAYER_STYLE[layer_name]
        points = self._edit_block_points(scale)
        if layer_name in self._layer_data:
            current_pts, current_scale, _, _ = self._layer_data[layer_name]
            merged = current_pts; added = 0
            for pt in points:
                if merged.size > 0:
                    if np.any(np.einsum("ij,ij->i", merged - pt, merged - pt) < 1e-8): continue
                merged = np.vstack([merged, pt]).astype(np.float32); added += 1
            if added == 0: self.info_label.setText(f"{label}栅格已存在。"); return
            points = merged; scale = np.asarray(current_scale, dtype=np.float32)
        self._layer_data[layer_name] = (points.astype(np.float32), scale, color, opacity)
        {"occupied": self.occupied_checkbox, "preblocked": self.preblocked_checkbox,
         "traversable": self.traversable_checkbox}[layer_name].setChecked(True)
        self._refresh_layers()
        if self.edit_checkbox.isChecked(): self._update_edit_cursor()
        if layer_name == "preblocked": self._sync_external_preblocked()
        self.info_label.setText(f"已生成{label}栅格块：中心[{self._edit_position[0]:.2f},{self._edit_position[1]:.2f},{self._edit_position[2]:.2f}]")

    def _clear_edit_voxel(self) -> None:
        scale = self._get_edit_scale()
        half = np.asarray(scale, dtype=np.float32) * float(self._edit_size_cells) * 0.5
        lo = self._edit_position - half; hi = self._edit_position + half
        eps = np.maximum(scale * 1e-3, 1e-6); changed = []
        for ln, layer in list(self._layer_data.items()):
            pts = np.asarray(layer[0], dtype=np.float32)
            if pts.size == 0: continue
            inside = np.all((pts >= lo-eps) & (pts <= hi+eps), axis=1)
            if not np.any(inside): continue
            keep = ~inside
            if ln == "risk":
                self._layer_data[ln] = (pts[keep], layer[1], np.asarray(layer[2],dtype=np.float32)[keep])
            else:
                self._layer_data[ln] = (pts[keep], layer[1], layer[2], layer[3])
            changed.append(ln)
        if not changed: self.info_label.setText("当前位置没有可清除的栅格。"); return
        self._refresh_layers()
        if self.edit_checkbox.isChecked(): self._update_edit_cursor()
        self.vtk_widget.GetRenderWindow().Render()
        if "preblocked" in changed: self._sync_external_preblocked()
        self.info_label.setText(f"已清空栅格块：中心[{self._edit_position[0]:.2f},{self._edit_position[1]:.2f},{self._edit_position[2]:.2f}]")

    def _sync_external_preblocked(self) -> None:
        layer = self._layer_data.get("preblocked")
        pts   = layer[0] if layer is not None else np.empty((0,3),dtype=np.float32)
        scale = layer[1] if layer is not None else self._get_edit_scale()
        self._ros_node.publish_external_preblocked(self._frame_id, pts, scale)

    def _refresh_map_from_edited_occupied(self) -> None:
        if not self._publish_edited_occupied_for_cpp_refresh(): return
        self._refresh_layers()
        if self.edit_checkbox.isChecked(): self._update_edit_cursor()
        self.vtk_widget.GetRenderWindow().Render()

    def _publish_edited_occupied_for_cpp_refresh(self) -> bool:
        occ = self._layer_data.get("occupied")
        if occ is None or occ[0].size == 0:
            self.info_label.setText("没有占据栅格，无法刷新地图。"); return False
        pts = np.asarray(occ[0], dtype=np.float32); scale = np.asarray(occ[1], dtype=np.float32)
        if np.any(scale <= 0): self.info_label.setText("占据栅格分辨率无效。"); return False
        color, opacity, _ = self._LAYER_STYLE["occupied"]
        self._ros_node.publish_voxel_marker("occupied", self._frame_id, pts, scale, color, opacity)
        self._ros_node.publish_edited_occupied(self._frame_id, pts, scale)
        self.info_label.setText(f"已发送编辑后占据栅格给 jie_path_node。占据栅格数：{len(pts)}。")
        return True

    # ── VTK visual helpers ────────────────────────────────────────────────
    def _make_ground_grid(self, size: float, step: float) -> vtk.vtkActor:
        append = vtk.vtkAppendPolyData(); half = int(size/step)
        for i in range(-half, half+1):
            for pt1, pt2 in [((-size,i*step,0),(size,i*step,0)), ((i*step,-size,0),(i*step,size,0))]:
                line = vtk.vtkLineSource(); line.SetPoint1(*pt1); line.SetPoint2(*pt2)
                append.AddInputConnection(line.GetOutputPort())
        mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(append.GetOutputPort())
        actor = vtk.vtkActor(); actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.24,0.30,0.34); actor.GetProperty().SetLineWidth(1.0); actor.GetProperty().SetOpacity(0.65)
        return actor

    def _build_voxel_actors(self, points, scale, color, opacity):
        vtk_pts = vtk.vtkPoints(); vtk_pts.SetData(numpy_support.numpy_to_vtk(points.astype(np.float32), deep=True))
        pd = vtk.vtkPolyData(); pd.SetPoints(vtk_pts)
        cube = vtk.vtkCubeSource(); cube.SetXLength(float(scale[0])); cube.SetYLength(float(scale[1])); cube.SetZLength(float(scale[2]))
        glyph = vtk.vtkGlyph3DMapper(); glyph.SetInputData(pd); glyph.SetSourceConnection(cube.GetOutputPort()); glyph.ScalingOff()
        actor = vtk.vtkActor(); actor.SetMapper(glyph)
        actor.GetProperty().SetColor(*color); actor.GetProperty().SetOpacity(opacity); actor.GetProperty().SetInterpolationToFlat()
        edge_cube = vtk.vtkCubeSource(); edge_cube.SetXLength(float(scale[0])); edge_cube.SetYLength(float(scale[1])); edge_cube.SetZLength(float(scale[2]))
        edge_ext = vtk.vtkExtractEdges(); edge_ext.SetInputConnection(edge_cube.GetOutputPort())
        edge_glyph = vtk.vtkGlyph3DMapper(); edge_glyph.SetInputData(pd); edge_glyph.SetSourceConnection(edge_ext.GetOutputPort()); edge_glyph.ScalingOff()
        edge_actor = vtk.vtkActor(); edge_actor.SetMapper(edge_glyph)
        edge_actor.GetProperty().SetColor(0,0,0); edge_actor.GetProperty().SetLineWidth(1.0); edge_actor.GetProperty().SetOpacity(1.0)
        return actor, edge_actor

    def _build_risk_actors(self, points, scale, intensity):
        vtk_pts = vtk.vtkPoints(); vtk_pts.SetData(numpy_support.numpy_to_vtk(points.astype(np.float32), deep=True))
        pd = vtk.vtkPolyData(); pd.SetPoints(vtk_pts)
        alphas = np.clip(0.12 + 0.83*intensity.astype(np.float32), 0.12, 0.95)
        colors = np.zeros((len(points),4), dtype=np.uint8)
        colors[:,0]=int(0.15*255); colors[:,1]=int(0.35*255); colors[:,2]=255
        colors[:,3]=np.round(alphas*255).astype(np.uint8)
        vtk_colors = numpy_support.numpy_to_vtk(colors, deep=True, array_type=vtk.VTK_UNSIGNED_CHAR)
        vtk_colors.SetName("risk_rgba"); pd.GetPointData().SetScalars(vtk_colors)
        cube = vtk.vtkCubeSource(); cube.SetXLength(float(scale[0])); cube.SetYLength(float(scale[1])); cube.SetZLength(float(scale[2]))
        glyph = vtk.vtkGlyph3DMapper(); glyph.SetInputData(pd); glyph.SetSourceConnection(cube.GetOutputPort()); glyph.ScalingOff()
        glyph.SetScalarModeToUsePointData(); glyph.ScalarVisibilityOn(); glyph.SetColorModeToDirectScalars()
        actor = vtk.vtkActor(); actor.SetMapper(glyph); actor.GetProperty().SetOpacity(1.0); actor.GetProperty().SetInterpolationToFlat()
        return actor, None

    def _refresh_layers(self, checked=None, reset_camera=False) -> None:
        for actor, edge_actor in self._layer_actors.values():
            self._renderer.RemoveActor(actor)
            if edge_actor is not None: self._renderer.RemoveActor(edge_actor)
        self._layer_actors.clear()
        visibility = {"occupied": self.occupied_checkbox.isChecked(), "preblocked": self.preblocked_checkbox.isChecked(),
                      "traversable": self.traversable_checkbox.isChecked(), "risk": self.risk_checkbox.isChecked()}
        for ln, visible in visibility.items():
            if not visible or ln not in self._layer_data: continue
            if ln == "risk":
                points, scale, intensity = self._layer_data[ln]
                if points.size == 0: continue
                actor, edge_actor = self._build_risk_actors(points, scale, intensity)
            else:
                points, scale, color, opacity = self._layer_data[ln]
                if points.size == 0: continue
                actor, edge_actor = self._build_voxel_actors(points, scale, color, opacity)
            self._renderer.AddActor(actor)
            if edge_actor is not None: self._renderer.AddActor(edge_actor)
            self._layer_actors[ln] = (actor, edge_actor)
        if reset_camera: self._renderer.ResetCamera()
        self.vtk_widget.GetRenderWindow().Render()

    def _update_point_actor(self, kind: str, xyz) -> None:
        old = self._start_actor if kind == "start" else self._goal_actor
        if old is not None: self._renderer.RemoveActor(old)
        sphere = vtk.vtkSphereSource(); sphere.SetCenter(*xyz); sphere.SetRadius(0.16)
        sphere.SetThetaResolution(18); sphere.SetPhiResolution(18)
        mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(sphere.GetOutputPort())
        actor = vtk.vtkActor(); actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.1,0.95,0.1) if kind=="start" else actor.GetProperty().SetColor(0.95,0.1,0.1)
        actor.GetProperty().SetOpacity(1.0)
        if kind == "start": self._start_actor = actor
        else:               self._goal_actor  = actor
        self._renderer.AddActor(actor); self.vtk_widget.GetRenderWindow().Render()

    def _make_arrow_actor(self, xyz, yaw, color):
        arrow = vtk.vtkArrowSource(); arrow.SetTipResolution(24); arrow.SetShaftResolution(24)
        arrow.SetTipLength(0.30); arrow.SetTipRadius(0.18); arrow.SetShaftRadius(0.08)
        tf = vtk.vtkTransform(); tf.PostMultiply()
        tf.Scale(0.90,0.90,0.90); tf.RotateZ(float(np.degrees(yaw))); tf.Translate(float(xyz[0]),float(xyz[1]),float(xyz[2]))
        tff = vtk.vtkTransformPolyDataFilter(); tff.SetTransform(tf); tff.SetInputConnection(arrow.GetOutputPort())
        mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(tff.GetOutputPort())
        actor = vtk.vtkActor(); actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*color); actor.GetProperty().SetOpacity(0.95)
        return actor

    def _update_goal_visual(self, xyz, yaw) -> None:
        self._update_point_actor("goal", xyz)
        if self._goal_arrow_actor is not None: self._renderer.RemoveActor(self._goal_arrow_actor)
        self._goal_arrow_actor = self._make_arrow_actor(xyz, yaw, (0.95,0.1,0.1))
        self._renderer.AddActor(self._goal_arrow_actor); self.vtk_widget.GetRenderWindow().Render()

    def _update_current_pose_visual(self, xyz, yaw) -> None:
        if self._current_pose_arrow_actor is not None: self._renderer.RemoveActor(self._current_pose_arrow_actor)
        self._current_pose_arrow_actor = self._make_arrow_actor(xyz, yaw, (0.10,0.95,0.10))
        self._renderer.AddActor(self._current_pose_arrow_actor); self.vtk_widget.GetRenderWindow().Render()

    def _clear_current_pose_visual(self) -> None:
        if self._current_pose_arrow_actor is not None:
            self._renderer.RemoveActor(self._current_pose_arrow_actor); self._current_pose_arrow_actor = None
            self.vtk_widget.GetRenderWindow().Render()

    def _update_robot_visual(self, xyz, yaw) -> None:
        display_xyz = (float(xyz[0]), float(xyz[1]), float(xyz[2]) + self._ROBOT_DISPLAY_Z_OFFSET)
        if self._robot_actor is None:
            self._robot_actor = self._build_simple_dog_actor()
            if self._robot_actor is None: return
            self._renderer.AddActor(self._robot_actor)
        self._robot_actor.SetPosition(*display_xyz)
        self._robot_actor.SetOrientation(0.0, 0.0, float(np.degrees(yaw)))
        self.vtk_widget.GetRenderWindow().Render()

    def _build_simple_dog_actor(self):
        # Try to locate d1_description via rospkg
        try:
            import rospkg
            urdf_path = Path(rospkg.RosPack().get_path("d1_description")) / "urdf" / "simple_dog.urdf"
            root = ET.parse(urdf_path).getroot()
        except Exception as exc:
            self.info_label.setText(f"加载 simple_dog.urdf 失败：{exc}"); return None

        materials: dict = {}
        for mat in root.findall("material"):
            name = mat.get("name"); color_tag = mat.find("color")
            if not name or color_tag is None: continue
            rgba_text = color_tag.get("rgba","").strip()
            if not rgba_text: continue
            rgba = tuple(float(v) for v in rgba_text.split())
            if len(rgba) == 4: materials[name] = rgba

        link_visuals: dict = {}
        for link in root.findall("link"):
            name = link.get("name")
            if not name: continue
            visuals = []
            for visual in link.findall("visual"):
                geom = visual.find("geometry"); box = geom.find("box") if geom is not None else None
                if box is None: continue
                size_text = box.get("size","").strip()
                if not size_text: continue
                size = tuple(float(v) for v in size_text.split())
                ot = visual.find("origin")
                xyz = self._parse_xyz(ot.get("xyz","0 0 0") if ot is not None else "0 0 0")
                rpy = self._parse_xyz(ot.get("rpy","0 0 0") if ot is not None else "0 0 0")
                mt = visual.find("material")
                rgba = materials.get(mt.get("name") if mt is not None else "", (0.7,0.7,0.7,1.0))
                visuals.append({"size":size,"xyz":xyz,"rpy":rpy,"rgba":rgba})
            link_visuals[name] = visuals

        children_by_parent: dict = {}
        for joint in root.findall("joint"):
            if joint.get("type") != "fixed": continue
            pt = joint.find("parent"); ct = joint.find("child")
            if pt is None or ct is None: continue
            parent = pt.get("link"); child = ct.get("link")
            if not parent or not child: continue
            ot = joint.find("origin")
            xyz = self._parse_xyz(ot.get("xyz","0 0 0") if ot is not None else "0 0 0")
            rpy = self._parse_xyz(ot.get("rpy","0 0 0") if ot is not None else "0 0 0")
            children_by_parent.setdefault(parent,[]).append((child, self._make_transform(xyz,rpy)))

        assembly = vtk.vtkAssembly()
        def add_link(link_name, parent_tf):
            for v in link_visuals.get(link_name,[]):
                assembly.AddPart(self._build_box_actor(v["size"], v["rgba"], parent_tf @ self._make_transform(v["xyz"],v["rpy"])))
            for child_name, jt in children_by_parent.get(link_name,[]):
                add_link(child_name, parent_tf @ jt)
        add_link("base_link", np.eye(4,dtype=np.float64))
        return assembly

    def _build_box_actor(self, size, rgba, tf_matrix):
        cube = vtk.vtkCubeSource(); cube.SetXLength(float(size[0])); cube.SetYLength(float(size[1])); cube.SetZLength(float(size[2]))
        mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(cube.GetOutputPort())
        actor = vtk.vtkActor(); actor.SetMapper(mapper)
        actor.GetProperty().SetColor(float(rgba[0]),float(rgba[1]),float(rgba[2])); actor.GetProperty().SetOpacity(float(rgba[3]))
        actor.SetUserMatrix(self._to_vtk_matrix(tf_matrix)); return actor

    def _parse_xyz(self, text: str):
        v = [float(x) for x in text.split()]
        return (v[0],v[1],v[2]) if len(v)==3 else (0.0,0.0,0.0)

    def _make_transform(self, xyz, rpy) -> np.ndarray:
        roll,pitch,yaw = rpy
        cx,sx = np.cos(roll),np.sin(roll); cy,sy = np.cos(pitch),np.sin(pitch); cz,sz = np.cos(yaw),np.sin(yaw)
        rx = np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]]); ry = np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]]); rz = np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])
        tf = np.eye(4,dtype=np.float64); tf[:3,:3] = rz@ry@rx; tf[:3,3] = np.asarray(xyz,dtype=np.float64); return tf

    def _to_vtk_matrix(self, matrix: np.ndarray) -> vtk.vtkMatrix4x4:
        m = vtk.vtkMatrix4x4()
        for r in range(4):
            for c in range(4): m.SetElement(r,c,float(matrix[r,c]))
        return m

    def _update_path(self, path_points) -> None:
        if self._path_actor is not None: self._renderer.RemoveActor(self._path_actor); self._path_actor = None
        if len(path_points) < 2: self.vtk_widget.GetRenderWindow().Render(); return
        vtk_pts = vtk.vtkPoints()
        for p in path_points: vtk_pts.InsertNextPoint(float(p[0]),float(p[1]),float(p[2]))
        poly_line = vtk.vtkPolyLine(); poly_line.GetPointIds().SetNumberOfIds(len(path_points))
        for i in range(len(path_points)): poly_line.GetPointIds().SetId(i,i)
        cells = vtk.vtkCellArray(); cells.InsertNextCell(poly_line)
        pd = vtk.vtkPolyData(); pd.SetPoints(vtk_pts); pd.SetLines(cells)
        tube = vtk.vtkTubeFilter(); tube.SetInputData(pd); tube.SetRadius(0.06); tube.SetNumberOfSides(16); tube.CappingOn()
        mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(tube.GetOutputPort())
        actor = vtk.vtkActor(); actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.69,0.40,1.0); actor.GetProperty().SetOpacity(1.0)
        self._renderer.AddActor(actor); self._path_actor = actor
        self.vtk_widget.GetRenderWindow().Render()


def main() -> None:
    rospy.init_node("map_viewer_gui_node")
    app = QApplication([])
    window = MapViewerWindow()
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()