#!/usr/bin/env python3
# world_selector_gui.py  —  ROS 1 port of the original ROS 2 implementation

from __future__ import annotations

import sys
import threading
from pathlib import Path

import numpy as np
import rospy
from jie_map_msgs.srv import SaveNavigationMapPackage, SaveNavigationMapPackageRequest
from PyQt5.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk
from vtk.util import numpy_support


# ── Save worker ────────────────────────────────────────────────────────────────
class SaveWorker(QObject):
    finished = pyqtSignal(bool, str)

    def __init__(self, package_path: str, overwrite: bool) -> None:
        super().__init__()
        self.package_path = package_path
        self.overwrite = overwrite

    def run(self) -> None:
        service_name = "/map_package_manager/save_package"
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
        except rospy.ROSException:
            self.finished.emit(False, f"保存服务 {service_name} 不可用。")
            return
        try:
            proxy = rospy.ServiceProxy(service_name, SaveNavigationMapPackage)
            req = SaveNavigationMapPackageRequest()
            req.package_path = self.package_path
            req.overwrite = self.overwrite
            resp = proxy(req)
            self.finished.emit(bool(resp.success), str(resp.message))
        except rospy.ServiceException as exc:
            self.finished.emit(False, f"保存地图失败：{exc}")


# ── ROS interface (plain class, no Node subclass) ──────────────────────────────
class WorldSelectorRosNode:
    def __init__(self) -> None:
        world_file_cmd_topic = rospy.get_param("~world_file_cmd_topic",    "/world_file_cmd")
        occupied_topic = rospy.get_param("~occupied_marker_topic",   "/octomap_occupied_markers")
        preblocked_topic = rospy.get_param("~preblocked_topic",        "/preblocked_cells_markers")
        traversable_topic = rospy.get_param("~traversable_topic",       "/traversable_cells_markers")
        risk_cost_topic = rospy.get_param("~risk_cost_topic",         "/risk_cost_cells")
        start_topic = rospy.get_param("~start_topic",             "/start_point")
        goal_topic = rospy.get_param("~goal_topic",              "/goal_point")
        goal_pose_topic = rospy.get_param("~goal_pose_topic",         "/goal_pose")
        path_topic = rospy.get_param("~path_topic",              "/planned_path")
        self._initial_world_file = rospy.get_param("~initial_world_file",  "")
        self._initial_world_name = rospy.get_param("~initial_world_name",  "")

        # Publishers (latch=True → transient_local)
        self.world_file_pub = rospy.Publisher(world_file_cmd_topic, String,       queue_size=1, latch=True)
        self.start_pub = rospy.Publisher(start_topic,          PointStamped, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher(goal_topic,           PointStamped, queue_size=1, latch=True)
        self.goal_pose_pub = rospy.Publisher(goal_pose_topic,      PoseStamped,  queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber(occupied_topic,    Marker,      self._on_occupied,    queue_size=1)
        rospy.Subscriber(preblocked_topic,  Marker,      self._on_preblocked,  queue_size=1)
        rospy.Subscriber(traversable_topic, Marker,      self._on_traversable, queue_size=1)
        rospy.Subscriber(risk_cost_topic,   PointCloud2, self._on_risk,        queue_size=1)
        rospy.Subscriber(path_topic,        PathMsg,     self._on_path,        queue_size=1)

        # State
        self._latest_occupied = None
        self._latest_preblocked = None
        self._latest_traversable = None
        self._latest_risk = None
        self._latest_path_points: list = []
        self._layer_dirty = False
        self._path_dirty = False

    def initial_world_file(self) -> str:
        # world_name takes precedence only if world_file is empty
        if self._initial_world_file:
            return self._initial_world_file
        if self._initial_world_name:
            pkg_share = Path(__file__).parent.parent
            candidate = pkg_share / "worlds" / self._initial_world_name
            if candidate.exists():
                return str(candidate)
        return ""

    def publish_world_file(self, world_file: str) -> None:
        msg = String()
        msg.data = world_file
        self.world_file_pub.publish(msg)

    def publish_point(self, topic: str, frame_id: str,
                      xyz: tuple) -> None:
        msg = PointStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = rospy.Time.now()
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        if topic == "start":
            self.start_pub.publish(msg)
        else:
            self.goal_pub.publish(msg)

    def publish_goal_pose(self, frame_id: str, xyz: tuple, yaw: float) -> None:
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        half_yaw = float(yaw) * 0.5
        msg.pose.orientation.z = float(np.sin(half_yaw))
        msg.pose.orientation.w = float(np.cos(half_yaw))
        self.goal_pose_pub.publish(msg)

    # ── subscribers ──────────────────────────────────────────────────────
    def _on_occupied(self, msg: Marker) -> None: self._store_marker("occupied",    msg)
    def _on_preblocked(self, msg: Marker) -> None: self._store_marker("preblocked",  msg)
    def _on_traversable(self, msg: Marker) -> None: self._store_marker("traversable", msg)

    def _on_risk(self, msg: PointCloud2) -> None:
        records = list(pc2.read_points(
            msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        if not records:
            xyz = np.empty((0, 3), dtype=np.float32)
            intensity = np.empty((0,), dtype=np.float32)
        else:
            arr = np.array([[r[0], r[1], r[2], r[3]] for r in records], dtype=np.float32)
            xyz = arr[:, :3]
            intensity = arr[:, 3]
        scale = self._infer_voxel_scale()
        self._latest_risk = (xyz, scale, intensity)
        self._layer_dirty = True

    def _on_path(self, msg: PathMsg) -> None:
        self._latest_path_points = [
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            for pose in msg.poses
        ]
        self._path_dirty = True

    def _store_marker(self, layer_name: str, msg: Marker) -> None:
        if msg.type != Marker.CUBE_LIST:
            return
        points = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float32)
        scale = np.array([msg.scale.x, msg.scale.y, msg.scale.z], dtype=np.float32)
        setattr(self, f"_latest_{layer_name}", (points, scale))
        self._layer_dirty = True

    def _infer_voxel_scale(self) -> np.ndarray:
        for payload in (self._latest_occupied,
                        self._latest_preblocked,
                        self._latest_traversable):
            if payload is not None:
                return np.asarray(payload[1], dtype=np.float32)
        return np.array([0.2, 0.2, 0.2], dtype=np.float32)

    def consume_layers(self):
        if not self._layer_dirty:
            return None
        self._layer_dirty = False
        return {
            "occupied":    self._latest_occupied,
            "preblocked":  self._latest_preblocked,
            "traversable": self._latest_traversable,
            "risk":        self._latest_risk,
        }

    def consume_path(self):
        if not self._path_dirty:
            return None
        self._path_dirty = False
        return list(self._latest_path_points)

    def shutdown(self) -> None:
        self.world_file_pub.unregister()
        self.start_pub.unregister()
        self.goal_pub.unregister()
        self.goal_pose_pub.unregister()


# ── Main window ────────────────────────────────────────────────────────────────
class WorldSelectorWindow(QWidget):
    _LAYER_STYLE = {
        "occupied":    ((0.95, 0.45, 0.15), 1.0),
        "preblocked":  ((1.0,  0.0,  0.0),  1.0),
        "traversable": ((0.0,  1.0,  0.0),  0.8),
    }

    def __init__(self) -> None:
        super().__init__()
        self._ros_node = WorldSelectorRosNode()
        self._default_root = Path("/home/robot/maps")
        self._worker_thread: threading.Thread | None = None
        self._renderer = vtk.vtkRenderer()
        self._layer_actors: dict = {}
        self._layer_data:   dict = {}
        self._camera_initialized = False
        self._frame_id = "map"
        self._pick_mode: str | None = None
        self._start_actor:      vtk.vtkActor | None = None
        self._goal_actor:       vtk.vtkActor | None = None
        self._goal_arrow_actor: vtk.vtkActor | None = None
        self._goal_pending_position = None
        self._goal_yaw = 0.0
        self._path_actor: vtk.vtkActor | None = None
        self._init_ui()

        # ROS1: poll dirty flags via QTimer (rospy delivers callbacks in threads)
        self._spin_timer = QTimer(self)
        self._spin_timer.timeout.connect(self._spin_ros_once)
        self._spin_timer.start(100)

        # Auto-load initial world if provided
        initial = self._ros_node.initial_world_file()
        if initial:
            self.path_edit.setText(initial)
            if not self.save_name_edit.text().strip():
                self.save_name_edit.setText(Path(initial).stem)
            self.status_label.setText(
                f"初始 world：{Path(initial).name}。后端正在自动转换 OctoMap。")

    def _init_ui(self) -> None:
        self.setWindowTitle("World Selector - OctoMap 3D")
        self.resize(1180, 760)
        root = QVBoxLayout()
        top_row = QHBoxLayout()

        # ── Left panel ────────────────────────────────────────────────────
        left_panel = QVBoxLayout()

        world_group = QGroupBox("World 文件")
        world_form = QFormLayout()
        world_row = QHBoxLayout()
        self.path_edit = QLineEdit()
        self.path_edit.setPlaceholderText("选择 .world 或 .sdf 文件")
        browse_btn = QPushButton("浏览")
        browse_btn.clicked.connect(self._browse_world)
        world_row.addWidget(self.path_edit, 1)
        world_row.addWidget(browse_btn)
        world_form.addRow("文件路径", world_row)
        load_btn = QPushButton("加载 World")
        load_btn.clicked.connect(self._load_world)
        world_form.addRow("", load_btn)
        world_group.setLayout(world_form)

        save_group = QGroupBox("OctoMap 地图保存")
        save_form = QFormLayout()
        save_root_row = QHBoxLayout()
        self.save_root_edit = QLineEdit(str(self._default_root))
        save_root_btn = QPushButton("选择目录")
        save_root_btn.clicked.connect(self._choose_save_root)
        save_root_row.addWidget(self.save_root_edit, 1)
        save_root_row.addWidget(save_root_btn)
        save_form.addRow("根目录", save_root_row)
        self.save_name_edit = QLineEdit()
        self.save_name_edit.setPlaceholderText("请输入地图名，例如 lv2_map")
        save_form.addRow("地图名", self.save_name_edit)
        self.overwrite_checkbox = QCheckBox("允许覆盖")
        self.overwrite_checkbox.setChecked(True)
        save_form.addRow("", self.overwrite_checkbox)
        save_btn = QPushButton("保存 OctoMap 地图")
        save_btn.clicked.connect(self._start_save)
        save_form.addRow("", save_btn)
        save_group.setLayout(save_form)

        nav_group = QGroupBox("导航设置")
        nav_row = QHBoxLayout()
        self.start_btn = QPushButton("起始点")
        self.start_btn.clicked.connect(lambda: self._set_pick_mode("start"))
        self.goal_btn = QPushButton("目标点")
        self.goal_btn.clicked.connect(lambda: self._set_pick_mode("goal"))
        nav_row.addWidget(self.start_btn)
        nav_row.addWidget(self.goal_btn)
        nav_row.addStretch(1)
        nav_group.setLayout(nav_row)

        display_group = QGroupBox("显示图层")
        display_row = QHBoxLayout()
        self.occupied_checkbox = QCheckBox("占据")
        self.occupied_checkbox.setChecked(True)
        self.preblocked_checkbox = QCheckBox("禁行")
        self.preblocked_checkbox.setChecked(True)
        self.traversable_checkbox = QCheckBox("可通行")
        self.traversable_checkbox.setChecked(False)
        self.risk_checkbox = QCheckBox("风险代价")
        self.risk_checkbox.setChecked(True)
        for cb in (self.occupied_checkbox, self.preblocked_checkbox,
                   self.traversable_checkbox, self.risk_checkbox):
            cb.toggled.connect(self._refresh_layers)
            display_row.addWidget(cb)
        display_row.addStretch(1)
        display_group.setLayout(display_row)

        self.status_label = QLabel("等待 world 加载。")
        self.status_label.setWordWrap(True)
        self.status_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        for w in (world_group, save_group, nav_group, display_group, self.status_label):
            left_panel.addWidget(w)
        left_panel.addStretch(1)
        left_panel_widget = QWidget()
        left_panel_widget.setLayout(left_panel)
        left_panel_widget.setFixedWidth(500)

        # ── VTK viewer ────────────────────────────────────────────────────
        self.vtk_widget = QVTKRenderWindowInteractor(self)
        self.vtk_widget.GetRenderWindow().AddRenderer(self._renderer)
        self._renderer.SetBackground(0.04, 0.07, 0.09)
        self._renderer.GradientBackgroundOn()
        self._renderer.SetBackground2(0.12, 0.16, 0.19)
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(1.5, 1.5, 1.5)
        axes.SetXAxisLabelText("")
        axes.SetYAxisLabelText("")
        axes.SetZAxisLabelText("")
        self._renderer.AddActor(axes)
        self._renderer.AddActor(self._make_ground_grid(24.0, 1.0))
        interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        interactor.Initialize()
        interactor.AddObserver("LeftButtonPressEvent", self._on_left_button_press, 1.0)
        interactor.AddObserver("MouseMoveEvent",       self._on_mouse_move,        1.0)

        top_row.addWidget(left_panel_widget, 0)
        top_row.addWidget(self.vtk_widget,   1)
        root.addLayout(top_row)
        self.setLayout(root)

    def closeEvent(self, event) -> None:
        self._ros_node.shutdown()
        super().closeEvent(event)

    # ── callbacks / slots ─────────────────────────────────────────────────
    def _spin_ros_once(self) -> None:
        # ROS1: rospy callbacks run in subscriber threads; just drain dirty flags
        layers = self._ros_node.consume_layers()
        if layers is not None:
            for layer_name, payload in layers.items():
                if payload is None:
                    continue
                if layer_name == "risk":
                    self._layer_data[layer_name] = payload
                else:
                    points, scale = payload
                    color, opacity = self._LAYER_STYLE[layer_name]
                    self._layer_data[layer_name] = (points, scale, color, opacity)
            self._refresh_layers()
            occupied_count = len(self._layer_data.get("occupied",    (np.empty((0, 3)),))[0])
            preblocked_count = len(self._layer_data.get("preblocked",  (np.empty((0, 3)),))[0])
            traversable_count = len(self._layer_data.get("traversable", (np.empty((0, 3)),))[0])
            risk_count = len(self._layer_data.get("risk",        (np.empty((0, 3)),))[0])
            self.status_label.setText(
                f"OctoMap 已更新。占据 {occupied_count}，禁行 {preblocked_count}，"
                f"可通行 {traversable_count}，风险 {risk_count}。")

        path_points = self._ros_node.consume_path()
        if path_points is not None:
            self._update_path(path_points)

    def _browse_world(self) -> None:
        selected, _ = QFileDialog.getOpenFileName(
            self, "选择 Gazebo World/SDF", str(Path.home()),
            "World Files (*.world *.sdf);;All Files (*)")
        if selected:
            self.path_edit.setText(selected)
            if not self.save_name_edit.text().strip():
                self.save_name_edit.setText(Path(selected).stem)

    def _choose_save_root(self) -> None:
        selected = QFileDialog.getExistingDirectory(
            self, "选择地图保存根目录",
            self.save_root_edit.text().strip() or str(self._default_root),
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks)
        if selected:
            self.save_root_edit.setText(selected)

    def _load_world(self) -> None:
        world_file = self.path_edit.text().strip()
        if not world_file:
            QMessageBox.warning(self, "World 文件", "请先选择 world 文件。")
            return
        path = Path(world_file)
        if not path.is_file():
            QMessageBox.warning(self, "World 文件", f"文件不存在：{path}")
            return
        self._layer_data.clear()
        self._camera_initialized = False
        self._refresh_layers()
        self._ros_node.publish_world_file(str(path))
        self.status_label.setText(
            f"已发送 world 文件：{path.name}。后端正在转换为 3D OctoMap，请稍候。")

    def _start_save(self) -> None:
        root_dir = self.save_root_edit.text().strip()
        map_name = self.save_name_edit.text().strip()
        if not root_dir:
            QMessageBox.warning(self, "保存地图", "请先选择地图根目录。")
            return
        if not map_name:
            QMessageBox.warning(self, "保存地图", "请输入地图名。")
            return
        package_path = str(Path(root_dir).expanduser() / map_name)
        self.save_root_edit.setEnabled(False)
        self.save_name_edit.setEnabled(False)
        self.overwrite_checkbox.setEnabled(False)
        self.status_label.setText(f"正在保存地图到 {package_path}，请稍候。")
        worker = SaveWorker(package_path, self.overwrite_checkbox.isChecked())
        worker.finished.connect(self._on_save_finished)
        thread = threading.Thread(target=worker.run, daemon=True)
        self._worker_thread = thread
        self._worker = worker
        thread.start()

    def _on_save_finished(self, success: bool, message: str) -> None:
        self.save_root_edit.setEnabled(True)
        self.save_name_edit.setEnabled(True)
        self.overwrite_checkbox.setEnabled(True)
        self.status_label.setText(message)
        if success:
            QMessageBox.information(self, "保存地图", "地图保存成功。")
        else:
            QMessageBox.critical(self, "保存地图", message)

    def _set_pick_mode(self, mode: str) -> None:
        self._pick_mode = mode
        self._goal_pending_position = None
        if mode == "start":
            self.status_label.setText("点击 3D 视图设置起始点。")
        else:
            self.status_label.setText("点击 3D 视图设置目标点位置，再点击一次设置目标朝向。")

    def _on_left_button_press(self, obj, _event) -> None:
        if self._pick_mode is None:
            return
        actor_list = [a for a, _ in self._layer_actors.values()]
        if not actor_list:
            return
        click_x, click_y = obj.GetEventPosition()
        picker = vtk.vtkPropPicker()
        picker.PickFromListOn()
        for a in actor_list:
            picker.AddPickList(a)
        if picker.Pick(click_x, click_y, 0, self._renderer) == 0:
            self.status_label.setText("没有选中栅格。")
            return
        pos = picker.GetPickPosition()
        picked_xyz = self._snap_pick((float(pos[0]), float(pos[1]), float(pos[2])))
        mode = self._pick_mode
        if mode == "start":
            self._pick_mode = None
            self._ros_node.publish_point("start", self._frame_id, picked_xyz)
            self._update_point_actor("start", picked_xyz)
            self.status_label.setText(
                f"起始点已设置：[{picked_xyz[0]:.2f}, {picked_xyz[1]:.2f}, {picked_xyz[2]:.2f}]")
        elif mode == "goal":
            self._goal_pending_position = picked_xyz
            self._pick_mode = "goal_heading"
            self._update_goal_visual(picked_xyz, self._goal_yaw)
            self.status_label.setText("目标点位置已设置。移动鼠标预览朝向，再点击一次确认姿态。")
        elif mode == "goal_heading" and self._goal_pending_position is not None:
            yaw = self._compute_yaw(self._goal_pending_position, picked_xyz)
            self._goal_yaw = yaw
            self._pick_mode = None
            goal_xyz = self._goal_pending_position
            self._goal_pending_position = None
            self._ros_node.publish_point("goal", self._frame_id, goal_xyz)
            self._ros_node.publish_goal_pose(self._frame_id, goal_xyz, yaw)
            self._update_goal_visual(goal_xyz, yaw)
            self.status_label.setText(
                f"目标点已设置：[{goal_xyz[0]:.2f}, {goal_xyz[1]:.2f}, {goal_xyz[2]:.2f}]，"
                f"朝向 {np.degrees(yaw):.1f}°，正在规划路径。")

    def _on_mouse_move(self, obj, _event) -> None:
        if self._pick_mode != "goal_heading" or self._goal_pending_position is None:
            return
        actor_list = [a for a, _ in self._layer_actors.values()]
        if not actor_list:
            return
        mx, my = obj.GetEventPosition()
        picker = vtk.vtkPropPicker()
        picker.PickFromListOn()
        for a in actor_list:
            picker.AddPickList(a)
        if picker.Pick(mx, my, 0, self._renderer) == 0:
            return
        pos = picker.GetPickPosition()
        cursor = self._snap_pick((float(pos[0]), float(pos[1]), float(pos[2])))
        self._goal_yaw = self._compute_yaw(self._goal_pending_position, cursor)
        self._update_goal_visual(self._goal_pending_position, self._goal_yaw)

    def _compute_yaw(self, origin, target) -> float:
        dx = float(target[0] - origin[0])
        dy = float(target[1] - origin[1])
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return self._goal_yaw
        return float(np.arctan2(dy, dx))

    def _snap_pick(self, xyz):
        layer = self._layer_data.get("traversable")
        if layer is None:
            return xyz
        points = layer[0]
        if points.size == 0:
            return xyz
        diffs = points - np.asarray(xyz, dtype=np.float32)
        nearest = int(np.argmin(np.einsum("ij,ij->i", diffs, diffs)))
        s = points[nearest]
        return (float(s[0]), float(s[1]), float(s[2]))

    # ── VTK helpers ───────────────────────────────────────────────────────
    def _make_ground_grid(self, size: float, step: float) -> vtk.vtkActor:
        append = vtk.vtkAppendPolyData()
        half = int(size / step)
        for i in range(-half, half + 1):
            for axis in range(2):
                line = vtk.vtkLineSource()
                if axis == 0:
                    line.SetPoint1(-size, i*step, 0.0)
                    line.SetPoint2(size, i*step, 0.0)
                else:
                    line.SetPoint1(i*step, -size, 0.0)
                    line.SetPoint2(i*step, size, 0.0)
                append.AddInputConnection(line.GetOutputPort())
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(append.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.24, 0.30, 0.34)
        actor.GetProperty().SetLineWidth(1.0)
        actor.GetProperty().SetOpacity(0.65)
        return actor

    def _build_voxel_actors(self, points, scale, color, opacity):
        vtk_pts = vtk.vtkPoints()
        vtk_pts.SetData(numpy_support.numpy_to_vtk(points.astype(np.float32), deep=True))
        pd = vtk.vtkPolyData()
        pd.SetPoints(vtk_pts)
        cube = vtk.vtkCubeSource()
        cube.SetXLength(float(scale[0]))
        cube.SetYLength(float(scale[1]))
        cube.SetZLength(float(scale[2]))
        glyph = vtk.vtkGlyph3DMapper()
        glyph.SetInputData(pd)
        glyph.SetSourceConnection(cube.GetOutputPort())
        glyph.ScalingOff()
        actor = vtk.vtkActor()
        actor.SetMapper(glyph)
        actor.GetProperty().SetColor(*color)
        actor.GetProperty().SetOpacity(opacity)
        actor.GetProperty().SetInterpolationToFlat()
        edge_cube = vtk.vtkCubeSource()
        edge_cube.SetXLength(float(scale[0]))
        edge_cube.SetYLength(float(scale[1]))
        edge_cube.SetZLength(float(scale[2]))
        edge_ext = vtk.vtkExtractEdges()
        edge_ext.SetInputConnection(edge_cube.GetOutputPort())
        edge_glyph = vtk.vtkGlyph3DMapper()
        edge_glyph.SetInputData(pd)
        edge_glyph.SetSourceConnection(edge_ext.GetOutputPort())
        edge_glyph.ScalingOff()
        edge_actor = vtk.vtkActor()
        edge_actor.SetMapper(edge_glyph)
        edge_actor.GetProperty().SetColor(0.0, 0.0, 0.0)
        edge_actor.GetProperty().SetLineWidth(1.0)
        edge_actor.GetProperty().SetOpacity(1.0)
        return actor, edge_actor

    def _build_risk_actors(self, points, scale, intensity):
        vtk_pts = vtk.vtkPoints()
        vtk_pts.SetData(numpy_support.numpy_to_vtk(points.astype(np.float32), deep=True))
        pd = vtk.vtkPolyData()
        pd.SetPoints(vtk_pts)
        alphas = np.clip(0.12 + 0.83 * intensity.astype(np.float32), 0.12, 0.95)
        colors = np.zeros((len(points), 4), dtype=np.uint8)
        colors[:, 0] = int(0.15 * 255)
        colors[:, 1] = int(0.35 * 255)
        colors[:, 2] = 255
        colors[:, 3] = np.round(alphas * 255).astype(np.uint8)
        vtk_colors = numpy_support.numpy_to_vtk(colors, deep=True, array_type=vtk.VTK_UNSIGNED_CHAR)
        vtk_colors.SetName("risk_rgba")
        pd.GetPointData().SetScalars(vtk_colors)
        cube = vtk.vtkCubeSource()
        cube.SetXLength(float(scale[0]))
        cube.SetYLength(float(scale[1]))
        cube.SetZLength(float(scale[2]))
        glyph = vtk.vtkGlyph3DMapper()
        glyph.SetInputData(pd)
        glyph.SetSourceConnection(cube.GetOutputPort())
        glyph.ScalingOff()
        glyph.SetScalarModeToUsePointData()
        glyph.ScalarVisibilityOn()
        glyph.SetColorModeToDirectScalars()
        actor = vtk.vtkActor()
        actor.SetMapper(glyph)
        actor.GetProperty().SetOpacity(1.0)
        actor.GetProperty().SetInterpolationToFlat()
        return actor, None

    def _refresh_layers(self, _checked=None) -> None:
        for actor, edge_actor in self._layer_actors.values():
            self._renderer.RemoveActor(actor)
            if edge_actor is not None:
                self._renderer.RemoveActor(edge_actor)
        self._layer_actors.clear()
        visibility = {
            "occupied":    self.occupied_checkbox.isChecked(),
            "preblocked":  self.preblocked_checkbox.isChecked(),
            "traversable": self.traversable_checkbox.isChecked(),
            "risk":        self.risk_checkbox.isChecked(),
        }
        for layer_name, visible in visibility.items():
            if not visible or layer_name not in self._layer_data:
                continue
            if layer_name == "risk":
                points, scale, intensity = self._layer_data[layer_name]
                if points.size == 0:
                    continue
                actor, edge_actor = self._build_risk_actors(points, scale, intensity)
            else:
                points, scale, color, opacity = self._layer_data[layer_name]
                if points.size == 0:
                    continue
                actor, edge_actor = self._build_voxel_actors(points, scale, color, opacity)
            self._renderer.AddActor(actor)
            if edge_actor is not None:
                self._renderer.AddActor(edge_actor)
            self._layer_actors[layer_name] = (actor, edge_actor)
        if not self._camera_initialized and self._layer_actors:
            self._renderer.ResetCamera()
            self._camera_initialized = True
        self.vtk_widget.GetRenderWindow().Render()

    def _update_point_actor(self, kind: str, xyz) -> None:
        old = self._start_actor if kind == "start" else self._goal_actor
        if old is not None:
            self._renderer.RemoveActor(old)
        sphere = vtk.vtkSphereSource()
        sphere.SetCenter(*xyz)
        sphere.SetRadius(0.16)
        sphere.SetThetaResolution(18)
        sphere.SetPhiResolution(18)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.1, 0.95, 0.1) if kind == "start" \
            else actor.GetProperty().SetColor(0.95, 0.1, 0.1)
        actor.GetProperty().SetOpacity(1.0)
        if kind == "start":
            self._start_actor = actor
        else:
            self._goal_actor = actor
        self._renderer.AddActor(actor)
        self.vtk_widget.GetRenderWindow().Render()

    def _update_goal_visual(self, xyz, yaw: float) -> None:
        self._update_point_actor("goal", xyz)
        if self._goal_arrow_actor is not None:
            self._renderer.RemoveActor(self._goal_arrow_actor)
            self._goal_arrow_actor = None
        arrow = vtk.vtkArrowSource()
        arrow.SetTipResolution(24)
        arrow.SetShaftResolution(24)
        arrow.SetTipLength(0.30)
        arrow.SetTipRadius(0.18)
        arrow.SetShaftRadius(0.08)
        transform = vtk.vtkTransform()
        transform.PostMultiply()
        transform.Scale(0.90, 0.90, 0.90)
        transform.RotateZ(float(np.degrees(yaw)))
        transform.Translate(float(xyz[0]), float(xyz[1]), float(xyz[2]))
        tf_filter = vtk.vtkTransformPolyDataFilter()
        tf_filter.SetTransform(transform)
        tf_filter.SetInputConnection(arrow.GetOutputPort())
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(tf_filter.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.95, 0.1, 0.1)
        actor.GetProperty().SetOpacity(0.95)
        self._renderer.AddActor(actor)
        self._goal_arrow_actor = actor
        self.vtk_widget.GetRenderWindow().Render()

    def _update_path(self, path_points) -> None:
        if self._path_actor is not None:
            self._renderer.RemoveActor(self._path_actor)
            self._path_actor = None
        if len(path_points) < 2:
            self.vtk_widget.GetRenderWindow().Render()
            return
        vtk_pts = vtk.vtkPoints()
        for p in path_points:
            vtk_pts.InsertNextPoint(float(p[0]), float(p[1]), float(p[2]))
        poly_line = vtk.vtkPolyLine()
        poly_line.GetPointIds().SetNumberOfIds(len(path_points))
        for i in range(len(path_points)):
            poly_line.GetPointIds().SetId(i, i)
        cells = vtk.vtkCellArray()
        cells.InsertNextCell(poly_line)
        poly_data = vtk.vtkPolyData()
        poly_data.SetPoints(vtk_pts)
        poly_data.SetLines(cells)
        tube = vtk.vtkTubeFilter()
        tube.SetInputData(poly_data)
        tube.SetRadius(0.06)
        tube.SetNumberOfSides(16)
        tube.CappingOn()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(tube.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.69, 0.40, 1.0)
        actor.GetProperty().SetOpacity(1.0)
        self._renderer.AddActor(actor)
        self._path_actor = actor
        self.vtk_widget.GetRenderWindow().Render()


def main() -> None:
    rospy.init_node("world_selector_gui_node", anonymous=True)
    app = QApplication(sys.argv)
    window = WorldSelectorWindow()
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()
