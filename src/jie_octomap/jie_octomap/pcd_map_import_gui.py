#!/usr/bin/env python3
# pcd_map_import_gui.py  —  ROS 1 port of the original ROS 2 implementation

from __future__ import annotations

import threading
import time
import tempfile
import hashlib
from pathlib import Path

import numpy as np
import open3d as o3d
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path as PathMsg
from jie_map_msgs.srv import SaveNavigationMapPackage, SaveNavigationMapPackageRequest
from PyQt5.QtCore import QEvent, QObject, Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
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
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk
from vtk.util import numpy_support


class PcdMapImportNode:
    """ROS 1 replacement for the ROS 2 Node subclass."""
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self) -> None:
        if hasattr(self, '_initialized'):
            return

        self._initialized = True
        rospy.loginfo("Creating PcdMapImportNode (first and only time)")

        # Publishers
        self.file_pub = rospy.Publisher("/pcd_file_cmd", String, queue_size=1)
        self.start_pub = rospy.Publisher("/start_point", PointStamped, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher("/goal_point", PointStamped, queue_size=1, latch=True)
        self.goal_pose_pub = rospy.Publisher("/goal_pose", PoseStamped, queue_size=1, latch=True)

        # Subscribers
        self.occupied_sub = rospy.Subscriber(
            "/octomap_occupied_markers", Marker, self._on_occupied, queue_size=1)
        self.preblocked_sub = rospy.Subscriber(
            "/preblocked_cells_markers", Marker, self._on_preblocked, queue_size=1)
        self.traversable_sub = rospy.Subscriber(
            "/traversable_cells_markers", Marker, self._on_traversable, queue_size=1)
        self.path_sub = rospy.Subscriber(
            "/planned_path", PathMsg, self._on_path, queue_size=1)
        self.risk_sub = rospy.Subscriber(
            "/risk_cost_cells", PointCloud2, self._on_risk, queue_size=1)

        self._latest_occupied = None
        self._latest_preblocked = None
        self._latest_traversable = None
        self._latest_path_points = []
        self._latest_risk = None
        self._layer_signatures: dict[str, tuple] = {}
        self._layer_dirty = False
        self._path_dirty = False
        self._risk_dirty = False

    def publish_pcd_file(self, file_path: str) -> None:
        msg = String()
        msg.data = file_path
        self.file_pub.publish(msg)

    def publish_point(self, topic: str, frame_id: str, xyz: tuple[float, float, float]) -> None:
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

    def publish_goal_pose(self, frame_id: str, xyz: tuple[float, float, float]) -> None:
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        msg.pose.orientation.w = 1.0
        self.goal_pose_pub.publish(msg)

    def _on_occupied(self, msg: Marker) -> None:
        self._store_marker("occupied", msg)

    def _on_preblocked(self, msg: Marker) -> None:
        self._store_marker("preblocked", msg)

    def _on_traversable(self, msg: Marker) -> None:
        self._store_marker("traversable", msg)

    def _on_path(self, msg: PathMsg) -> None:
        self._latest_path_points = [
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            for pose in msg.poses
        ]
        self._path_dirty = True

    def _on_risk(self, msg: PointCloud2) -> None:
        records = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        if not records:
            xyz = np.empty((0, 3), dtype=np.float32)
            intensity = np.empty((0,), dtype=np.float32)
        else:
            values = np.array([[r[0], r[1], r[2], r[3]] for r in records], dtype=np.float32)
            xyz = values[:, :3]
            intensity = values[:, 3]
        self._latest_risk = (xyz, self._infer_voxel_scale(), intensity)
        self._risk_dirty = True

    def _infer_voxel_scale(self) -> np.ndarray:
        for layer in (self._latest_occupied, self._latest_preblocked, self._latest_traversable):
            if layer is not None:
                return np.asarray(layer[1], dtype=np.float32)
        return np.array([0.2, 0.2, 0.2], dtype=np.float32)

    def _store_marker(self, layer_name: str, msg: Marker) -> None:
        if msg.type != Marker.CUBE_LIST:
            return
        points = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float32)
        scale = np.array([msg.scale.x, msg.scale.y, msg.scale.z], dtype=np.float32)
        point_digest = hashlib.blake2b(points.tobytes(), digest_size=8).digest()
        signature = (points.shape, point_digest, float(scale[0]), float(scale[1]), float(scale[2]))
        if self._layer_signatures.get(layer_name) == signature:
            return
        self._layer_signatures[layer_name] = signature
        setattr(self, f"_latest_{layer_name}", (points, scale))
        self._layer_dirty = True

    def consume_layers(self):
        if not self._layer_dirty:
            return None
        self._layer_dirty = False
        return {
            "occupied":    self._latest_occupied,
            "preblocked":  self._latest_preblocked,
            "traversable": self._latest_traversable,
        }

    def consume_path(self):
        if not self._path_dirty:
            return None
        self._path_dirty = False
        return list(self._latest_path_points)

    def consume_risk(self):
        if not self._risk_dirty or self._latest_risk is None:
            return None
        self._risk_dirty = False
        return self._latest_risk

    def save_package(self, package_path: str, overwrite: bool, timeout_sec: float = 100.0):
        service_name = "/map_package_manager/save_package"
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
        except rospy.ROSException:
            return False, f"保存服务 {service_name} 不可用。"
        try:
            proxy = rospy.ServiceProxy(service_name, SaveNavigationMapPackage)
            req = SaveNavigationMapPackageRequest()
            req.package_path = package_path
            req.overwrite = overwrite
            resp = proxy(req)
            return bool(resp.success), str(resp.message)
        except rospy.ServiceException as exc:
            return False, f"保存地图失败：{exc}"

    def shutdown(self) -> None:
        """Unregister all pub/sub (called on window close)."""
        self.file_pub.unregister()
        self.start_pub.unregister()
        self.goal_pub.unregister()
        self.goal_pose_pub.unregister()
        self.occupied_sub.unregister()
        self.preblocked_sub.unregister()
        self.traversable_sub.unregister()
        self.path_sub.unregister()
        self.risk_sub.unregister()


class SaveWorker(QObject):
    finished = pyqtSignal(bool, str)

    def __init__(self, package_path: str, overwrite: bool) -> None:
        super().__init__()
        self.package_path = package_path
        self.overwrite = overwrite

    def run(self) -> None:
        node = PcdMapImportNode()
        try:
            ok, message = node.save_package(self.package_path, self.overwrite)
        finally:
            node.shutdown()
        self.finished.emit(ok, message)


class PcdMapImportWindow(QWidget):
    _LAYER_STYLE = {
        "occupied":    ((0.95, 0.45, 0.15), 1.0),
        "preblocked":  ((0.15, 0.35, 1.0),  1.0),
        "traversable": ((0.20, 0.95, 0.55), 0.30),
        "risk":        ((0.15, 0.35, 1.0),  0.55),
    }

    def __init__(self) -> None:
        super().__init__()
        self._default_root = Path("~/catkin_3dnavi")
        self._selected_pcd: Path | None = None
        self._preview_points: np.ndarray | None = None
        self._source_cloud: o3d.geometry.PointCloud | None = None
        self._working_cloud: o3d.geometry.PointCloud | None = None
        self._temp_convert_pcd: Path | None = None
        self._has_previewed_pcd = False
        self._has_converted_map = False
        self._worker_thread: threading.Thread | None = None
        self._ros_node = PcdMapImportNode()
        self._pcd_renderer = vtk.vtkRenderer()
        self._octomap_renderer = vtk.vtkRenderer()
        self._pcd_actor: vtk.vtkActor | None = None
        self._start_actor: vtk.vtkActor | None = None
        self._goal_actor: vtk.vtkActor | None = None
        self._path_actor: vtk.vtkActor | None = None
        self._latest_path_points: list[tuple[float, float, float]] = []
        self._pick_mode: str | None = None
        self._frame_id = "map"
        self._erase_cursor_actor: vtk.vtkActor | None = None
        self._erase_cursor_edge_actor: vtk.vtkActor | None = None
        self._erase_position = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self._layer_actors: dict[str, tuple[vtk.vtkActor, vtk.vtkActor]] = {}
        self._layer_data: dict = {}
        self._octomap_camera_needs_reset = True
        self._init_ui()
        QApplication.instance().installEventFilter(self)
        # ROS 1: poll rospy callbacks via a QTimer (no spin thread needed)
        self._spin_timer = QTimer(self)
        self._spin_timer.timeout.connect(self._spin_ros_once)
        self._spin_timer.start(100)

    def _init_ui(self) -> None:
        self.setWindowTitle("PCD 地图导入")
        self.resize(2200, 1500)

        root = QVBoxLayout()
        top_row = QHBoxLayout()

        import_group = QGroupBox("PCD 文件读取")
        import_form = QFormLayout()
        pcd_row = QHBoxLayout()
        self.pcd_edit = QLineEdit()
        self.pcd_edit.setPlaceholderText("选择 PCD 文件")
        default_path = "/home/ubuntu/jie_3d_nav/src/jie_octomap/pcd/scene_downsampled.pcd"
        self.pcd_edit.setText(default_path)
        pcd_btn = QPushButton("选择 PCD")
        pcd_btn.clicked.connect(self._choose_pcd)
        pcd_row.addWidget(self.pcd_edit, 1)
        pcd_row.addWidget(pcd_btn)
        import_form.addRow("PCD 文件", pcd_row)

        self.resolution_spin = QDoubleSpinBox()
        self.resolution_spin.setDecimals(3)
        self.resolution_spin.setRange(0.01, 2.0)
        self.resolution_spin.setSingleStep(0.05)
        self.resolution_spin.setValue(0.2)
        self.resolution_spin.setEnabled(False)
        import_form.addRow("Octomap分辨率", self.resolution_spin)

        self.downsample_spin = QDoubleSpinBox()
        self.downsample_spin.setDecimals(3)
        self.downsample_spin.setRange(0.0, 2.0)
        self.downsample_spin.setSingleStep(0.05)
        self.downsample_spin.setValue(0.1)
        import_form.addRow("降采样体素(m)", self.downsample_spin)

        for label, slot in [
            ("统计离群点滤波", self._apply_statistical_filter),
            ("半径离群点滤波", self._apply_radius_filter),
            ("删除小簇",       self._apply_cluster_filter),
            ("平面范围裁剪",   self._apply_ransac_filter),
        ]:
            btn = QPushButton(label)
            btn.clicked.connect(slot)
            import_form.addRow("", btn)

        save_pcd_btn = QPushButton("保存 PCD 文件")
        save_pcd_btn.clicked.connect(self._save_edited_pcd)
        import_form.addRow("", save_pcd_btn)

        convert_btn = QPushButton("转换为 Octomap")
        convert_btn.clicked.connect(self._convert_to_octomap)
        import_form.addRow("", convert_btn)
        import_group.setLayout(import_form)

        save_group = QGroupBox("Octomap地图保存")
        save_form = QFormLayout()
        root_row = QHBoxLayout()
        self.root_edit = QLineEdit(str(self._default_root))
        choose_root_btn = QPushButton("选择目录")
        choose_root_btn.clicked.connect(self._choose_root)
        root_row.addWidget(self.root_edit, 1)
        root_row.addWidget(choose_root_btn)
        save_form.addRow("根目录", root_row)

        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("请输入保存后的地图名")
        save_form.addRow("地图名", self.name_edit)

        self.overwrite_checkbox = QCheckBox("允许覆盖")
        self.overwrite_checkbox.setChecked(True)
        save_form.addRow("", self.overwrite_checkbox)

        save_btn = QPushButton("保存Octomap地图")
        save_btn.clicked.connect(self._save_package)
        save_form.addRow("", save_btn)
        save_group.setLayout(save_form)

        self.status_label = QLabel("等待操作。")
        self.status_label.setWordWrap(True)
        self.status_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        left_panel = QVBoxLayout()
        left_panel.addWidget(import_group)
        left_panel.addWidget(save_group)
        left_panel.addWidget(self.status_label)
        left_panel_widget = QWidget()
        left_panel_widget.setLayout(left_panel)
        left_panel_widget.setFixedWidth(360)

        viewer_layout = QHBoxLayout()

        pcd_view_group = QGroupBox("PCD 点云预览")
        pcd_view_layout = QVBoxLayout()
        erase_row = QHBoxLayout()
        self.erase_checkbox = QCheckBox("启用抹除方块")
        self.erase_checkbox.toggled.connect(self._toggle_erase_mode)
        erase_row.addWidget(self.erase_checkbox)
        erase_row.addWidget(QLabel("方块边长(m)"))
        self.erase_size_spin = QDoubleSpinBox()
        self.erase_size_spin.setDecimals(2)
        self.erase_size_spin.setRange(0.05, 20.0)
        self.erase_size_spin.setSingleStep(0.05)
        self.erase_size_spin.setValue(0.5)
        self.erase_size_spin.valueChanged.connect(self._on_erase_size_changed)
        erase_row.addWidget(self.erase_size_spin)
        erase_hint = QLabel("[X轴移动：W/S] [Y轴移动：A/D] [Z轴移动：Q/E] 空格:抹除")
        erase_hint.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        erase_row.addWidget(erase_hint)
        erase_row.addStretch(1)
        pcd_view_layout.addLayout(erase_row)
        self.pcd_vtk_widget = QVTKRenderWindowInteractor(self)
        self.pcd_vtk_widget.GetRenderWindow().AddRenderer(self._pcd_renderer)
        self._setup_renderer(self._pcd_renderer)
        pcd_view_layout.addWidget(self.pcd_vtk_widget)
        pcd_view_group.setLayout(pcd_view_layout)

        octomap_view_group = QGroupBox("Octomap")
        octomap_view_layout = QVBoxLayout()
        octomap_control_row = QHBoxLayout()
        self.occupied_checkbox = QCheckBox("占据")
        self.occupied_checkbox.setChecked(True)
        self.occupied_checkbox.toggled.connect(self._refresh_layers)
        self.preblocked_checkbox = QCheckBox("禁行")
        self.preblocked_checkbox.setChecked(False)
        self.preblocked_checkbox.toggled.connect(self._refresh_layers)
        self.traversable_checkbox = QCheckBox("可通行")
        self.traversable_checkbox.setChecked(False)
        self.traversable_checkbox.toggled.connect(self._refresh_layers)
        self.risk_checkbox = QCheckBox("风险代价")
        self.risk_checkbox.setChecked(False)
        self.risk_checkbox.toggled.connect(self._refresh_layers)
        self.path_checkbox = QCheckBox("规划路径")
        self.path_checkbox.setChecked(True)
        self.path_checkbox.toggled.connect(self._toggle_path_visibility)
        start_btn = QPushButton("起始点")
        start_btn.clicked.connect(lambda: self._set_pick_mode("start"))
        goal_btn = QPushButton("目标点")
        goal_btn.clicked.connect(lambda: self._set_pick_mode("goal"))
        for w in (self.occupied_checkbox, self.preblocked_checkbox,
                  self.traversable_checkbox, self.risk_checkbox,
                  self.path_checkbox, start_btn, goal_btn):
            octomap_control_row.addWidget(w)
        octomap_control_row.addStretch(1)
        octomap_view_layout.addLayout(octomap_control_row)
        self.octomap_vtk_widget = QVTKRenderWindowInteractor(self)
        self.octomap_vtk_widget.GetRenderWindow().AddRenderer(self._octomap_renderer)
        self._setup_renderer(self._octomap_renderer)
        octomap_view_layout.addWidget(self.octomap_vtk_widget)
        octomap_view_group.setLayout(octomap_view_layout)

        viewer_layout.addWidget(pcd_view_group, 1)
        viewer_layout.addWidget(octomap_view_group, 1)

        top_row.addWidget(left_panel_widget, 0)
        top_row.addLayout(viewer_layout, 1)
        root.addLayout(top_row)
        self.setLayout(root)

        self._initialize_interactor(self.pcd_vtk_widget)
        self._initialize_interactor(self.octomap_vtk_widget)
        self.octomap_vtk_widget.GetRenderWindow().GetInteractor().AddObserver(
            "LeftButtonPressEvent", self._on_octomap_left_button_press, 1.0
        )
        self.installEventFilter(self)
        self.pcd_vtk_widget.installEventFilter(self)
        QTimer.singleShot(500, self._auto_preview_if_path_exists)

    def _auto_preview_if_path_exists(self) -> None:
        pcd_text = self.pcd_edit.text().strip()
        if pcd_text and Path(pcd_text).exists():
            self._selected_pcd = Path(pcd_text)
            self._has_previewed_pcd = False
            self._has_converted_map = False
            self._preview_points = None
            self._source_cloud = None
            self._working_cloud = None
            self._clear_pcd_preview()
            self._remove_erase_cursor()
            self._clear_octomap_layers()
            self._cleanup_temp_convert_pcd()
            self._octomap_camera_needs_reset = True
            if not self.name_edit.text().strip():
                self.name_edit.setText(self._selected_pcd.stem)
            self._preview_pcd()
            self.status_label.setText(f"已自动加载 PCD 文件：{pcd_text}")

    def closeEvent(self, event) -> None:
        self._cleanup_temp_convert_pcd()
        self._ros_node.shutdown()
        super().closeEvent(event)

    def eventFilter(self, obj, event):
        if event.type() == QEvent.KeyPress and self.erase_checkbox.isChecked():
            if self._handle_erase_key(event):
                return True
        return super().eventFilter(obj, event)

    def _handle_erase_key(self, event) -> bool:
        key = event.key()
        move_map = {
            Qt.Key_W: np.array([1.0,  0.0,  0.0], dtype=np.float32),
            Qt.Key_S: np.array([-1.0,  0.0,  0.0], dtype=np.float32),
            Qt.Key_A: np.array([0.0,  1.0,  0.0], dtype=np.float32),
            Qt.Key_D: np.array([0.0, -1.0,  0.0], dtype=np.float32),
            Qt.Key_Q: np.array([0.0,  0.0, -1.0], dtype=np.float32),
            Qt.Key_E: np.array([0.0,  0.0,  1.0], dtype=np.float32),
        }
        if key in move_map:
            self._erase_position = self._erase_position + move_map[key] * 0.10
            self._update_erase_cursor()
            self.status_label.setText(
                f"抹除方块位置：[{self._erase_position[0]:.2f}, "
                f"{self._erase_position[1]:.2f}, {self._erase_position[2]:.2f}]"
            )
            return True
        if key == Qt.Key_Space:
            self._erase_points_in_cursor()
            return True
        return False

    def _spin_ros_once(self) -> None:
        # ROS 1: callbacks are delivered in subscriber threads; just drain dirty flags
        layers = self._ros_node.consume_layers()
        if layers is not None:
            for layer_name, payload in layers.items():
                if payload is None:
                    continue
                points, scale = payload
                color, opacity = self._LAYER_STYLE[layer_name]
                self._layer_data[layer_name] = (points, scale, color, opacity)
            self._refresh_layers()

        risk = self._ros_node.consume_risk()
        if risk is not None:
            self._layer_data["risk"] = risk
            self._refresh_layers()

        path_points = self._ros_node.consume_path()
        if path_points is not None:
            self._latest_path_points = path_points
            if self.path_checkbox.isChecked():
                self._update_path(path_points)

    def _set_pick_mode(self, mode: str) -> None:
        self._pick_mode = mode
        self.status_label.setText(
            "点击右侧 3D 栅格结果设置起始点。" if mode == "start"
            else "点击右侧 3D 栅格结果设置目标点。"
        )

    def _on_octomap_left_button_press(self, obj, _event) -> None:
        if self._pick_mode is None:
            return
        actor_list = [actor for actor, _edge in self._layer_actors.values()]
        if not actor_list:
            self.status_label.setText("没有可选中的栅格。")
            return
        click_x, click_y = obj.GetEventPosition()
        picker = vtk.vtkPropPicker()
        picker.PickFromListOn()
        for actor in actor_list:
            picker.AddPickList(actor)
        if picker.Pick(click_x, click_y, 0, self._octomap_renderer) == 0:
            self.status_label.setText("没有选中栅格。")
            return
        pos = picker.GetPickPosition()
        picked_xyz = self._snap_pick_to_cell((float(pos[0]), float(pos[1]), float(pos[2])))
        mode = self._pick_mode
        self._pick_mode = None
        self._ros_node.publish_point(mode, self._frame_id, picked_xyz)
        if mode == "goal":
            self._ros_node.publish_goal_pose(self._frame_id, picked_xyz)
        self._update_point_actor(mode, picked_xyz)
        label = "起始点" if mode == "start" else "目标点"
        self.status_label.setText(
            f"{label}已设置：[{picked_xyz[0]:.2f}, {picked_xyz[1]:.2f}, {picked_xyz[2]:.2f}]"
        )

    def _snap_pick_to_cell(self, xyz):
        for layer_name in ("traversable", "occupied", "preblocked"):
            layer = self._layer_data.get(layer_name)
            if layer is None or len(layer) < 2:
                continue
            points = layer[0]
            if points.size == 0:
                continue
            diffs = points - np.asarray(xyz, dtype=np.float32)
            nearest = int(np.argmin(np.einsum("ij,ij->i", diffs, diffs)))
            snapped = points[nearest]
            return (float(snapped[0]), float(snapped[1]), float(snapped[2]))
        return xyz

    def _choose_pcd(self) -> None:
        default_dir = Path("~/ros_ws").expanduser()
        if not default_dir.is_dir():
            default_dir = Path.home()
        selected, _ = QFileDialog.getOpenFileName(
            self, "选择 PCD 文件", str(default_dir), "Point Cloud Files (*.pcd)")
        if selected:
            self._selected_pcd = Path(selected)
            self.pcd_edit.setText(selected)
            self._reset_pcd_state()
            if not self.name_edit.text().strip():
                self.name_edit.setText(self._selected_pcd.stem)
            self._preview_pcd()

    def _reset_pcd_state(self) -> None:
        self._has_previewed_pcd = False
        self._has_converted_map = False
        self._preview_points = None
        self._source_cloud = None
        self._working_cloud = None
        self._clear_pcd_preview()
        self._remove_erase_cursor()
        self._clear_octomap_layers()
        self._cleanup_temp_convert_pcd()
        self._octomap_camera_needs_reset = True

    def _choose_root(self) -> None:
        selected = QFileDialog.getExistingDirectory(
            self, "选择地图保存根目录",
            self.root_edit.text().strip() or str(self._default_root),
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
        )
        if selected:
            self.root_edit.setText(selected)

    def _preview_pcd(self) -> None:
        pcd_text = self.pcd_edit.text().strip()
        if not pcd_text:
            QMessageBox.warning(self, "PCD 地图导入", "请先选择 PCD 文件。")
            return
        pcd_path = Path(pcd_text)
        if not pcd_path.exists():
            QMessageBox.warning(self, "PCD 地图导入", f"PCD 文件不存在：{pcd_path}")
            return
        try:
            point_cloud = o3d.io.read_point_cloud(str(pcd_path))
        except Exception as exc:
            QMessageBox.critical(self, "PCD 地图导入", f"读取 PCD 失败：{exc}")
            return
        if not point_cloud.has_points():
            QMessageBox.warning(self, "PCD 地图导入", "PCD 文件里没有点。")
            return
        preview_cloud = point_cloud
        downsample = float(self.downsample_spin.value())
        if downsample > 0.0:
            preview_cloud = point_cloud.voxel_down_sample(downsample)
        self._source_cloud = point_cloud
        self._working_cloud = preview_cloud
        self._initialize_erase_position()
        self._has_previewed_pcd = True
        self._has_converted_map = False
        self._cleanup_temp_convert_pcd()
        self._update_preview_from_working_cloud(reset_camera=True)
        self._clear_octomap_layers()
        self._octomap_camera_needs_reset = True
        self.status_label.setText(
            f"已读取 PCD：{pcd_path.name}。左侧显示预览点云，共 {self._preview_points.shape[0]} 个预览点。 确认后点击转换为 Octomap"
        )

    def _convert_to_octomap(self) -> None:
        if not self._has_previewed_pcd or self._selected_pcd is None or self._working_cloud is None:
            QMessageBox.warning(self, "Octomap 转换", "请先读取并预览 PCD。")
            return
        self._clear_octomap_layers()
        self._octomap_camera_needs_reset = True
        try:
            convert_path = self._prepare_convert_pcd()
        except Exception as exc:
            QMessageBox.critical(self, "Octomap 转换", f"写入处理后点云失败：{exc}")
            return
        self._ros_node.publish_pcd_file(str(convert_path))
        self._has_converted_map = True
        self.status_label.setText(
            f"已发送转换请求：{convert_path.name}。右侧窗口会在 Octomap 和各种栅格层生成后刷新。"
        )

    def _build_package_path(self):
        root_dir = self.root_edit.text().strip()
        map_name = self.name_edit.text().strip()
        if not root_dir:
            QMessageBox.warning(self, "地图目录", "请先选择保存根目录。")
            return None
        if not map_name:
            QMessageBox.warning(self, "地图目录", "请输入地图名。")
            return None
        return Path(root_dir).expanduser() / map_name

    def _save_package(self) -> None:
        if not self._has_converted_map:
            QMessageBox.warning(self, "地图包保存", "请先完成 Octomap 转换。")
            return
        package_path = self._build_package_path()
        if package_path is None:
            return
        self.status_label.setText(f"正在保存地图包到 {package_path} ，请稍候。")
        worker = SaveWorker(str(package_path), self.overwrite_checkbox.isChecked())
        worker.finished.connect(self._on_save_finished)
        thread = threading.Thread(target=worker.run, daemon=True)
        self._worker_thread = thread
        self._worker = worker
        thread.start()

    def _on_save_finished(self, success: bool, message: str) -> None:
        self.status_label.setText(message)
        if success:
            QMessageBox.information(self, "地图包保存", "地图包保存成功。")
        else:
            QMessageBox.critical(self, "地图包保存", message)

    def _save_edited_pcd(self) -> None:
        if self._working_cloud is None or not self._working_cloud.has_points():
            QMessageBox.warning(self, "保存 PCD 文件", "请先选择并编辑 PCD 点云。")
            return
        default_path = (
            self._selected_pcd.with_name(f"{self._selected_pcd.stem}_edited.pcd")
            if self._selected_pcd else Path.home() / "edited.pcd"
        )
        selected, _ = QFileDialog.getSaveFileName(
            self, "保存编辑后的 PCD 文件", str(default_path), "Point Cloud Files (*.pcd)")
        if not selected:
            return
        output_path = Path(selected).expanduser()
        if output_path.suffix.lower() != ".pcd":
            output_path = output_path.with_suffix(".pcd")
        try:
            ok = o3d.io.write_point_cloud(str(output_path), self._working_cloud, write_ascii=False)
        except Exception as exc:
            QMessageBox.critical(self, "保存 PCD 文件", f"保存失败：{exc}")
            return
        if not ok:
            QMessageBox.critical(self, "保存 PCD 文件", f"保存失败：{output_path}")
            return
        self.status_label.setText(f"已保存编辑后的 PCD 文件：{output_path}")
        QMessageBox.information(self, "保存 PCD 文件", "保存成功。")

    # ---- erase mode ----
    def _toggle_erase_mode(self, checked: bool) -> None:
        if checked:
            if self._working_cloud is None or not self._working_cloud.has_points():
                self.erase_checkbox.setChecked(False)
                QMessageBox.warning(self, "PCD 点云抹除", "请先选择并读取 PCD 点云。")
                return
            self._initialize_erase_position()
            self._update_erase_cursor()
            self.activateWindow()
            self.pcd_vtk_widget.setFocus()
            self.status_label.setText(
                "PCD 点云抹除已开启。使用 W/S 移动 X，A/D 移动 Y，Q/E 移动 Z，空格抹除当前方块内点云。"
            )
        else:
            self._remove_erase_cursor()
            self.pcd_vtk_widget.GetRenderWindow().Render()

    def _on_erase_size_changed(self, _value: float) -> None:
        if self.erase_checkbox.isChecked():
            self._update_erase_cursor()
            self.pcd_vtk_widget.setFocus()

    def _initialize_erase_position(self) -> None:
        if self._working_cloud is None or not self._working_cloud.has_points():
            self._erase_position = np.array([0.0, 0.0, 0.0], dtype=np.float32)
            return
        points = np.asarray(self._working_cloud.points, dtype=np.float32)
        self._erase_position = np.mean(points, axis=0).astype(np.float32)

    def _remove_erase_cursor(self) -> None:
        for attr in ("_erase_cursor_actor", "_erase_cursor_edge_actor"):
            actor = getattr(self, attr)
            if actor is not None:
                self._pcd_renderer.RemoveActor(actor)
                setattr(self, attr, None)

    def _update_erase_cursor(self) -> None:
        self._remove_erase_cursor()
        size = float(self.erase_size_spin.value())
        actor, edge_actor = self._build_box_actors(self._erase_position, size)
        self._pcd_renderer.AddActor(actor)
        self._pcd_renderer.AddActor(edge_actor)
        self._erase_cursor_actor = actor
        self._erase_cursor_edge_actor = edge_actor
        self.pcd_vtk_widget.GetRenderWindow().Render()

    def _erase_points_in_cursor(self) -> None:
        cloud = self._require_working_cloud()
        if cloud is None:
            return
        points = np.asarray(cloud.points, dtype=np.float32)
        if points.size == 0:
            return
        half_size = float(self.erase_size_spin.value()) * 0.5
        lower = self._erase_position - half_size
        upper = self._erase_position + half_size
        inside = np.all((points >= lower) & (points <= upper), axis=1)
        removed_count = int(np.count_nonzero(inside))
        if removed_count == 0:
            self.status_label.setText("当前抹除方块内没有点云。")
            return
        keep_indices = np.where(~inside)[0]
        if keep_indices.size == 0:
            QMessageBox.warning(self, "PCD 点云抹除", "抹除会导致点云为空，已取消。")
            return
        self._working_cloud = cloud.select_by_index(keep_indices.tolist())
        self._has_converted_map = False
        self._cleanup_temp_convert_pcd()
        self._update_preview_from_working_cloud(reset_camera=False)
        if self.erase_checkbox.isChecked():
            self._update_erase_cursor()
        self.status_label.setText(
            f"已抹除 {removed_count} 个点。剩余 {len(self._working_cloud.points)} 个点。"
        )

    def _require_working_cloud(self):
        if not self._has_previewed_pcd or self._working_cloud is None:
            QMessageBox.warning(self, "PCD 处理", "请先读取并预览 PCD。")
            return None
        return self._working_cloud

    def _apply_processed_cloud(self, processed_cloud, action_name, reset_camera=False):
        if not processed_cloud.has_points():
            QMessageBox.warning(self, "PCD 处理", f"{action_name} 后点云为空，已取消。")
            return
        self._working_cloud = processed_cloud
        self._has_converted_map = False
        self._cleanup_temp_convert_pcd()
        self._update_preview_from_working_cloud(reset_camera=reset_camera)
        self.status_label.setText(
            f"{action_name}完成。左侧已更新为处理后的点云，共 "
            f"{self._preview_points.shape[0]} 个预览点。"
        )
        if self.erase_checkbox.isChecked():
            self._initialize_erase_position()
            self._update_erase_cursor()

    def _apply_statistical_filter(self) -> None:
        cloud = self._require_working_cloud()
        if cloud is None:
            return
        filtered_cloud, inlier_indices = cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)
        if len(inlier_indices) == 0:
            QMessageBox.warning(self, "PCD 处理", "统计离群点滤波后没有保留任何点。")
            return
        self._apply_processed_cloud(filtered_cloud, "统计离群点滤波")

    def _apply_radius_filter(self) -> None:
        cloud = self._require_working_cloud()
        if cloud is None:
            return
        radius = max(float(self.downsample_spin.value()) * 2.0, 0.20)
        filtered_cloud, inlier_indices = cloud.remove_radius_outlier(nb_points=12, radius=radius)
        if len(inlier_indices) == 0:
            QMessageBox.warning(self, "PCD 处理", "半径离群点滤波后没有保留任何点。")
            return
        self._apply_processed_cloud(filtered_cloud, "半径离群点滤波")

    def _apply_cluster_filter(self) -> None:
        cloud = self._require_working_cloud()
        if cloud is None:
            return
        eps = max(float(self.downsample_spin.value()) * 2.0, 0.20)
        labels = np.asarray(cloud.cluster_dbscan(eps=eps, min_points=20, print_progress=False))
        if labels.size == 0 or np.all(labels < 0):
            QMessageBox.warning(self, "PCD 处理", "没有检测到有效聚类。")
            return
        valid_labels = labels[labels >= 0]
        cluster_ids, cluster_sizes = np.unique(valid_labels, return_counts=True)
        keep_cluster_ids = cluster_ids[cluster_sizes >= 100]
        if keep_cluster_ids.size == 0:
            QMessageBox.warning(self, "PCD 处理", "删除小簇后没有保留任何聚类。")
            return
        keep_indices = np.where(np.isin(labels, keep_cluster_ids))[0]
        self._apply_processed_cloud(cloud.select_by_index(keep_indices.tolist()), "删除小簇")

    def _apply_ransac_filter(self) -> None:
        cloud = self._require_working_cloud()
        if cloud is None:
            return
        if len(cloud.points) < 100:
            QMessageBox.warning(self, "PCD 处理", "点数太少，无法进行 RANSAC 平面范围裁剪。")
            return
        distance_threshold = max(float(self.downsample_spin.value()), 0.05)
        extent_margin = max(float(self.downsample_spin.value()) * 6.0, 0.5)
        min_plane_points = 80
        working_cloud = cloud
        plane_boxes = []
        for _ in range(6):
            if len(working_cloud.points) < min_plane_points:
                break
            _, inlier_indices = working_cloud.segment_plane(
                distance_threshold=distance_threshold, ransac_n=3, num_iterations=1000)
            if len(inlier_indices) < min_plane_points:
                break
            plane_points = np.asarray(
                working_cloud.select_by_index(inlier_indices).points, dtype=np.float32)
            plane_boxes.append((plane_points.min(axis=0) - extent_margin,
                                plane_points.max(axis=0) + extent_margin))
            working_cloud = working_cloud.select_by_index(inlier_indices, invert=True)
        if not plane_boxes:
            QMessageBox.warning(self, "PCD 处理", "没有分割出足够稳定的平面范围。")
            return
        points = np.asarray(cloud.points, dtype=np.float32)
        keep_mask = np.zeros(points.shape[0], dtype=bool)
        for plane_min, plane_max in plane_boxes:
            keep_mask |= np.all((points >= plane_min) & (points <= plane_max), axis=1)
        keep_indices = np.where(keep_mask)[0]
        if keep_indices.size == 0:
            QMessageBox.warning(self, "PCD 处理", "按平面范围裁剪后没有保留任何点。")
            return
        self._apply_processed_cloud(
            cloud.select_by_index(keep_indices.tolist()),
            f"RANSAC平面范围裁剪（{len(plane_boxes)}个平面，扩展距离 {extent_margin:.2f}m）",
        )

    def _prepare_convert_pcd(self) -> Path:
        if self._working_cloud is None:
            raise RuntimeError("working cloud is empty")
        self._cleanup_temp_convert_pcd()
        tmp = tempfile.NamedTemporaryFile(prefix="pcd_map_import_", suffix=".pcd", delete=False)
        tmp_path = Path(tmp.name)
        tmp.close()
        if not o3d.io.write_point_cloud(str(tmp_path), self._working_cloud, write_ascii=False):
            tmp_path.unlink(missing_ok=True)
            raise RuntimeError("failed to write temp pcd")
        self._temp_convert_pcd = tmp_path
        return tmp_path

    def _cleanup_temp_convert_pcd(self) -> None:
        if self._temp_convert_pcd is not None:
            self._temp_convert_pcd.unlink(missing_ok=True)
            self._temp_convert_pcd = None

    # ---- VTK helpers ----
    def _setup_renderer(self, renderer: vtk.vtkRenderer) -> None:
        renderer.SetBackground(0.04, 0.07, 0.09)
        renderer.GradientBackgroundOn()
        renderer.SetBackground2(0.12, 0.16, 0.19)
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(1.5, 1.5, 1.5)
        axes.SetXAxisLabelText("")
        axes.SetYAxisLabelText("")
        axes.SetZAxisLabelText("")
        renderer.AddActor(axes)
        renderer.AddActor(self._make_ground_grid(24.0, 1.0))

    def _initialize_interactor(self, vtk_widget) -> None:
        interactor = vtk_widget.GetRenderWindow().GetInteractor()
        interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        interactor.Initialize()

    def _make_ground_grid(self, size: float, step: float) -> vtk.vtkActor:
        append = vtk.vtkAppendPolyData()
        half = int(size / step)
        for i in range(-half, half + 1):
            for axis in range(2):
                line = vtk.vtkLineSource()
                if axis == 0:
                    line.SetPoint1(-size, i * step, 0.0)
                    line.SetPoint2(size, i * step, 0.0)
                else:
                    line.SetPoint1(i * step, -size, 0.0)
                    line.SetPoint2(i * step,  size, 0.0)
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
        vtk_points = vtk.vtkPoints()
        vtk_points.SetData(numpy_support.numpy_to_vtk(points.astype(np.float32), deep=True))
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(vtk_points)
        cube = vtk.vtkCubeSource()
        cube.SetXLength(float(scale[0]))
        cube.SetYLength(float(scale[1]))
        cube.SetZLength(float(scale[2]))
        glyph = vtk.vtkGlyph3DMapper()
        glyph.SetInputData(polydata)
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
        edge_extract = vtk.vtkExtractEdges()
        edge_extract.SetInputConnection(edge_cube.GetOutputPort())
        edge_glyph = vtk.vtkGlyph3DMapper()
        edge_glyph.SetInputData(polydata)
        edge_glyph.SetSourceConnection(edge_extract.GetOutputPort())
        edge_glyph.ScalingOff()
        edge_actor = vtk.vtkActor()
        edge_actor.SetMapper(edge_glyph)
        edge_actor.GetProperty().SetColor(0.0, 0.0, 0.0)
        edge_actor.GetProperty().SetLineWidth(1.0)
        edge_actor.GetProperty().SetOpacity(1.0)
        return actor, edge_actor

    def _build_box_actors(self, center, size):
        cube = vtk.vtkCubeSource()
        cube.SetCenter(float(center[0]), float(center[1]), float(center[2]))
        cube.SetXLength(size)
        cube.SetYLength(size)
        cube.SetZLength(size)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cube.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(1.0, 0.85, 0.0)
        actor.GetProperty().SetOpacity(0.22)
        actor.GetProperty().SetInterpolationToFlat()
        edge_extract = vtk.vtkExtractEdges()
        edge_extract.SetInputConnection(cube.GetOutputPort())
        edge_mapper = vtk.vtkPolyDataMapper()
        edge_mapper.SetInputConnection(edge_extract.GetOutputPort())
        edge_actor = vtk.vtkActor()
        edge_actor.SetMapper(edge_mapper)
        edge_actor.GetProperty().SetColor(1.0, 0.75, 0.0)
        edge_actor.GetProperty().SetLineWidth(2.0)
        edge_actor.GetProperty().SetOpacity(1.0)
        return actor, edge_actor

    def _build_point_cloud_actor(self, points: np.ndarray) -> vtk.vtkActor:
        vtk_points = vtk.vtkPoints()
        vtk_points.SetData(numpy_support.numpy_to_vtk(points.astype(np.float32), deep=True))
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(vtk_points)
        verts = vtk.vtkCellArray()
        verts.Allocate(points.shape[0])
        for idx in range(points.shape[0]):
            verts.InsertNextCell(1)
            verts.InsertCellPoint(idx)
        polydata.SetVerts(verts)
        z = points[:, 2].astype(np.float32)
        z_min, z_max = float(z.min()), float(z.max())
        t = ((z - z_min) / (z_max - z_min)).reshape(-1, 1) if z_max > z_min else np.zeros((len(z), 1), dtype=np.float32)
        low = np.array([153., 230., 255.])
        mid = np.array([0., 220., 80.])
        high = np.array([255., 0., 0.])
        colors = np.empty((len(z), 3), dtype=np.float32)
        fh = (t <= 0.5)[:, 0]
        colors[fh] = (1 - t[fh]*2) * low + (t[fh]*2) * mid
        colors[~fh] = (1 - (t[~fh]-0.5)*2) * mid + ((t[~fh]-0.5)*2) * high
        colors = np.clip(colors, 0, 255).astype(np.uint8)
        vtk_colors = vtk.vtkUnsignedCharArray()
        vtk_colors.SetName("z_color")
        vtk_colors.SetNumberOfComponents(3)
        vtk_colors.SetNumberOfTuples(len(z))
        for idx, c in enumerate(colors):
            vtk_colors.SetTuple3(idx, int(c[0]), int(c[1]), int(c[2]))
        polydata.GetPointData().SetScalars(vtk_colors)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polydata)
        mapper.SetScalarModeToUsePointData()
        mapper.ScalarVisibilityOn()
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetPointSize(2.0)
        actor.GetProperty().RenderPointsAsSpheresOn()
        return actor

    def _show_pcd_preview(self, points: np.ndarray) -> None:
        self._clear_pcd_preview()
        if points.size == 0:
            return
        self._pcd_actor = self._build_point_cloud_actor(points)
        self._pcd_renderer.AddActor(self._pcd_actor)
        self._pcd_renderer.ResetCamera()
        self.pcd_vtk_widget.GetRenderWindow().Render()

    def _clear_pcd_preview(self) -> None:
        if self._pcd_actor is not None:
            self._pcd_renderer.RemoveActor(self._pcd_actor)
            self._pcd_actor = None
        if hasattr(self, "pcd_vtk_widget"):
            self.pcd_vtk_widget.GetRenderWindow().Render()

    def _make_preview_points(self, cloud) -> np.ndarray:
        points = np.asarray(cloud.points, dtype=np.float32)
        if points.shape[0] > 200000:
            step = int(np.ceil(points.shape[0] / 200000.0))
            points = points[::step]
        return points

    def _update_preview_from_working_cloud(self, reset_camera: bool) -> None:
        if self._working_cloud is None:
            self._preview_points = None
            self._clear_pcd_preview()
            return
        self._preview_points = self._make_preview_points(self._working_cloud)
        self._clear_pcd_preview()
        if self._preview_points.size == 0:
            return
        self._pcd_actor = self._build_point_cloud_actor(self._preview_points)
        self._pcd_renderer.AddActor(self._pcd_actor)
        if reset_camera:
            self._pcd_renderer.ResetCamera()
        self.pcd_vtk_widget.GetRenderWindow().Render()

    def _clear_octomap_actors(self) -> None:
        for actor, edge_actor in self._layer_actors.values():
            self._octomap_renderer.RemoveActor(actor)
            if edge_actor is not None:
                self._octomap_renderer.RemoveActor(edge_actor)
        self._layer_actors.clear()
        if hasattr(self, "octomap_vtk_widget"):
            self.octomap_vtk_widget.GetRenderWindow().Render()

    def _clear_octomap_layers(self) -> None:
        self._clear_octomap_actors()
        self._layer_data.clear()

    def _refresh_layers(self, checked=None) -> None:
        self._clear_octomap_actors()
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
            self._octomap_renderer.AddActor(actor)
            if edge_actor is not None:
                self._octomap_renderer.AddActor(edge_actor)
            self._layer_actors[layer_name] = (actor, edge_actor)
        if self._octomap_camera_needs_reset:
            self._octomap_renderer.ResetCamera()
            self._octomap_camera_needs_reset = False
        self.octomap_vtk_widget.GetRenderWindow().Render()

    def _toggle_path_visibility(self, checked: bool) -> None:
        if checked:
            self._update_path(self._latest_path_points)
        else:
            if self._path_actor is not None:
                self._octomap_renderer.RemoveActor(self._path_actor)
                self._path_actor = None
                self.octomap_vtk_widget.GetRenderWindow().Render()

    def _update_point_actor(self, kind: str, xyz) -> None:
        old_actor = self._start_actor if kind == "start" else self._goal_actor
        if old_actor is not None:
            self._octomap_renderer.RemoveActor(old_actor)
        sphere = vtk.vtkSphereSource()
        sphere.SetCenter(float(xyz[0]), float(xyz[1]), float(xyz[2]))
        sphere.SetRadius(0.16)
        sphere.SetThetaResolution(18)
        sphere.SetPhiResolution(18)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.1, 0.95, 0.1) if kind == "start" else actor.GetProperty().SetColor(0.95, 0.1, 0.1)
        actor.GetProperty().SetOpacity(1.0)
        if kind == "start":
            self._start_actor = actor
        else:
            self._goal_actor = actor
        self._octomap_renderer.AddActor(actor)
        self.octomap_vtk_widget.GetRenderWindow().Render()

    def _update_path(self, path_points) -> None:
        if self._path_actor is not None:
            self._octomap_renderer.RemoveActor(self._path_actor)
            self._path_actor = None
        if not self.path_checkbox.isChecked() or len(path_points) < 2:
            self.octomap_vtk_widget.GetRenderWindow().Render()
            return
        vtk_points = vtk.vtkPoints()
        for p in path_points:
            vtk_points.InsertNextPoint(float(p[0]), float(p[1]), float(p[2]))
        poly_line = vtk.vtkPolyLine()
        poly_line.GetPointIds().SetNumberOfIds(len(path_points))
        for i in range(len(path_points)):
            poly_line.GetPointIds().SetId(i, i)
        cells = vtk.vtkCellArray()
        cells.InsertNextCell(poly_line)
        poly_data = vtk.vtkPolyData()
        poly_data.SetPoints(vtk_points)
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
        self._octomap_renderer.AddActor(actor)
        self._path_actor = actor
        self.octomap_vtk_widget.GetRenderWindow().Render()

    def _build_risk_actors(self, points, scale, intensity):
        vtk_points = vtk.vtkPoints()
        vtk_points.SetData(numpy_support.numpy_to_vtk(points.astype(np.float32), deep=True))
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(vtk_points)
        alphas = np.clip(0.12 + 0.83 * intensity.astype(np.float32), 0.12, 0.95)
        colors = np.zeros((len(points), 4), dtype=np.uint8)
        colors[:, 0] = int(0.15 * 255)
        colors[:, 1] = int(0.35 * 255)
        colors[:, 2] = 255
        colors[:, 3] = np.round(alphas * 255).astype(np.uint8)
        vtk_colors = numpy_support.numpy_to_vtk(colors, deep=True, array_type=vtk.VTK_UNSIGNED_CHAR)
        vtk_colors.SetName("risk_rgba")
        polydata.GetPointData().SetScalars(vtk_colors)
        cube = vtk.vtkCubeSource()
        cube.SetXLength(float(scale[0]))
        cube.SetYLength(float(scale[1]))
        cube.SetZLength(float(scale[2]))
        glyph = vtk.vtkGlyph3DMapper()
        glyph.SetInputData(polydata)
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


def main() -> None:
    rospy.init_node("pcd_map_import_gui_node", anonymous=True)
    app = QApplication([])
    window = PcdMapImportWindow()
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()
