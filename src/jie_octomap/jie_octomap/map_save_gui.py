#!/usr/bin/env python3

from __future__ import annotations

import sys
import threading
from pathlib import Path

import rospy
from jie_map_msgs.srv import LoadNavigationMapPackage, SaveNavigationMapPackage
from PyQt5.QtCore import QObject, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class SaveMapClient:
    def __init__(self) -> None:
        rospy.wait_for_service("/map_package_manager/save_package")
        self.save_package_service = rospy.ServiceProxy(
            "/map_package_manager/save_package", SaveNavigationMapPackage
        )

    def save_package(self, package_path: str, overwrite: bool, timeout_sec: float = 10.0):
        try:
            result = self.save_package_service(package_path, overwrite)  # 应该是 overwrite
            return bool(result.success), str(result.message)
        except rospy.ServiceException as e:
            return False, f"保存地图失败: {str(e)}"
        except rospy.ROSException as e:
            return False, f"保存地图超时: {str(e)}"


class LoadMapClient:
    def __init__(self) -> None:
        rospy.wait_for_service("/map_package_manager/load_package")
        self.load_package_service = rospy.ServiceProxy(
            "/map_package_manager/load_package", LoadNavigationMapPackage
        )

    def load_package(self, package_path: str, timeout_sec: float = 10.0):
        try:
            result = self.load_package_service(package_path)
            return bool(result.success), str(result.message)
        except rospy.ServiceException as e:
            return False, f"读取地图失败: {str(e)}"
        except rospy.ROSException as e:
            return False, f"读取地图超时: {str(e)}"


class SaveWorker(QObject):
    finished = pyqtSignal(bool, str)

    def __init__(self, package_path: str, overwrite: bool) -> None:
        super().__init__()
        self.package_path = package_path
        self.overwrite = overwrite

    def run(self) -> None:
        client = SaveMapClient()
        ok, message = client.save_package(self.package_path, self.overwrite)
        self.finished.emit(ok, message)


class LoadWorker(QObject):
    finished = pyqtSignal(bool, str)

    def __init__(self, package_path: str) -> None:
        super().__init__()
        self.package_path = package_path

    def run(self) -> None:
        client = LoadMapClient()
        ok, message = client.load_package(self.package_path)
        self.finished.emit(ok, message)


class MapSaveWindow(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self._worker_thread: threading.Thread | None = None
        self._default_root = Path("/home/robot/maps")
        self._init_ui()

    def _init_ui(self) -> None:
        self.setWindowTitle("地图保存")
        self.setMinimumWidth(560)

        root = QVBoxLayout()
        root.setSpacing(14)

        path_label = QLabel("根目录")
        path_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        path_row = QHBoxLayout()
        self.path_edit = QLineEdit()
        self.path_edit.setText(str(self._default_root))
        self.path_edit.setPlaceholderText("请选择地图根目录")
        browse_btn = QPushButton("选择文件夹")
        browse_btn.clicked.connect(self._choose_directory)
        path_row.addWidget(self.path_edit, 1)
        path_row.addWidget(browse_btn)

        name_label = QLabel("地图名")
        name_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("请输入地图名，例如 lv2_map_001")

        self.overwrite_checkbox = QCheckBox("允许覆盖已存在目录")
        self.overwrite_checkbox.setChecked(True)

        self.status_label = QLabel("等待操作。")
        self.status_label.setWordWrap(True)

        self.save_button = QPushButton("保存地图")
        self.save_button.clicked.connect(self._start_save)
        self.load_button = QPushButton("加载地图")
        self.load_button.clicked.connect(self._start_load)

        root.addWidget(path_label)
        root.addLayout(path_row)
        root.addWidget(name_label)
        root.addWidget(self.name_edit)
        root.addWidget(self.overwrite_checkbox)
        root.addWidget(self.status_label)
        button_row = QHBoxLayout()
        button_row.addWidget(self.save_button)
        button_row.addWidget(self.load_button)
        root.addLayout(button_row)
        self.setLayout(root)

    def _choose_directory(self) -> None:
        initial_dir = self.path_edit.text().strip() or str(Path.home())
        selected = QFileDialog.getExistingDirectory(
            self,
            "选择地图保存目录",
            initial_dir,
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
        )
        if selected:
            self.path_edit.setText(selected)

    def _set_busy(self, busy: bool) -> None:
        self.path_edit.setEnabled(not busy)
        self.name_edit.setEnabled(not busy)
        self.overwrite_checkbox.setEnabled(not busy)
        self.save_button.setEnabled(not busy)
        self.load_button.setEnabled(not busy)

    def _build_package_path(self) -> Path | None:
        root_dir = self.path_edit.text().strip()
        map_name = self.name_edit.text().strip()
        if not root_dir:
            QMessageBox.warning(self, "地图目录", "请先选择地图根目录。")
            return None
        if not map_name:
            QMessageBox.warning(self, "地图目录", "请输入地图名。")
            return None
        return Path(root_dir).expanduser() / map_name

    def _start_save(self) -> None:
        package_path = self._build_package_path()
        if package_path is None:
            return

        self._set_busy(True)
        self.status_label.setText(f"正在保存地图到 {package_path} ，请稍候。")

        worker = SaveWorker(str(package_path), self.overwrite_checkbox.isChecked())
        worker.finished.connect(self._on_save_finished)

        thread = threading.Thread(target=worker.run, daemon=True)
        self._worker_thread = thread
        self._worker = worker
        thread.start()

    def _start_load(self) -> None:
        selected = QFileDialog.getExistingDirectory(
            self,
            "选择地图目录",
            self.path_edit.text().strip() or str(self._default_root),
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
        )
        if not selected:
            return
        selected_path = Path(selected)
        self.path_edit.setText(str(selected_path.parent))
        self.name_edit.setText(selected_path.name)

        self._set_busy(True)
        self.status_label.setText(f"正在加载地图 {selected_path} ，请稍候。")

        worker = LoadWorker(str(selected_path))
        worker.finished.connect(self._on_load_finished)

        thread = threading.Thread(target=worker.run, daemon=True)
        self._worker_thread = thread
        self._worker = worker
        thread.start()

    def _on_save_finished(self, success: bool, message: str) -> None:
        self._set_busy(False)
        self.status_label.setText(message)
        if success:
            QMessageBox.information(self, "保存地图", "地图保存成功。")
        else:
            QMessageBox.critical(self, "保存地图", message)

    def _on_load_finished(self, success: bool, message: str) -> None:
        self._set_busy(False)
        self.status_label.setText(message)
        if success:
            QMessageBox.information(self, "加载地图", "地图加载成功。")
        else:
            QMessageBox.critical(self, "加载地图", message)


def main(args=None) -> None:
    rospy.init_node("map_save_gui_client", anonymous=True)
    app = QApplication(sys.argv)
    window = MapSaveWindow()
    window.show()
    exit_code = app.exec_()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
