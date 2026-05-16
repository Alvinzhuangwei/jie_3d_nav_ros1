from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def build_auto_load_action(context):
    package_path = LaunchConfiguration("map_package").perform(context)
    return [
        ExecuteProcess(
            cmd=[
                "/bin/bash",
                "-lc",
                "ros2 service call /map_package_manager/load_package "
                "jie_map_msgs/srv/LoadNavigationMapPackage "
                f"\"{{package_path: '{package_path}'}}\"",
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("jie_octomap")
    web_root = os.path.join(pkg_share, "web")

    map_package_arg = DeclareLaunchArgument(
        "map_package",
        default_value="/home/robot/maps/map",
        description="Path to saved OctoMap map package directory",
    )
    http_port_arg = DeclareLaunchArgument(
        "http_port",
        default_value="8080",
        description="Port for the static web server",
    )
    launch_rosbridge_arg = DeclareLaunchArgument(
        "launch_rosbridge",
        default_value="false",
        description="Launch rosbridge_websocket if rosbridge_server is installed",
    )
    launch_map_gui_arg = DeclareLaunchArgument(
        "launch_map_gui",
        default_value="false",
        description="Launch map package manager and PyQt save/load window",
    )

    planner_node = Node(
        package="octo_planner",
        executable="jie_path_node",
        name="jie_path_node",
        output="screen",
        parameters=[
            {
                "octomap_topic": "/octomap",
                "start_topic": "/start_point",
                "goal_topic": "/goal_point",
                "path_topic": "/planned_path",
                "path_marker_topic": "/planned_path_marker",
                "preblocked_marker_topic": "/preblocked_cells_markers",
                "traversable_marker_topic": "/traversable_cells_markers",
                "risk_cost_topic": "/risk_cost_cells",
                "frame_id": "map",
                "map_id": "web_loaded_map",
                "source_world_file": "",
                "robot_radius": 0.25,
                "max_iterations": 500000,
                "snap_search_radius_cells": 12,
                "require_ground_support": True,
                "strict_direct_ground_support": False,
                "ground_support_xy_radius_cells": 1,
                "ground_support_depth_cells": 1,
                "enable_preblocked_costmap": True,
                "preblocked_costmap_radius_cells": 3,
                "preblocked_costmap_weight": 2.5,
            }
        ],
    )

    occupied_marker_node = Node(
        package="jie_octomap",
        executable="octomap_to_occupied_markers_node",
        name="octomap_to_occupied_markers",
        output="screen",
        parameters=[
            {
                "octomap_topic": "/octomap",
                "marker_topic": "/octomap_occupied_markers",
                "frame_id": "map",
            }
        ],
    )

    web_click_selector_node = Node(
        package="jie_octomap",
        executable="web_click_selector.py",
        name="web_click_selector",
        output="screen",
        parameters=[
            {
                "occupied_marker_topic": "/octomap_occupied_markers",
                "preblocked_marker_topic": "/preblocked_cells_markers",
                "raw_click_topic": "/web_clicked_point", # 用户在网页 3D 视图中点击的位置坐标
                "marker_topic": "/selection_markers",
                "start_topic": "/start_point",
                "goal_topic": "/goal_point",
                "status_topic": "/web_selection_status", # 选点状态反馈（成功/失败/坐标等）
                "robot_radius": 0.25,
                "snap_search_radius_cells": 12,
                "require_ground_support": True,
                "strict_direct_ground_support": False,
                "ground_support_xy_radius_cells": 1,
                "ground_support_depth_cells": 1,
            }
        ],
    )

    map_package_manager_node = Node(
        package="jie_octomap",
        executable="map_package_manager",
        name="map_package_manager",
        output="screen",
    )

    map_save_gui_node = Node(
        package="jie_octomap",
        executable="map_save_gui",
        name="map_save_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_map_gui")),
    )

    http_server = ExecuteProcess(
        cmd=["python3", "-m", "http.server", LaunchConfiguration("http_port"), "--directory", web_root],
        output="screen",
    )

    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_rosbridge")),
    )

    auto_load_map = TimerAction(
        period=2.0,
        actions=[
            OpaqueFunction(function=build_auto_load_action)
        ],
    )

    return LaunchDescription(
        [
            map_package_arg,
            http_port_arg,
            launch_rosbridge_arg,
            launch_map_gui_arg,
            planner_node,
            occupied_marker_node,
            web_click_selector_node,
            map_package_manager_node,
            map_save_gui_node,
            http_server,
            rosbridge_node,
            auto_load_map,
        ]
    )
