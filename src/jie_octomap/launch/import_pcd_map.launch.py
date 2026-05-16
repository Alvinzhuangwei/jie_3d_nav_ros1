from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pcd_to_octomap_node = Node(
        package="jie_octomap",
        executable="pcd_to_octomap_node",
        name="pcd_to_octomap",
        output="screen",
        parameters=[
            {
                "pcd_file_cmd_topic": "/pcd_file_cmd",
                "octomap_topic": "/octomap",
                "frame_id": "map",
                "resolution": 0.5,
                "voxel_downsample_m": 0.1,
                "min_points_per_voxel": 1,
                "min_cluster_voxels": 2,
            }
        ],
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
                "map_id": "imported_pcd_map",
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

    map_package_manager_node = Node(
        package="jie_octomap",
        executable="map_package_manager",
        name="map_package_manager",
        output="screen",
    )

    importer_gui_node = Node(
        package="jie_octomap",
        executable="pcd_map_import_gui",
        name="pcd_map_import_gui",
        output="screen",
    )

    return LaunchDescription(
        [
            pcd_to_octomap_node,
            planner_node,
            occupied_marker_node,
            map_package_manager_node,
            importer_gui_node,
        ]
    )
