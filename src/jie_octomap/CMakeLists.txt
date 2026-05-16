cmake_minimum_required(VERSION 3.0.2)
project(jie_octomap)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  jie_map_msgs
  octomap_msgs
  visualization_msgs
  std_msgs
  tf2_ros
  tf2_msgs
)

find_package(octomap REQUIRED)
find_package(Open3D QUIET)
find_library(TINYXML2_LIB tinyxml2 REQUIRED)
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    geometry_msgs
    nav_msgs
    jie_map_msgs
    octomap_msgs
    visualization_msgs
    std_msgs
    tf2_ros
    tf2_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  /usr/include/eigen3
)

# ---- octomap_test ----
add_executable(octomap_test src/octomap_test.cpp)
target_link_libraries(octomap_test
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
add_dependencies(octomap_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ---- octomap_to_cloud_node ----
add_executable(octomap_to_cloud_node src/octomap_to_cloud_node.cpp)
target_link_libraries(octomap_to_cloud_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
add_dependencies(octomap_to_cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ---- octomap_to_occupied_markers_node ----
add_executable(octomap_to_occupied_markers_node src/octomap_to_occupied_markers_node.cpp)
target_link_libraries(octomap_to_occupied_markers_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
add_dependencies(octomap_to_occupied_markers_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ---- occupancy_grid_to_octomap_node ----
add_executable(occupancy_grid_to_octomap_node src/occupancy_grid_to_octomap_node.cpp)
target_link_libraries(occupancy_grid_to_octomap_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
add_dependencies(occupancy_grid_to_octomap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ---- rviz_click_selector_node ----
add_executable(rviz_click_selector_node src/rviz_click_selector_node.cpp)
target_link_libraries(rviz_click_selector_node ${catkin_LIBRARIES})
add_dependencies(rviz_click_selector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ---- world_to_octomap_node ----
add_executable(world_to_octomap_node src/world_to_octomap_node.cpp)
target_link_libraries(world_to_octomap_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${TINYXML2_LIB}
)
add_dependencies(world_to_octomap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ---- pcd_to_octomap_node (only if Open3D found) ----
if(Open3D_FOUND)
  add_executable(pcd_to_octomap_node src/pcd_to_octomap_node.cpp)
  target_link_libraries(pcd_to_octomap_node
    ${catkin_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    Open3D::Open3D
  )
  add_dependencies(pcd_to_octomap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  # ---- octomap_open3d_viewer_node ----
  add_executable(octomap_open3d_viewer_node src/octomap_open3d_viewer_node.cpp)
  target_link_libraries(octomap_open3d_viewer_node
    ${catkin_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    Open3D::Open3D
  )
  add_dependencies(octomap_open3d_viewer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  install(TARGETS pcd_to_octomap_node octomap_open3d_viewer_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

install(TARGETS
  octomap_test
  octomap_to_cloud_node
  octomap_to_occupied_markers_node
  occupancy_grid_to_octomap_node
  rviz_click_selector_node
  world_to_octomap_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/map_package_manager
  scripts/map_save_gui
  scripts/map_viewer_gui
  scripts/ros_map_import_gui
  scripts/pcd_map_import_gui
  scripts/open3d_octomap_viewer.py
  scripts/pcd_file_publisher.py
  scripts/no_cache_http_server.py
  scripts/save_bin_map_gui.py
  scripts/world_file_picker_gui.py
  scripts/world_selector_gui.py
  scripts/web_click_selector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(DIRECTORY jie_octomap/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)