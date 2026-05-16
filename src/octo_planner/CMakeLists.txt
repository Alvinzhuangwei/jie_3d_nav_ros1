cmake_minimum_required(VERSION 3.0.2)
project(octo_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  jie_map_msgs
  octomap_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    jie_map_msgs
    octomap_msgs
    visualization_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ---- jie_path_node ----
add_executable(jie_path_node src/jie_path_node.cpp)
target_link_libraries(jie_path_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
add_dependencies(jie_path_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ---- d1_controller ----
add_executable(d1_controller src/d1_controller.cpp)
target_link_libraries(d1_controller
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(d1_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ---- test_map_to_odom_tf_node ----
add_executable(test_map_to_odom_tf_node src/test_map_to_odom_tf_node.cpp)
target_link_libraries(test_map_to_odom_tf_node ${catkin_LIBRARIES})
add_dependencies(test_map_to_odom_tf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS
  jie_path_node
  d1_controller
  test_map_to_odom_tf_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)