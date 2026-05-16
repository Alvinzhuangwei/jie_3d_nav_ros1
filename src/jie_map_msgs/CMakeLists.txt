cmake_minimum_required(VERSION 3.0.2)
project(jie_map_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
)

add_service_files(
  FILES
  ExportNavigationSnapshot.srv
  GetNavigationMapMeta.srv
  LoadNavigationMapPackage.srv
  SaveNavigationMapPackage.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
    
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    std_msgs
)
