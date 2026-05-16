# 依赖包安装 (ROS 1 版本)

pip install vtk
pip install open3d

# 描述(基于6-robot/jie_3d_nav)
https://github.com/6-robot/jie_3d_nav
一套基于 ROS 1 的 3D 导航系统，通过 Web 界面交互。本系统已在智元科技 D1 机器狗以及留形科技 Odin 1 空间定位模组上测试通过。

本目录包含三个 ROS 1 功能包：

jie_map_msgs：地图包保存、加载、导出等自定义服务（srv）接口。

jie_octomap：OctoMap 管理包，负责多种地图格式导入、地图包保存/加载、OctoMap 可视化和编辑。

octo_planner：基于 OctoMap 的 3D 路径规划、路径跟踪控制和 Web 测试/导航 launch。

## 功能概览

将 PCD 点云地图导入为 OctoMap。

将 ROS 1 2D 栅格地图（OccupancyGrid）导入为 3D OctoMap。

将 Gazebo .world / .sdf 场景转换为 OctoMap。

保存、加载 OctoMap 地图包。

使用 Qt/VTK GUI 查看和编辑 OctoMap 栅格。

使用 Web 页面查看 OctoMap、选择起点/终点并进行路径规划。

提供面向安装了留形科技 Odin 1 的 智元 D1 机器狗的导航入口和独立网页测试入口。

## 介绍视频

Bilibili：【开源】基于ROS2的3D导航系统 (注：视频为原 ROS 2 版本演示)

YouTube：【开源】基于ROS2的3D导航系统 (注：视频为原 ROS 2 版本演示)

## 目录结构

Plaintext

jie_3d_nav/
├── jie_map_msgs/        # 自定义 srv 接口
├── jie_octomap/         # OctoMap 导入、管理、编辑、Web/GUI 工具
├── octo_planner/        # 3D 路径规划、控制器、导航 launch
├── worlds/              # 示例 Gazebo world
└── install_deps_noetic.sh

## 环境要求

Ubuntu 20.04

ROS 1 Noetic

catkin (支持 catkin_make 或 catkin build)

OctoMap / octomap_msgs

OpenCV

Open3D C++ 开发库

PyQt5、VTK、NumPy、Pillow、PyYAML

可选：rosbridge_server，用于 Web 页面通过 WebSocket 访问 ROS 1

基础编译不需要以下两个包：

d1_bringup

d1_description

💡 注意：完整智元科技 D1 机器狗导航入口 octo_planner/launch/nav.launch 仍然会在运行时使用 d1_bringup 和 d1_description，因为它会启动 d1_core 并读取智元科技 D1 机器狗的 URDF 模型。

## 安装依赖

可以使用仓库内脚本安装 ROS 1 常用依赖：

Bash

cd ~/catkin_ws/src/jie_3d_nav
bash install_deps_noetic.sh
如果 CMake 找不到 Open3D，需要额外安装 Open3D C++ 开发库，并确保 Open3DConfig.cmake 能被 CMake 找到，例如通过环境参数 Open3D_DIR 或在 CMakeLists.txt 中指定 CMAKE_PREFIX_PATH。

## 编译

从 ROS 1 工作空间（Workspace）根目录进行编译：

Bash

cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
# 使用 catkin_make 编译
catkin_make -DCMAKE_BUILD_TYPE=Release
# 或者使用 catkin build
# catkin build jie_map_msgs jie_octomap octo_planner
source devel/setup.bash
如果源码目录移动过导致 CMake 缓存冲突，可以清理后重编：

Bash

catkin_make clean
# 或者直接删除 build 和 devel 目录重编
rm -rf build/ devel/
catkin_make
## 地图导入

### 导入 PCD 点云地图

Bash

roslaunch jie_octomap import_pcd_map.launch
该 launch 会启动：

pcd_to_octomap_node

octomap_to_occupied_markers_node

map_package_manager

pcd_map_import_gui

octo_planner/jie_path_node

### 导入 ROS 2D 栅格地图

Bash

roslaunch jie_octomap import_ros_map.launch
该 launch 会启动：

occupancy_grid_to_octomap_node

octomap_to_occupied_markers_node

map_package_manager

ros_map_import_gui

octo_planner/jie_path_node

### 导入 Gazebo World / SDF

Bash

# 启动 Gazebo 物理环境
gazebo worlds/field.world
加载包内示例 world 时，推荐使用 world_name 参数：

Bash

roslaunch jie_octomap import_gazebo_world.launch world_name:=hello_gazebo.world
加载外部 world 文件时，请使用绝对路径：

Bash

roslaunch jie_octomap import_gazebo_world.launch world_file:=/absolute/path/to/map.world
如果同时传入 world_file 和 world_name，优先使用 world_file。

jie_octomap/worlds/ 目录内提供了两个示例 world 文件，并会随 jie_octomap 包安装到 share 路径：

2_storey.world：双层建筑/楼层示例。

field.world：场地示例。

加载双层建筑示例：

Bash

roslaunch jie_octomap import_gazebo_world.launch world_name:=2_storey.world
加载场地示例：

Bash

roslaunch jie_octomap import_gazebo_world.launch world_name:=hello_gazebo.world
该 launch 会启动：

world_to_octomap_node

world_selector_gui.py

map_package_manager

octo_planner/jie_path_node

## 地图管理与编辑

OctoMap 管理和编辑主入口：

Bash

roslaunch jie_octomap map_manager.launch
该 launch 会启动：

map_package_manager

octomap_to_occupied_markers_node

map_viewer_gui

可选 octo_planner/jie_path_node

map_viewer_gui 支持功能：

打开地图包 / 刷新地图 / 保存地图

查看占据、禁行、可通行、风险代价图层

编辑栅格状态：occupied、preblocked、traversable、clear

在地图上直观选择起点、终点、导航目标

## Web 可视化

### 加载地图并启动 Web 页面

Bash

roslaunch jie_octomap web_octomap.launch map_package:=/home/ubuntu/catkin_3dnavi/hello_gazebo launch_rosbridge:=true
常用参数：

map_package：已保存的地图包目录路径。

http_port：静态 Web 服务端口，默认 8080。

launch_rosbridge：是否启动 ROS 1 的 rosbridge_websocket。

launch_map_gui：是否同时启动 Qt 保存/加载窗口。

浏览器访问地址：

Plaintext

http://localhost:8080
http://<机器人IP>:8080
### 坐标系与话题规范
#### 启动gazebo环境和机器人,发送下面tree,网页自动显示定位.
坐标变换：map -> odom -> base_link

#### 导航及控制话题
路径话题：规划器向网页和底层发布 /planned_path (类型：nav_msgs/Path),相当于全局规划器,需要自己后接一个局部规划器.
注意web_cmd_vel是网页摇杆控制话题.

### Web 功能测试

Bash

roslaunch octo_planner web_test.launch
web_test.launch 用于测试网页访问、地图显示、Web 起终点选择、路径规划和基础控制链路。该 launch 已去除对 d1_bringup 和 d1_description 硬件依赖，会使用一个最小化的 base_link URDF 启动 ROS 1 的 robot_state_publisher。

启动前同样需要根据实际环境配置参数文件：

Plaintext

octo_planner/config/nav_params.yaml
至少需要部署好以下路径：

relocalization_bin_file：重定位使用的 .bin 地图文件。

map_package_dir：已经保存好的 OctoMap 地图包目录。

## 智元科技 D1 机器狗完整导航

完整机器人导航入口（连接真实硬件与动力学控制）：

Bash

roslaunch octo_planner nav.launch
该 launch 面向智元科技 D1 机器狗实际导航，并结合留形科技 Odin 1 空间定位模组相关驱动流程，会启动或使用：

d1_bringup/d1_core

d1_description/urdf/d1.urdf

odin_ros_driver

octo_planner/jie_path_node

octo_planner/d1_controller

jie_octomap/map_package_manager

Web viewer 和 rosbridge_websocket

运行前需要根据实际环境修改参数：

Plaintext

octo_planner/config/nav_params.yaml
重点字段说明：

relocalization_bin_file

map_package_dir

relocalization_pcd_file

show_rviz

show_map_gui

publish_d1_odom

use_static_odom_to_base

同时需要确认留形科技 Odin 1 空间定位模组的 ROS 1 驱动配置：

Plaintext

odin_ros_driver/config/control_command.yaml
将其中的 custom_map_mode 设置为 2，即 Relocalization mode（重定位模式）。

octo_planner/config/nav_params.yaml 中至少需要配置好：

relocalization_bin_file：重定位使用的 .bin 地图文件。

map_package_dir：已经保存好的 OctoMap 地图包目录。

如果需要使用 RViz 观察定位与点云对齐效果，还需要部署：

relocalization_pcd_file：用于 RViz 显示的 .pcd 点云地图文件。

## 其他 Launch 工具

Bash

roslaunch jie_octomap octomap_test.launch
roslaunch jie_octomap octomap_open3d.launch
roslaunch jie_octomap odin1_slam.launch
roslaunch jie_octomap odin1_loc.launch
其中 odin1_slam.launch 和 odin1_loc.launch 面向留形科技 Odin 1 空间定位模组流程，运行时需要确保 ROS 1 环境中已有 odin_ros_driver，并可选使用 odin_costmap 插件配置。
