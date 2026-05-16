#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <open3d/Open3D.h>

#include <ros/ros.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class OctomapOpen3DViewerNode
{
public:
  OctomapOpen3DViewerNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  : dirty_(false), arrow_dirty_(false), path_dirty_(false), preblocked_dirty_(false)
  {
    std::string octomap_topic, marker_topic, path_topic, preblocked_marker_topic;
    pnh.param<std::string>("octomap_topic", octomap_topic, "/octomap");
    pnh.param<std::string>("marker_topic", marker_topic, "/selection_markers");
    pnh.param<std::string>("path_topic", path_topic, "/planned_path");
    pnh.param<std::string>("preblocked_marker_topic", preblocked_marker_topic, "/preblocked_cells_markers");
    pnh.param<bool>("freeze_cloud_after_first", freeze_cloud_after_first_, true);

    octomap_sub_ = nh.subscribe(octomap_topic, 1,
                                &OctomapOpen3DViewerNode::onOctomap, this);
    marker_sub_ = nh.subscribe(marker_topic, 1,
                               &OctomapOpen3DViewerNode::onMarkers, this);
    path_sub_ = nh.subscribe(path_topic, 1,
                             &OctomapOpen3DViewerNode::onPath, this);
    preblocked_sub_ = nh.subscribe(preblocked_marker_topic, 1,
                                   &OctomapOpen3DViewerNode::onPreblocked, this);

    ROS_INFO("octomap_open3d_viewer started. octomap=%s marker=%s path=%s preblocked=%s",
             octomap_topic.c_str(), marker_topic.c_str(),
             path_topic.c_str(), preblocked_marker_topic.c_str());
  }

  bool consumeLatest(std::vector<Eigen::Vector3d> & out_points)
  {
    std::lock_guard<std::mutex> lock(points_mutex_);
    if (!dirty_) return false;
    out_points = latest_points_;
    dirty_ = false;
    return true;
  }

  struct ArrowState
  {
    bool has_start{false};
    bool has_goal{false};
    bool has_start_cube{false};
    bool has_goal_cube{false};
    Eigen::Vector3d start_base{0, 0, 0};
    Eigen::Vector3d start_tip{0, 0, 0};
    Eigen::Vector3d goal_base{0, 0, 0};
    Eigen::Vector3d goal_tip{0, 0, 0};
    Eigen::Vector3d start_cube_center{0, 0, 0};
    Eigen::Vector3d goal_cube_center{0, 0, 0};
    double start_cube_size{0.30};
    double goal_cube_size{0.30};
  };

  bool consumeArrows(ArrowState & out)
  {
    std::lock_guard<std::mutex> lock(arrow_mutex_);
    if (!arrow_dirty_) return false;
    out = arrows_;
    arrow_dirty_ = false;
    return true;
  }

  bool consumePath(std::vector<Eigen::Vector3d> & out)
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!path_dirty_) return false;
    out = latest_path_points_;
    path_dirty_ = false;
    return true;
  }

  bool consumePreblocked(std::vector<Eigen::Vector3d> & out)
  {
    std::lock_guard<std::mutex> lock(preblocked_mutex_);
    if (!preblocked_dirty_) return false;
    out = latest_preblocked_points_;
    preblocked_dirty_ = false;
    return true;
  }

  bool freezeCloudAfterFirst() const { return freeze_cloud_after_first_; }

  static std::shared_ptr<open3d::geometry::TriangleMesh> makeArrowMesh(
    const Eigen::Vector3d & base, const Eigen::Vector3d & tip,
    const Eigen::Vector3d & color)
  {
    const Eigen::Vector3d dir = tip - base;
    const double len = dir.norm();
    const Eigen::Vector3d u = len > 1e-6 ? dir / len : Eigen::Vector3d(0, 0, 1);
    const double cyl_h = std::max(0.1, len * 0.65);
    const double cone_h = std::max(0.05, len * 0.35);
    const double cyl_r = std::max(0.08, len * 0.20);
    const double cone_r = std::max(0.16, len * 0.36);
    auto mesh = open3d::geometry::TriangleMesh::CreateArrow(cyl_r, cone_r, cyl_h, cone_h, 24, 4, 1);
    mesh->ComputeVertexNormals();
    mesh->PaintUniformColor(color);
    const Eigen::Quaterniond q =
      Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), u);
    mesh->Rotate(q.toRotationMatrix(), Eigen::Vector3d::Zero());
    mesh->Translate(base);
    return mesh;
  }

  static std::shared_ptr<open3d::geometry::TriangleMesh> makeCubeMesh(
    const Eigen::Vector3d & center, double size, const Eigen::Vector3d & color)
  {
    auto cube = open3d::geometry::TriangleMesh::CreateBox(size, size, size);
    cube->ComputeVertexNormals();
    cube->PaintUniformColor(color);
    cube->Translate(center - Eigen::Vector3d(size * 0.5, size * 0.5, size * 0.5));
    return cube;
  }

private:
  void onOctomap(const octomap_msgs::Octomap::ConstPtr & msg)
  {
    std::unique_ptr<octomap::AbstractOcTree> tree_ptr(octomap_msgs::msgToMap(*msg));
    if (!tree_ptr) { ROS_ERROR("Failed to decode octomap message."); return; }
    auto * oc = dynamic_cast<octomap::OcTree *>(tree_ptr.get());
    if (!oc) { ROS_ERROR("Decoded map is not octomap::OcTree."); return; }

    std::vector<Eigen::Vector3d> pts;
    pts.reserve(oc->size());
    for (auto it = oc->begin_leafs(); it != oc->end_leafs(); ++it) {
      if (!oc->isNodeOccupied(*it)) continue;
      pts.emplace_back(it.getX(), it.getY(), it.getZ());
    }
    {
      std::lock_guard<std::mutex> lock(points_mutex_);
      latest_points_.swap(pts);
      dirty_ = true;
    }
    ROS_INFO_THROTTLE(3.0, "Received OctoMap and converted to %zu points.",
                      latest_points_.size());
  }

  void onMarkers(const visualization_msgs::MarkerArray::ConstPtr & msg)
  {
    ArrowState next;
    for (const auto & m : msg->markers) {
      if (m.type == visualization_msgs::Marker::ARROW && m.points.size() >= 2) {
        if (m.id == 0) {
          next.has_start = true;
          next.start_base = Eigen::Vector3d(m.points[0].x, m.points[0].y, m.points[0].z);
          next.start_tip  = Eigen::Vector3d(m.points[1].x, m.points[1].y, m.points[1].z);
        } else if (m.id == 1) {
          next.has_goal = true;
          next.goal_base = Eigen::Vector3d(m.points[0].x, m.points[0].y, m.points[0].z);
          next.goal_tip  = Eigen::Vector3d(m.points[1].x, m.points[1].y, m.points[1].z);
        }
      } else if (m.type == visualization_msgs::Marker::CUBE) {
        if (m.id == 2) {
          next.has_start_cube = true;
          next.start_cube_center = Eigen::Vector3d(
            m.pose.position.x, m.pose.position.y, m.pose.position.z);
          next.start_cube_size = std::max(0.05, static_cast<double>(m.scale.x));
        } else if (m.id == 3) {
          next.has_goal_cube = true;
          next.goal_cube_center = Eigen::Vector3d(
            m.pose.position.x, m.pose.position.y, m.pose.position.z);
          next.goal_cube_size = std::max(0.05, static_cast<double>(m.scale.x));
        }
      }
    }
    {
      std::lock_guard<std::mutex> lock(arrow_mutex_);
      arrows_ = next;
      arrow_dirty_ = true;
    }
  }

  void onPath(const nav_msgs::Path::ConstPtr & msg)
  {
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(msg->poses.size());
    for (const auto & p : msg->poses)
      pts.emplace_back(p.pose.position.x, p.pose.position.y, p.pose.position.z);
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      latest_path_points_ = std::move(pts);
      path_dirty_ = true;
    }
  }

  void onPreblocked(const visualization_msgs::Marker::ConstPtr & msg)
  {
    if (msg->type != visualization_msgs::Marker::CUBE_LIST) return;
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(msg->points.size());
    for (const auto & p : msg->points)
      pts.emplace_back(p.x, p.y, p.z);
    {
      std::lock_guard<std::mutex> lock(preblocked_mutex_);
      latest_preblocked_points_ = std::move(pts);
      preblocked_dirty_ = true;
    }
  }

  std::mutex points_mutex_, arrow_mutex_, path_mutex_, preblocked_mutex_;
  std::vector<Eigen::Vector3d> latest_points_, latest_path_points_, latest_preblocked_points_;
  ArrowState arrows_;
  bool dirty_, arrow_dirty_, path_dirty_, preblocked_dirty_;
  bool freeze_cloud_after_first_;
  ros::Subscriber octomap_sub_, marker_sub_, path_sub_, preblocked_sub_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "octomap_open3d_viewer");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  auto node = std::make_shared<OctomapOpen3DViewerNode>(nh, pnh);

  // ROS spin 在独立线程，Open3D 渲染在主线程
  std::thread ros_thread([&]() { ros::spin(); });

  open3d::visualization::Visualizer vis;
  const bool ok = vis.CreateVisualizerWindow("Open3D OctoMap Viewer (Read-Only)", 1280, 800);
  if (!ok) {
    ROS_ERROR("Failed to create Open3D window.");
    ros::shutdown();
    ros_thread.join();
    return 1;
  }
  auto & ro = vis.GetRenderOption();
  ro.background_color_ = Eigen::Vector3d(0, 0, 0);
  ro.point_size_ = 4.0;
  ro.line_width_ = 4.0;

  auto cloud           = std::make_shared<open3d::geometry::PointCloud>();
  auto preblocked_cloud = std::make_shared<open3d::geometry::PointCloud>();
  std::shared_ptr<open3d::geometry::TriangleMesh> start_arrow, goal_arrow, start_cube, goal_cube;
  std::shared_ptr<open3d::geometry::LineSet> path_lines;
  bool geometry_added = false, preblocked_added = false;
  bool start_added = false, goal_added = false;
  bool start_cube_added = false, goal_cube_added = false;
  bool path_added = false;

  while (ros::ok()) {
    std::vector<Eigen::Vector3d> pts;
    if (node->consumeLatest(pts)) {
      if (!geometry_added || !node->freezeCloudAfterFirst()) {
        cloud->points_ = pts;
        cloud->colors_.assign(pts.size(), Eigen::Vector3d(1, 1, 1));
        if (!geometry_added) {
          vis.AddGeometry(cloud);
          vis.GetViewControl().SetZoom(0.35);
          geometry_added = true;
        } else {
          vis.UpdateGeometry(cloud);
        }
      }
    }

    std::vector<Eigen::Vector3d> pre_pts;
    if (node->consumePreblocked(pre_pts)) {
      preblocked_cloud->points_ = pre_pts;
      preblocked_cloud->colors_.assign(pre_pts.size(), Eigen::Vector3d(0.15, 0.35, 1.0));
      if (!preblocked_added) { vis.AddGeometry(preblocked_cloud); preblocked_added = true; }
      else vis.UpdateGeometry(preblocked_cloud);
    }

    OctomapOpen3DViewerNode::ArrowState arrows;
    if (node->consumeArrows(arrows)) {
      // Start arrow
      if (arrows.has_start) {
        auto na = OctomapOpen3DViewerNode::makeArrowMesh(
          arrows.start_base, arrows.start_tip, Eigen::Vector3d(0.1, 0.95, 0.1));
        if (start_added && start_arrow) vis.RemoveGeometry(start_arrow);
        start_arrow = na; vis.AddGeometry(start_arrow); start_added = true;
      } else if (start_added && start_arrow) {
        vis.RemoveGeometry(start_arrow); start_arrow.reset(); start_added = false;
      }
      // Start cube
      if (arrows.has_start_cube) {
        auto nc = OctomapOpen3DViewerNode::makeCubeMesh(
          arrows.start_cube_center, arrows.start_cube_size, Eigen::Vector3d(0.1, 0.95, 0.1));
        if (start_cube_added && start_cube) vis.RemoveGeometry(start_cube);
        start_cube = nc; vis.AddGeometry(start_cube); start_cube_added = true;
      } else if (start_cube_added && start_cube) {
        vis.RemoveGeometry(start_cube); start_cube.reset(); start_cube_added = false;
      }
      // Goal arrow
      if (arrows.has_goal) {
        auto na = OctomapOpen3DViewerNode::makeArrowMesh(
          arrows.goal_base, arrows.goal_tip, Eigen::Vector3d(0.95, 0.1, 0.1));
        if (goal_added && goal_arrow) vis.RemoveGeometry(goal_arrow);
        goal_arrow = na; vis.AddGeometry(goal_arrow); goal_added = true;
      } else if (goal_added && goal_arrow) {
        vis.RemoveGeometry(goal_arrow); goal_arrow.reset(); goal_added = false;
      }
      // Goal cube
      if (arrows.has_goal_cube) {
        auto nc = OctomapOpen3DViewerNode::makeCubeMesh(
          arrows.goal_cube_center, arrows.goal_cube_size, Eigen::Vector3d(0.95, 0.1, 0.1));
        if (goal_cube_added && goal_cube) vis.RemoveGeometry(goal_cube);
        goal_cube = nc; vis.AddGeometry(goal_cube); goal_cube_added = true;
      } else if (goal_cube_added && goal_cube) {
        vis.RemoveGeometry(goal_cube); goal_cube.reset(); goal_cube_added = false;
      }
    }

    std::vector<Eigen::Vector3d> path_pts;
    if (node->consumePath(path_pts)) {
      if (path_pts.size() >= 2) {
        auto nl = std::make_shared<open3d::geometry::LineSet>();
        nl->points_ = path_pts;
        nl->lines_.reserve(path_pts.size() - 1);
        for (std::size_t i = 0; i + 1 < path_pts.size(); ++i)
          nl->lines_.push_back(Eigen::Vector2i(i, i + 1));
        nl->colors_.assign(nl->lines_.size(), Eigen::Vector3d(0.1, 0.95, 0.95));
        if (path_added && path_lines) vis.RemoveGeometry(path_lines);
        path_lines = nl; vis.AddGeometry(path_lines); path_added = true;
      } else if (path_added && path_lines) {
        vis.RemoveGeometry(path_lines); path_lines.reset(); path_added = false;
      }
    }

    if (!vis.PollEvents()) break;
    vis.UpdateRender();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  vis.DestroyVisualizerWindow();
  ros::shutdown();
  ros_thread.join();
  return 0;
}