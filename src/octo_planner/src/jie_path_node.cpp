// jie_path_node.cpp  —  ROS 1 port of the original ROS 2 implementation
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "jie_map_msgs/ExportNavigationSnapshot.h"
#include "jie_map_msgs/GetNavigationMapMeta.h"
#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/Octomap.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "visualization_msgs/Marker.h"

namespace
{
struct GridIndex
{
  int x;
  int y;
  int z;

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridIndexHash
{
  std::size_t operator()(const GridIndex & k) const
  {
    const std::size_t h1 = std::hash<int>{}(k.x);
    const std::size_t h2 = std::hash<int>{}(k.y);
    const std::size_t h3 = std::hash<int>{}(k.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

struct QueueNode
{
  GridIndex idx;
  double f;
  double g;
};

struct QueueNodeCompare
{
  bool operator()(const QueueNode & a, const QueueNode & b) const
  {
    return a.f > b.f;
  }
};

double euclidean(const GridIndex & a, const GridIndex & b)
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  const double dz = static_cast<double>(a.z - b.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::uint64_t hashOctomapData(const std::vector<int8_t> & data)
{
  std::uint64_t h = 1469598103934665603ULL;
  for (const auto v : data) {
    h ^= static_cast<std::uint8_t>(v);
    h *= 1099511628211ULL;
  }
  return h;
}
}  // namespace

class JiePathNode
{
public:
  JiePathNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  : nh_(nh), pnh_(pnh),
    map_ready_(false),
    has_start_(false),
    has_goal_(false),
    has_goal_pose_(false),
    planning_in_progress_(false),
    plan_seq_(0),
    last_success_seq_(0),
    last_octomap_hash_(0)
  {
    // Parameters
    pnh_.param<std::string>("octomap_topic", octomap_topic_, "/octomap");
    pnh_.param<std::string>("start_topic", start_topic_, "/start_point");
    pnh_.param<std::string>("goal_topic", goal_topic_, "/goal_point");
    pnh_.param<std::string>("goal_pose_topic", goal_pose_topic_, "/goal_pose");
    pnh_.param<std::string>("path_topic", path_topic_, "/planned_path");
    pnh_.param<std::string>("path_marker_topic", path_marker_topic_, "/planned_path_marker");
    pnh_.param<std::string>("preblocked_marker_topic", preblocked_marker_topic_, "/preblocked_cells_markers");
    pnh_.param<std::string>("external_preblocked_marker_topic", external_preblocked_marker_topic_, "/edited_preblocked_cells_markers");
    pnh_.param<std::string>("edited_occupied_marker_topic", edited_occupied_marker_topic_, "/edited_occupied_markers");
    pnh_.param<std::string>("traversable_marker_topic", traversable_marker_topic_, "/traversable_cells_markers");
    pnh_.param<std::string>("risk_cost_topic", risk_cost_topic_, "/risk_cost_cells");
    pnh_.param<std::string>("frame_id", frame_id_, "map");
    pnh_.param<double>("robot_radius", robot_radius_, 0.20);
    pnh_.param<int>("max_iterations", max_iterations_, 250000);
    pnh_.param<int>("snap_search_radius_cells", snap_search_radius_cells_, 8);
    pnh_.param<bool>("require_ground_support", require_ground_support_, true);
    pnh_.param<bool>("strict_direct_ground_support", strict_direct_ground_support_, true);
    pnh_.param<int>("ground_support_xy_radius_cells", ground_support_xy_radius_cells_, 1);
    pnh_.param<int>("ground_support_depth_cells", ground_support_depth_cells_, 2);
    pnh_.param<bool>("enable_preblocked_costmap", enable_preblocked_costmap_, true);
    pnh_.param<int>("preblocked_costmap_radius_cells", preblocked_costmap_radius_cells_, 3);
    pnh_.param<double>("preblocked_costmap_weight", preblocked_costmap_weight_, 1.5);
    pnh_.param<bool>("lowest_traversable_only", lowest_traversable_only_, false);
    pnh_.param<std::string>("map_id", map_id_, "navigation_map");
    pnh_.param<std::string>("source_world_file", source_world_file_, "");

    // Subscribers (latching = transient_local equivalent in ROS1)
    octomap_sub_ = nh_.subscribe(octomap_topic_, 1, &JiePathNode::onOctomap, this);
    start_sub_   = nh_.subscribe(start_topic_, 1, &JiePathNode::onStart, this);
    goal_sub_    = nh_.subscribe(goal_topic_, 1, &JiePathNode::onGoal, this);
    goal_pose_sub_ = nh_.subscribe(goal_pose_topic_, 1, &JiePathNode::onGoalPose, this);
    external_preblocked_sub_ = nh_.subscribe(
      external_preblocked_marker_topic_, 1, &JiePathNode::onExternalPreblockedMarker, this);
    edited_occupied_sub_ = nh_.subscribe(
      edited_occupied_marker_topic_, 1, &JiePathNode::onEditedOccupiedMarker, this);

    // Publishers (latch=true mimics transient_local)
    path_pub_             = nh_.advertise<nav_msgs::Path>(path_topic_, 1, /*latch=*/true);
    path_marker_pub_      = nh_.advertise<visualization_msgs::Marker>(path_marker_topic_, 1, true);
    octomap_pub_          = nh_.advertise<octomap_msgs::Octomap>(octomap_topic_, 1, true);
    preblocked_marker_pub_= nh_.advertise<visualization_msgs::Marker>(preblocked_marker_topic_, 1, true);
    traversable_marker_pub_= nh_.advertise<visualization_msgs::Marker>(traversable_marker_topic_, 1, true);
    risk_cost_pub_        = nh_.advertise<sensor_msgs::PointCloud2>(risk_cost_topic_, 1, true);

    // Services (use private node handle so they appear as ~/get_meta etc.)
    get_meta_srv_ = pnh_.advertiseService("get_meta", &JiePathNode::handleGetMapMeta, this);
    export_snapshot_srv_ = pnh_.advertiseService("export_snapshot", &JiePathNode::handleExportSnapshot, this);

    ROS_INFO(
      "jie_path_node started. octomap=%s start=%s goal=%s path=%s "
      "preblocked_marker=%s edited_occupied=%s meta_service=~/get_meta export_service=~/export_snapshot",
      octomap_topic_.c_str(), start_topic_.c_str(), goal_topic_.c_str(), path_topic_.c_str(),
      preblocked_marker_topic_.c_str(), edited_occupied_marker_topic_.c_str());
  }

private:
  // ---- helpers ----
  ros::Time now() const { return ros::Time::now(); }

  void fillBounds(geometry_msgs::Point & min_bound, geometry_msgs::Point & max_bound) const
  {
    if (!octree_) return;
    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);
    min_bound.x = min_x; min_bound.y = min_y; min_bound.z = min_z;
    max_bound.x = max_x; max_bound.y = max_y; max_bound.z = max_z;
  }

  // ---- services ----
  bool handleGetMapMeta(
    jie_map_msgs::GetNavigationMapMeta::Request & /*req*/,
    jie_map_msgs::GetNavigationMapMeta::Response & res)
  {
    res.success = map_ready_ && static_cast<bool>(octree_);
    res.message = res.success ? "ok" : "octomap not ready";
    res.map_id  = map_id_;
    res.frame_id = frame_id_;
    res.resolution = octree_ ? octree_->getResolution() : 0.0;
    fillBounds(res.min_bound, res.max_bound);
    res.robot_radius = robot_radius_;
    res.snap_search_radius_cells = snap_search_radius_cells_;
    res.require_ground_support = require_ground_support_;
    res.strict_direct_ground_support = strict_direct_ground_support_;
    res.ground_support_xy_radius_cells = ground_support_xy_radius_cells_;
    res.ground_support_depth_cells = ground_support_depth_cells_;
    res.enable_preblocked_costmap = enable_preblocked_costmap_;
    res.preblocked_costmap_radius_cells = preblocked_costmap_radius_cells_;
    res.preblocked_costmap_weight = preblocked_costmap_weight_;
    res.source_world_file = source_world_file_;
    return true;
  }

  bool handleExportSnapshot(
    jie_map_msgs::ExportNavigationSnapshot::Request & req,
    jie_map_msgs::ExportNavigationSnapshot::Response & res)
  {
    if (!map_ready_ || !octree_) {
      res.success = false;
      res.message = "octomap not ready";
      res.snapshot_stamp = now();
      return true;
    }
    if (req.recompute_layers) {
      rebuildPreblockedCells();
      rebuildDerivedLayers();
      rebuildPreblockedCostmap();
    } else {
      publishPreblockedCellsMarker();
      publishCellSetMarker(traversable_cells_, traversable_marker_pub_, "traversable_cells", 0.20F, 0.95F, 0.55F, 0.55F);
      publishRiskCostCloud();
    }
    res.success = true;
    res.message = "snapshot ready";
    res.snapshot_stamp = now();
    return true;
  }

  // ---- subscribers ----
  void onOctomap(const octomap_msgs::Octomap::ConstPtr & msg)
  {
    const std::uint64_t map_hash = hashOctomapData(msg->data);
    if (map_ready_ && map_hash == last_octomap_hash_) return;

    octree_.reset(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg)));
    if (!octree_) {
      ROS_ERROR("Failed to convert OctoMap message to OcTree.");
      return;
    }
    map_ready_ = true;
    last_octomap_hash_ = map_hash;
    rebuildPreblockedCells();
    rebuildDerivedLayers();
    rebuildPreblockedCostmap();
  }

  void onEditedOccupiedMarker(const visualization_msgs::Marker::ConstPtr & msg)
  {
    if (msg->type != visualization_msgs::Marker::CUBE_LIST) {
      ROS_WARN("Ignored edited occupied marker because it is not CUBE_LIST.");
      return;
    }
    const double resolution = markerResolution(*msg);
    if (resolution <= 0.0) {
      ROS_WARN("Ignored edited occupied marker because scale is invalid.");
      return;
    }
    auto edited_tree = std::make_shared<octomap::OcTree>(resolution);
    for (const auto & point : msg->points) {
      edited_tree->updateNode(
        octomap::point3d(static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z)),
        true);
    }
    edited_tree->updateInnerOccupancy();
    octree_ = edited_tree;
    map_ready_ = true;
    last_octomap_hash_ = 0;
    if (!msg->header.frame_id.empty()) frame_id_ = msg->header.frame_id;

    publishCurrentOctomap();
    rebuildPreblockedCells();
    rebuildDerivedLayers();
    rebuildPreblockedCostmap();

    ROS_INFO("Edited occupied marker applied. occupied_cells=%zu resolution=%.3f",
      msg->points.size(), resolution);

    if (has_start_ && has_goal_) {
      if (!planAndPublish()) ROS_WARN("No path found after edited occupied map refresh.");
    }
  }

  static double markerResolution(const visualization_msgs::Marker & msg)
  {
    const double sx = msg.scale.x > 0.0 ? msg.scale.x : 0.0;
    const double sy = msg.scale.y > 0.0 ? msg.scale.y : 0.0;
    const double sz = msg.scale.z > 0.0 ? msg.scale.z : 0.0;
    if (sx <= 0.0 && sy <= 0.0 && sz <= 0.0) return 0.0;
    if (sx > 0.0) return sx;
    if (sy > 0.0) return sy;
    return sz;
  }

  void publishCurrentOctomap()
  {
    if (!octree_) return;
    octomap_msgs::Octomap msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    if (!octomap_msgs::fullMapToMsg(*octree_, msg)) {
      ROS_WARN("Failed to publish edited OctoMap message.");
      return;
    }
    octomap_pub_.publish(msg);
  }

  void onStart(const geometry_msgs::PointStamped::ConstPtr & msg)
  {
    start_point_ = *msg;
    has_start_ = true;
    ROS_INFO("Start set to [%.3f, %.3f, %.3f]", msg->point.x, msg->point.y, msg->point.z);
  }

  void onGoal(const geometry_msgs::PointStamped::ConstPtr & msg)
  {
    goal_point_ = *msg;
    has_goal_ = true;
    ROS_INFO("Goal set to [%.3f, %.3f, %.3f]", msg->point.x, msg->point.y, msg->point.z);
    tryPlan();
  }

  void onGoalPose(const geometry_msgs::PoseStamped::ConstPtr & msg)
  {
    goal_pose_ = *msg;
    has_goal_pose_ = true;
    const double yaw = std::atan2(
      2.0 * (msg->pose.orientation.w * msg->pose.orientation.z +
      msg->pose.orientation.x * msg->pose.orientation.y),
      1.0 - 2.0 * (msg->pose.orientation.y * msg->pose.orientation.y +
      msg->pose.orientation.z * msg->pose.orientation.z));
    ROS_INFO("Goal pose yaw set to %.1f deg in frame %s",
      yaw * 180.0 / M_PI,
      msg->header.frame_id.empty() ? frame_id_.c_str() : msg->header.frame_id.c_str());
    tryPlan();
  }

  void tryPlan()
  {
    if (!map_ready_ || !has_start_ || !has_goal_ || planning_in_progress_) return;
    planning_in_progress_ = true;
    ++plan_seq_;
    const bool ok = planAndPublish();
    planning_in_progress_ = false;
    if (!ok) {
      ROS_WARN_THROTTLE(2.0, "\033[1;31mA* planning failed.\033[0m");
    } else {
      last_success_seq_ = plan_seq_;
    }
  }

  void onExternalPreblockedMarker(const visualization_msgs::Marker::ConstPtr & msg)
  {
    external_preblocked_cells_.clear();
    if (!octree_) return;
    for (const auto & point : msg->points) {
      const GridIndex idx = worldToGrid(point.x, point.y, point.z);
      if (isInsideMetricBounds(idx) && !isOccupiedCell(idx))
        external_preblocked_cells_.insert(idx);
    }
    ROS_INFO("Received external preblocked marker. cells=%zu", external_preblocked_cells_.size());
    rebuildPreblockedCells();
    rebuildDerivedLayers();
    rebuildPreblockedCostmap();
    if (map_ready_ && has_start_ && has_goal_) {
      if (!planAndPublish()) ROS_WARN("No path found after external preblocked update.");
    }
  }

  // ---- grid helpers ----
  GridIndex worldToGrid(double x, double y, double z) const
  {
    const double r = octree_->getResolution();
    return GridIndex{
      static_cast<int>(std::floor(x / r)),
      static_cast<int>(std::floor(y / r)),
      static_cast<int>(std::floor(z / r))};
  }

  octomap::point3d gridToWorld(const GridIndex & idx) const
  {
    const double r = octree_->getResolution();
    return octomap::point3d(
      static_cast<float>((static_cast<double>(idx.x) + 0.5) * r),
      static_cast<float>((static_cast<double>(idx.y) + 0.5) * r),
      static_cast<float>((static_cast<double>(idx.z) + 0.5) * r));
  }

  bool isInsideMetricBounds(const GridIndex & idx) const
  {
    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);
    const auto p = gridToWorld(idx);
    return p.x() >= static_cast<float>(min_x) && p.x() <= static_cast<float>(max_x) &&
           p.y() >= static_cast<float>(min_y) && p.y() <= static_cast<float>(max_y) &&
           p.z() >= static_cast<float>(min_z) && p.z() <= static_cast<float>(max_z);
  }

  bool hasGroundSupport(const GridIndex & idx, bool strict, int xy_r, int depth) const
  {
    if (strict) {
      GridIndex below{idx.x, idx.y, idx.z - 1};
      if (!isInsideMetricBounds(below)) return false;
      const auto p = gridToWorld(below);
      const octomap::OcTreeNode * node = octree_->search(p);
      return node && octree_->isNodeOccupied(node);
    }
    for (int dz = 1; dz <= std::max(1, depth); ++dz) {
      for (int dx = -xy_r; dx <= xy_r; ++dx) {
        for (int dy = -xy_r; dy <= xy_r; ++dy) {
          GridIndex below{idx.x + dx, idx.y + dy, idx.z - dz};
          if (!isInsideMetricBounds(below)) continue;
          const auto p = gridToWorld(below);
          const octomap::OcTreeNode * node = octree_->search(p);
          if (node && octree_->isNodeOccupied(node)) return true;
        }
      }
    }
    return false;
  }

  bool isOccupiedCell(const GridIndex & idx) const
  {
    if (!isInsideMetricBounds(idx)) return false;
    const auto p = gridToWorld(idx);
    const octomap::OcTreeNode * node = octree_->search(p);
    return node && octree_->isNodeOccupied(node);
  }

  bool hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const
  {
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;
        const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
        if (!isInsideMetricBounds(n)) continue;
        if (!isOccupiedCell(n)) return true;
      }
    return false;
  }

  bool hasSameLevelNeighborWithOccupiedBelow(const GridIndex & idx) const
  {
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;
        const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
        if (!isInsideMetricBounds(n)) continue;
        const GridIndex n_below{n.x, n.y, n.z - 1};
        if (!isInsideMetricBounds(n_below)) continue;
        if (isOccupiedCell(n_below)) return true;
      }
    return false;
  }

  bool hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const
  {
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;
        const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
        if (!isInsideMetricBounds(n)) continue;
        const GridIndex n_above1{n.x, n.y, n.z + 1};
        if (!isInsideMetricBounds(n_above1)) continue;
        if (isOccupiedCell(n_above1)) return true;
      }
    return false;
  }

  // ---- layer rebuilds ----
  void rebuildPreblockedCells()
  {
    preblocked_cells_.clear();
    if (!octree_) return;

    std::unordered_set<GridIndex, GridIndexHash> candidates;
    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
      if (!octree_->isNodeOccupied(*it)) continue;
      const GridIndex occ = worldToGrid(it.getX(), it.getY(), it.getZ());
      for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;
          candidates.insert(GridIndex{occ.x + dx, occ.y + dy, occ.z});
        }
    }

    for (const auto & c : candidates) {
      if (!isInsideMetricBounds(c) || isOccupiedCell(c)) continue;
      const GridIndex below0{c.x, c.y, c.z - 1};
      const bool below0_occ = isInsideMetricBounds(below0) && isOccupiedCell(below0);
      if (below0_occ && hasSameLevelNeighborWithOccupiedAbove(c)) {
        preblocked_cells_.insert(c);
        continue;
      }
      const GridIndex above1{c.x, c.y, c.z + 1};
      const bool above1_occ = isInsideMetricBounds(above1) && isOccupiedCell(above1);
      if (!hasNonOccupiedNeighborSameLevel(c)) continue;
      if (above1_occ) continue;
      const GridIndex below1{c.x, c.y, c.z - 1};
      if (!isInsideMetricBounds(below1)) continue;
      if (!isOccupiedCell(below1)) preblocked_cells_.insert(c);
    }

    for (const auto & c : external_preblocked_cells_) {
      if (isInsideMetricBounds(c) && !isOccupiedCell(c))
        preblocked_cells_.insert(c);
    }

    ROS_INFO("Preprocess mask rebuilt. preblocked_cells=%zu external=%zu",
      preblocked_cells_.size(), external_preblocked_cells_.size());
    publishPreblockedCellsMarker();
  }

  void rebuildPreblockedCostmap()
  {
    preblocked_costmap_.clear();
    if (!octree_ || !enable_preblocked_costmap_) return;

    const int radius_cells = std::max(1, preblocked_costmap_radius_cells_);
    const double denom = static_cast<double>(radius_cells) + 1.0;

    for (const auto & c : preblocked_cells_) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx)
        for (int dy = -radius_cells; dy <= radius_cells; ++dy)
          for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) continue;
            const GridIndex n{c.x + dx, c.y + dy, c.z + dz};
            if (!isInsideMetricBounds(n)) continue;
            if (traversable_cells_.find(n) == traversable_cells_.end()) continue;
            if (preblocked_cells_.find(n) != preblocked_cells_.end()) continue;
            const double d = std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
            if (d > static_cast<double>(radius_cells)) continue;
            const double cst = std::max(0.0, (denom - d) / denom);
            auto it = preblocked_costmap_.find(n);
            if (it == preblocked_costmap_.end() || cst > it->second)
              preblocked_costmap_[n] = cst;
          }
    }

    ROS_INFO("Preblocked costmap rebuilt. cells=%zu radius=%d",
      preblocked_costmap_.size(), radius_cells);
    publishRiskCostCloud();
  }

  double getPreblockedCost(const GridIndex & idx) const
  {
    const auto it = preblocked_costmap_.find(idx);
    return it == preblocked_costmap_.end() ? 0.0 : it->second;
  }

  void rebuildDerivedLayers()
  {
    traversable_cells_.clear();
    if (!octree_) return;

    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);
    const GridIndex min_idx = worldToGrid(min_x, min_y, min_z);
    const GridIndex max_idx = worldToGrid(max_x, max_y, max_z);

    for (int x = min_idx.x; x <= max_idx.x; ++x)
      for (int y = min_idx.y; y <= max_idx.y; ++y)
        for (int z = min_idx.z; z <= max_idx.z; ++z) {
          const GridIndex idx{x, y, z};
          if (!isInsideMetricBounds(idx) || isOccupiedCell(idx)) continue;
          if (isCellTraversable(idx, robot_radius_, require_ground_support_,
                strict_direct_ground_support_, ground_support_xy_radius_cells_,
                ground_support_depth_cells_))
          {
            traversable_cells_.insert(idx);
            if (lowest_traversable_only_) break;
          }
        }

    publishCellSetMarker(traversable_cells_, traversable_marker_pub_, "traversable_cells",
      0.20F, 0.95F, 0.55F, 0.55F);
  }

  bool isCellTraversable(const GridIndex & idx, double robot_radius,
    bool require_ground_support, bool strict, int xy_r, int depth) const
  {
    if (!isInsideMetricBounds(idx)) return false;
    if (require_ground_support && !hasGroundSupport(idx, strict, xy_r, depth)) return false;

    for (int z = idx.z - 1; z >= 0; --z) {
      const GridIndex below_idx{idx.x, idx.y, z};
      if (isOccupiedCell(below_idx)) break;
      if (preblocked_cells_.find(below_idx) != preblocked_cells_.end()) return false;
    }

    const octomap::point3d center = gridToWorld(idx);
    const double r = octree_->getResolution();
    const int n = std::max(1, static_cast<int>(std::ceil(robot_radius / r)));
    const double radius_sq = robot_radius * robot_radius;

    for (int dx = -n; dx <= n; ++dx)
      for (int dy = -n; dy <= n; ++dy)
        for (int dz = 0; dz <= n; ++dz) {
          const double dist_sq =
            (dx * r) * (dx * r) + (dy * r) * (dy * r) + (dz * r) * (dz * r);
          if (dist_sq > radius_sq) continue;
          const octomap::point3d p(
            center.x() + static_cast<float>(dx * r),
            center.y() + static_cast<float>(dy * r),
            center.z() + static_cast<float>(dz * r));
          const GridIndex nearby_idx = worldToGrid(p.x(), p.y(), p.z());
          if (preblocked_cells_.find(nearby_idx) != preblocked_cells_.end()) return false;
          const octomap::OcTreeNode * node = octree_->search(p);
          if (node && octree_->isNodeOccupied(node)) return false;
        }
    return true;
  }

  bool findNearestFreeCell(const GridIndex & seed, double robot_radius, int radius_cells,
    bool require_ground_support, bool strict, int xy_r, int depth, GridIndex & out) const
  {
    if (isCellTraversable(seed, robot_radius, require_ground_support, strict, xy_r, depth)) {
      out = seed; return true;
    }
    for (int r = 1; r <= radius_cells; ++r)
      for (int dz = 0; dz <= r; ++dz)
        for (int dx = -r; dx <= r; ++dx)
          for (int dy = -r; dy <= r; ++dy) {
            if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) continue;
            GridIndex c1{seed.x + dx, seed.y + dy, seed.z + dz};
            if (isCellTraversable(c1, robot_radius, require_ground_support, strict, xy_r, depth)) {
              out = c1; return true;
            }
            if (dz > 0) {
              GridIndex c2{seed.x + dx, seed.y + dy, seed.z - dz};
              if (isCellTraversable(c2, robot_radius, require_ground_support, strict, xy_r, depth)) {
                out = c2; return true;
              }
            }
          }
    return false;
  }

  std::vector<GridIndex> make26Directions() const
  {
    std::vector<GridIndex> dirs;
    dirs.reserve(26);
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) continue;
          dirs.push_back(GridIndex{dx, dy, dz});
        }
    return dirs;
  }

  std::vector<GridIndex> reconstructPath(
    const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
    GridIndex current) const
  {
    std::vector<GridIndex> path;
    path.push_back(current);
    while (came_from.find(current) != came_from.end()) {
      current = came_from.at(current);
      path.push_back(current);
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  bool planAndPublish()
  {
    const uint64_t this_plan_seq = plan_seq_;  // 记录本次规划的序列号
    const GridIndex start_raw = worldToGrid(
      start_point_.point.x, start_point_.point.y, start_point_.point.z);
    const GridIndex goal_raw = worldToGrid(
      goal_point_.point.x, goal_point_.point.y, goal_point_.point.z);

    GridIndex start = start_raw, goal = goal_raw;
    if (!findNearestFreeCell(start_raw, robot_radius_, snap_search_radius_cells_,
          require_ground_support_, strict_direct_ground_support_,
          ground_support_xy_radius_cells_, ground_support_depth_cells_, start))
    {
      ROS_ERROR_THROTTLE(2.0, "Start is occupied/out of map and no nearby free cell.");
      return false;
    }
    if (!findNearestFreeCell(goal_raw, robot_radius_, snap_search_radius_cells_,
          require_ground_support_, strict_direct_ground_support_,
          ground_support_xy_radius_cells_, ground_support_depth_cells_, goal))
    {
      ROS_ERROR_THROTTLE(2.0, "Goal is occupied/out of map and no nearby free cell.");
      return false;
    }

    if (!(start == start_raw)) {
      const auto p = gridToWorld(start);
      ROS_INFO_THROTTLE(2.0, "Start snapped to free cell: [%.2f, %.2f, %.2f]", p.x(), p.y(), p.z());
    }
    if (!(goal == goal_raw)) {
      const auto p = gridToWorld(goal);
      ROS_INFO_THROTTLE(2.0, "Goal snapped to free cell: [%.2f, %.2f, %.2f]", p.x(), p.y(), p.z());
    }

    std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;
    std::unordered_map<GridIndex, double, GridIndexHash> g_score;
    std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
    std::unordered_set<GridIndex, GridIndexHash> closed_set;

    g_score[start] = 0.0;
    open_set.push(QueueNode{start, euclidean(start, goal), 0.0});
    const std::vector<GridIndex> directions = make26Directions();
    int iters = 0;

    while (!open_set.empty() && iters < max_iterations_) {
      const QueueNode current = open_set.top();
      open_set.pop();
      ++iters;
      if (closed_set.find(current.idx) != closed_set.end()) continue;
      closed_set.insert(current.idx);

      if (current.idx == goal) {
        // 只有序列号匹配时才发布（避免旧规划覆盖新规划）
        if (this_plan_seq == plan_seq_) {
            const auto cells = reconstructPath(came_from, current.idx);
            publishPath(cells, frame_id_);
            ROS_INFO("\033[1;32mA* path found in %d iterations. waypoints=%zu\033[0m", 
                     iters, cells.size());
            return true;
        }
    }

      for (const auto & d : directions) {
        GridIndex nbr{current.idx.x + d.x, current.idx.y + d.y, current.idx.z + d.z};
        if (closed_set.find(nbr) != closed_set.end()) continue;
        if (!isCellTraversable(nbr, robot_radius_, require_ground_support_,
              strict_direct_ground_support_, ground_support_xy_radius_cells_,
              ground_support_depth_cells_))
          continue;

        double tentative_g = current.g + euclidean(current.idx, nbr);
        if (enable_preblocked_costmap_)
          tentative_g += preblocked_costmap_weight_ * getPreblockedCost(nbr);

        auto g_it = g_score.find(nbr);
        if (g_it == g_score.end() || tentative_g < g_it->second) {
          came_from[nbr] = current.idx;
          g_score[nbr] = tentative_g;
          open_set.push(QueueNode{nbr, tentative_g + euclidean(nbr, goal), tentative_g});
        }
      }
    }
    return false;
  }

  void publishPath(const std::vector<GridIndex> & cells, const std::string & frame_id)
  {
    nav_msgs::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = frame_id;
    path_msg.poses.reserve(cells.size());

    visualization_msgs::Marker m;
    m.header = path_msg.header;
    m.ns = "jie_path";
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.32;
    m.color.r = 0.1F; m.color.g = 0.95F; m.color.b = 0.95F; m.color.a = 1.0F;
    m.pose.orientation.w = 1.0;

    for (std::size_t i = 0; i < cells.size(); ++i) {
      const auto p = gridToWorld(cells[i]);
      geometry_msgs::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();
      pose.pose.orientation.w = 1.0;
      if (has_goal_pose_ && i + 1 == cells.size())
        pose.pose.orientation = goal_pose_.pose.orientation;
      path_msg.poses.push_back(pose);

      geometry_msgs::Point q;
      q.x = p.x(); q.y = p.y(); q.z = p.z();
      m.points.push_back(q);
    }

    path_pub_.publish(path_msg);
    path_marker_pub_.publish(m);
  }

  // ---- marker/cloud publishing ----
  void publishCellSetMarker(
    const std::unordered_set<GridIndex, GridIndexHash> & cells,
    ros::Publisher & publisher,
    const std::string & ns,
    float r_color, float g_color, float b_color, float a_color) const
  {
    if (!octree_) return;
    visualization_msgs::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = frame_id_;
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    const double resolution = octree_->getResolution();
    marker.scale.x = resolution;
    marker.scale.y = resolution;
    marker.scale.z = resolution;
    marker.color.r = r_color;
    marker.color.g = g_color;
    marker.color.b = b_color;
    marker.color.a = a_color;
    marker.points.reserve(cells.size());
    for (const auto & c : cells) {
      const auto p = gridToWorld(c);
      geometry_msgs::Point q;
      q.x = p.x(); q.y = p.y(); q.z = p.z();
      marker.points.push_back(q);
    }
    publisher.publish(marker);
  }

  void publishPreblockedCellsMarker()
  {
    publishCellSetMarker(preblocked_cells_, preblocked_marker_pub_,
      "preblocked_cells", 0.15F, 0.35F, 1.0F, 0.95F);
  }

  void publishRiskCostCloud() const
  {
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = now();
    cloud_msg.header.frame_id = frame_id_;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(preblocked_costmap_.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(cloud_msg, "intensity");

    for (const auto & entry : preblocked_costmap_) {
      const auto p = gridToWorld(entry.first);
      *iter_x = p.x(); *iter_y = p.y(); *iter_z = p.z();
      *iter_i = static_cast<float>(entry.second);
      ++iter_x; ++iter_y; ++iter_z; ++iter_i;
    }
    risk_cost_pub_.publish(cloud_msg);
  }

  // ---- node handles ----
  ros::NodeHandle & nh_;
  ros::NodeHandle & pnh_;

  // ---- parameters ----
  std::string octomap_topic_, start_topic_, goal_topic_, goal_pose_topic_;
  std::string path_topic_, path_marker_topic_, preblocked_marker_topic_;
  std::string external_preblocked_marker_topic_, edited_occupied_marker_topic_;
  std::string traversable_marker_topic_, risk_cost_topic_;
  std::string frame_id_, map_id_, source_world_file_;
  double robot_radius_;
  int max_iterations_, snap_search_radius_cells_;
  bool require_ground_support_, strict_direct_ground_support_;
  int ground_support_xy_radius_cells_, ground_support_depth_cells_;
  bool enable_preblocked_costmap_;
  int preblocked_costmap_radius_cells_;
  double preblocked_costmap_weight_;
  bool lowest_traversable_only_;

  // ---- state ----
  bool map_ready_, has_start_, has_goal_, has_goal_pose_, planning_in_progress_;
  std::uint64_t plan_seq_, last_success_seq_, last_octomap_hash_;
  geometry_msgs::PointStamped start_point_, goal_point_;
  geometry_msgs::PoseStamped goal_pose_;
  std::shared_ptr<octomap::OcTree> octree_;
  std::unordered_set<GridIndex, GridIndexHash> traversable_cells_;
  std::unordered_set<GridIndex, GridIndexHash> preblocked_cells_;
  std::unordered_set<GridIndex, GridIndexHash> external_preblocked_cells_;
  std::unordered_map<GridIndex, double, GridIndexHash> preblocked_costmap_;

  // ---- ros handles ----
  ros::Subscriber octomap_sub_, start_sub_, goal_sub_, goal_pose_sub_;
  ros::Subscriber external_preblocked_sub_, edited_occupied_sub_;
  ros::Publisher path_pub_, path_marker_pub_, octomap_pub_;
  ros::Publisher preblocked_marker_pub_, traversable_marker_pub_, risk_cost_pub_;
  ros::ServiceServer get_meta_srv_, export_snapshot_srv_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "jie_path_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  JiePathNode node(nh, pnh);
  ros::spin();
  return 0;
}