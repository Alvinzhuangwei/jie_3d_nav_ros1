#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

struct XYKey
{
  int x;
  int y;
  bool operator==(const XYKey & o) const { return x == o.x && y == o.y; }
};

struct XYKeyHash
{
  std::size_t operator()(const XYKey & k) const
  {
    return std::hash<int>{}(k.x) ^ (std::hash<int>{}(k.y) << 1);
  }
};

class OccupancyGridToOctomapNode
{
public:
  OccupancyGridToOctomapNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  {
    std::string grid_topic, octomap_topic, frame_id;
    pnh.param<std::string>("grid_topic", grid_topic, "/import_occupancy_grid");
    pnh.param<std::string>("octomap_topic", octomap_topic, "/octomap");
    pnh.param<std::string>("frame_id", frame_id_, "map");
    pnh.param<double>("octomap_resolution", octomap_resolution_, 0.2);
    pnh.param<double>("wall_height_m", wall_height_m_, 1.0);
    pnh.param<double>("floor_z_m", floor_z_m_, 0.0);
    pnh.param<int>("occupied_threshold", occupied_threshold_, 50);

    octomap_pub_ = nh.advertise<octomap_msgs::Octomap>(octomap_topic, 1, /*latch=*/true);
    grid_sub_ = nh.subscribe(grid_topic, 1, &OccupancyGridToOctomapNode::onGrid, this);

    ROS_INFO("occupancy_grid_to_octomap started. grid_topic=%s octomap_topic=%s",
             grid_topic.c_str(), octomap_topic.c_str());
  }

private:
  void onGrid(const nav_msgs::OccupancyGrid::ConstPtr & msg)
  {
    const double grid_resolution = msg->info.resolution;
    if (grid_resolution <= 0.0 || octomap_resolution_ <= 0.0) {
      ROS_ERROR("Grid and OctoMap resolutions must be positive.");
      return;
    }

    octomap::OcTree tree(octomap_resolution_);
    const double wall_height = std::max(octomap_resolution_, wall_height_m_);
    const int height_cells = std::max(1, static_cast<int>(std::ceil(wall_height / octomap_resolution_)));

    const auto & origin = msg->info.origin.position;
    const std::size_t width = msg->info.width;
    const std::size_t height = msg->info.height;

    std::unordered_set<XYKey, XYKeyHash> known_cells;
    std::unordered_set<XYKey, XYKeyHash> occupied_cells;

    for (std::size_t y = 0; y < height; ++y) {
      for (std::size_t x = 0; x < width; ++x) {
        const std::size_t index = y * width + x;
        if (index >= msg->data.size()) continue;
        const int8_t value = msg->data[index];
        if (value < 0) continue;

        const double world_x = origin.x + (static_cast<double>(x) + 0.5) * grid_resolution;
        const double world_y = origin.y + (static_cast<double>(y) + 0.5) * grid_resolution;
        const int grid_x = static_cast<int>(std::floor(world_x / octomap_resolution_));
        const int grid_y = static_cast<int>(std::floor(world_y / octomap_resolution_));
        const XYKey key{grid_x, grid_y};
        known_cells.insert(key);
        if (value >= occupied_threshold_) occupied_cells.insert(key);
      }
    }

    for (const auto & key : known_cells) {
      const double wx = (static_cast<double>(key.x) + 0.5) * octomap_resolution_;
      const double wy = (static_cast<double>(key.y) + 0.5) * octomap_resolution_;
      tree.updateNode(wx, wy, floor_z_m_ + 0.5 * octomap_resolution_, true);
    }
    for (const auto & key : occupied_cells) {
      const double wx = (static_cast<double>(key.x) + 0.5) * octomap_resolution_;
      const double wy = (static_cast<double>(key.y) + 0.5) * octomap_resolution_;
      for (int z = 1; z <= height_cells; ++z) {
        const double wz = floor_z_m_ + (static_cast<double>(z) + 0.5) * octomap_resolution_;
        tree.updateNode(wx, wy, wz, true);
      }
    }
    tree.updateInnerOccupancy();

    octomap_msgs::Octomap octomap_msg;
    octomap_msg.header.stamp = ros::Time::now();
    octomap_msg.header.frame_id = msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id;

    if (!octomap_msgs::binaryMapToMsg(tree, octomap_msg)) {
      ROS_ERROR("Failed to convert OcTree to Octomap message.");
      return;
    }
    octomap_pub_.publish(octomap_msg);
    ROS_INFO("Published OctoMap from OccupancyGrid. width=%zu height=%zu occupied=%zu",
             width, height, occupied_cells.size());
  }

  ros::Subscriber grid_sub_;
  ros::Publisher octomap_pub_;
  std::string frame_id_;
  double octomap_resolution_;
  double wall_height_m_;
  double floor_z_m_;
  int occupied_threshold_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "occupancy_grid_to_octomap");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  OccupancyGridToOctomapNode node(nh, pnh);
  ros::spin();
  return 0;
}