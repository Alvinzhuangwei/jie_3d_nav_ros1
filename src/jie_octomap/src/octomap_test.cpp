#include <cmath>
#include <memory>
#include <random>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <visualization_msgs/Marker.h>

class OctomapTestNode
{
public:
  OctomapTestNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  {
    double resolution;
    pnh.param<double>("resolution", resolution, 0.2);
    pnh.param<std::string>("frame_id", frame_id_, "map");
    std::string octomap_topic, marker_topic;
    pnh.param<std::string>("octomap_topic", octomap_topic, "/octomap");
    pnh.param<std::string>("marker_topic", marker_topic, "/octomap_occupied_markers");

    octomap_pub_ = nh.advertise<octomap_msgs::Octomap>(octomap_topic, 1, true);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>(marker_topic, 1, true);

    generateMap(resolution);
    publishAll();
    timer_ = nh.createTimer(ros::Duration(1.0), &OctomapTestNode::onTimer, this);
  }

private:
  void onTimer(const ros::TimerEvent &) { publishAll(); }

  double groundZ(double x, double y) const { return 0.10 * x + 0.03 * y; }

  void addVerticalWallX(double x_fixed, double y_start, double y_end,
                        double wall_height, double res)
  {
    for (double y = y_start; y <= y_end; y += res) {
      const double base = groundZ(x_fixed, y);
      for (double z = base; z <= base + wall_height; z += res)
        tree_->updateNode(octomap::point3d(x_fixed, y, z), true);
    }
  }
  void addVerticalWallY(double y_fixed, double x_start, double x_end,
                        double wall_height, double res)
  {
    for (double x = x_start; x <= x_end; x += res) {
      const double base = groundZ(x, y_fixed);
      for (double z = base; z <= base + wall_height; z += res)
        tree_->updateNode(octomap::point3d(x, y_fixed, z), true);
    }
  }

  void generateMap(double resolution)
  {
    tree_ = std::make_shared<octomap::OcTree>(resolution);
    const double x_min = -8.0, x_max = 8.0, y_min = -6.0, y_max = 6.0;
    const int ix_min = static_cast<int>(std::floor(x_min / resolution));
    const int ix_max = static_cast<int>(std::ceil(x_max / resolution));
    const int iy_min = static_cast<int>(std::floor(y_min / resolution));
    const int iy_max = static_cast<int>(std::ceil(y_max / resolution));
    for (int ix = ix_min; ix <= ix_max; ++ix) {
      const double x = (static_cast<double>(ix) + 0.5) * resolution;
      for (int iy = iy_min; iy <= iy_max; ++iy) {
        const double y = (static_cast<double>(iy) + 0.5) * resolution;
        const double gz = groundZ(x, y);
        const int iz_center = static_cast<int>(std::floor(gz / resolution));
        for (int iz = iz_center - 2; iz <= iz_center; ++iz) {
          const double z = (static_cast<double>(iz) + 0.5) * resolution;
          tree_->updateNode(octomap::point3d(x, y, z), true);
        }
      }
    }
    addVerticalWallX(1.5, -4.0, -0.8, 2.8, resolution);
    addVerticalWallX(4.0, 0.5, 4.5, 2.3, resolution);
    addVerticalWallY(-2.0, -6.0, -2.5, 2.6, resolution);
    addVerticalWallY(3.0, -1.0, 5.5, 2.0, resolution);
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> dist_x(-5.5, 5.5);
    std::uniform_real_distribution<double> dist_y(-4.5, 4.5);
    for (int i = 0; i < 14; ++i) {
      const double px = dist_x(gen), py = dist_y(gen);
      const double base = groundZ(px, py);
      const double h = 0.8 + 0.15 * static_cast<double>(i % 5);
      for (double z = base; z <= base + h; z += resolution)
        tree_->updateNode(octomap::point3d(px, py, z), true);
    }
    tree_->updateInnerOccupancy();
    ROS_INFO("Generated random OctoMap for testing.");
  }

  void publishAll()
  {
    if (!tree_) return;
    const ros::Time stamp = ros::Time::now();
    octomap_msgs::Octomap octomap_msg;
    if (!octomap_msgs::binaryMapToMsg(*tree_, octomap_msg)) {
      ROS_ERROR("Failed to convert OcTree to octomap message.");
      return;
    }
    octomap_msg.header.stamp = stamp;
    octomap_msg.header.frame_id = frame_id_;
    octomap_pub_.publish(octomap_msg);

    visualization_msgs::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = frame_id_;
    marker.ns = "occupied_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = tree_->getResolution();
    marker.color.r = 0.95f; marker.color.g = 0.45f;
    marker.color.b = 0.15f; marker.color.a = 0.95f;
    marker.lifetime = ros::Duration(0.0);
    for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
      if (!tree_->isNodeOccupied(*it)) continue;
      geometry_msgs::Point p;
      p.x = it.getX(); p.y = it.getY(); p.z = it.getZ();
      marker.points.push_back(p);
    }
    marker_pub_.publish(marker);
    ROS_INFO_THROTTLE(3.0, "Published OctoMap (%zu occupied voxels).", marker.points.size());
  }

  std::shared_ptr<octomap::OcTree> tree_;
  ros::Timer timer_;
  ros::Publisher octomap_pub_;
  ros::Publisher marker_pub_;
  std::string frame_id_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "octomap_test");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  OctomapTestNode node(nh, pnh);
  ros::spin();
  return 0;
}