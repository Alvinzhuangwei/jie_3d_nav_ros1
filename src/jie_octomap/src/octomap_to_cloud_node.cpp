#include <string>

#include <ros/ros.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class OctomapToCloudNode
{
public:
  OctomapToCloudNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  {
    std::string octomap_topic, cloud_topic;
    pnh.param<std::string>("octomap_topic", octomap_topic, "/octomap");
    pnh.param<std::string>("cloud_topic", cloud_topic, "/octomap_points");
    pnh.param<std::string>("frame_id", frame_id_, "map");

    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic, 1, /*latch=*/true);
    octomap_sub_ = nh.subscribe(octomap_topic, 1, &OctomapToCloudNode::onOctomap, this);

    ROS_INFO("octomap_to_cloud started. octomap=%s cloud=%s",
             octomap_topic.c_str(), cloud_topic.c_str());
  }

private:
  void onOctomap(const octomap_msgs::Octomap::ConstPtr & msg)
  {
    std::unique_ptr<octomap::AbstractOcTree> tree_ptr(octomap_msgs::msgToMap(*msg));
    if (!tree_ptr) { ROS_ERROR("Failed to decode octomap message."); return; }
    auto * oc_tree = dynamic_cast<octomap::OcTree *>(tree_ptr.get());
    if (!oc_tree) { ROS_ERROR("Decoded map is not octomap::OcTree."); return; }

    std::size_t occupied_count = 0;
    for (auto it = oc_tree->begin_leafs(); it != oc_tree->end_leafs(); ++it)
      if (oc_tree->isNodeOccupied(*it)) ++occupied_count;

    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = msg->header.stamp;
    cloud_msg.header.frame_id = msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(occupied_count);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (auto it = oc_tree->begin_leafs(); it != oc_tree->end_leafs(); ++it) {
      if (!oc_tree->isNodeOccupied(*it)) continue;
      *iter_x = static_cast<float>(it.getX());
      *iter_y = static_cast<float>(it.getY());
      *iter_z = static_cast<float>(it.getZ());
      ++iter_x; ++iter_y; ++iter_z;
    }

    cloud_pub_.publish(cloud_msg);
    ROS_INFO_THROTTLE(3.0, "Published PointCloud2 from OctoMap: %zu points", occupied_count);
  }

  ros::Subscriber octomap_sub_;
  ros::Publisher cloud_pub_;
  std::string frame_id_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "octomap_to_cloud");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  OctomapToCloudNode node(nh, pnh);
  ros::spin();
  return 0;
}