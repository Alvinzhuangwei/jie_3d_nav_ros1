#include <string>

#include <ros/ros.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <visualization_msgs/Marker.h>

class OctomapToOccupiedMarkersNode
{
public:
  OctomapToOccupiedMarkersNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  {
    std::string octomap_topic, marker_topic;
    pnh.param<std::string>("octomap_topic", octomap_topic, "/octomap");
    pnh.param<std::string>("marker_topic", marker_topic, "/octomap_occupied_markers");
    pnh.param<std::string>("frame_id", frame_id_, "map");

    marker_pub_ = nh.advertise<visualization_msgs::Marker>(marker_topic, 1, /*latch=*/true);
    octomap_sub_ = nh.subscribe(octomap_topic, 1,
                                &OctomapToOccupiedMarkersNode::onOctomap, this);

    ROS_INFO("octomap_to_occupied_markers started. octomap=%s marker=%s",
             octomap_topic.c_str(), marker_topic.c_str());
  }

private:
  void onOctomap(const octomap_msgs::Octomap::ConstPtr & msg)
  {
    std::unique_ptr<octomap::AbstractOcTree> tree_ptr(octomap_msgs::msgToMap(*msg));
    if (!tree_ptr) {
      ROS_ERROR("Failed to decode octomap message.");
      return;
    }
    auto * oc_tree = dynamic_cast<octomap::OcTree *>(tree_ptr.get());
    if (!oc_tree) {
      ROS_ERROR("Decoded map is not octomap::OcTree.");
      return;
    }

    visualization_msgs::Marker marker;
    marker.header.stamp = msg->header.stamp;
    marker.header.frame_id = msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id;
    marker.ns = "occupied_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = oc_tree->getResolution();
    marker.scale.y = oc_tree->getResolution();
    marker.scale.z = oc_tree->getResolution();
    marker.color.r = 0.95f;
    marker.color.g = 0.45f;
    marker.color.b = 0.15f;
    marker.color.a = 0.95f;

    for (auto it = oc_tree->begin_leafs(); it != oc_tree->end_leafs(); ++it) {
      if (!oc_tree->isNodeOccupied(*it)) continue;
      geometry_msgs::Point p;
      p.x = it.getX();
      p.y = it.getY();
      p.z = it.getZ();
      marker.points.push_back(p);
    }

    marker_pub_.publish(marker);
    ROS_INFO_THROTTLE(3.0, "Published occupied marker from OctoMap: %zu voxels",
                      marker.points.size());
  }

  ros::Subscriber octomap_sub_;
  ros::Publisher marker_pub_;
  std::string frame_id_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "octomap_to_occupied_markers");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  OctomapToOccupiedMarkersNode node(nh, pnh);
  ros::spin();
  return 0;
}