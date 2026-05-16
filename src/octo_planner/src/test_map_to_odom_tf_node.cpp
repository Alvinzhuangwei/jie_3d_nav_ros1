// test_map_to_odom_tf_node.cpp  —  ROS 1 port of the original ROS 2 implementation
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

class TestMapToOdomTfNode
{
public:
  TestMapToOdomTfNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  : start_time_(ros::Time::now())
  {
    pnh.param<double>     ("radius",       radius_,       2.0);
    pnh.param<double>     ("orbit_period", orbit_period_, 20.0);
    pnh.param<double>     ("spin_rate",    spin_rate_,    0.8);
    pnh.param<std::string>("parent_frame", parent_frame_, "map");
    pnh.param<std::string>("child_frame",  child_frame_,  "odom");

    orbit_period_ = std::max(0.1, orbit_period_);

    timer_ = nh.createTimer(
      ros::Duration(0.033),
      &TestMapToOdomTfNode::onTimer, this);

    ROS_INFO(
      "test_map_to_odom_tf_node started. parent=%s child=%s "
      "radius=%.2f orbit_period=%.2f spin_rate=%.2f",
      parent_frame_.c_str(), child_frame_.c_str(),
      radius_, orbit_period_, spin_rate_);
  }

private:
  void onTimer(const ros::TimerEvent &)
  {
    const double t = (ros::Time::now() - start_time_).toSec();
    const double orbit_angle = 2.0 * M_PI * t / orbit_period_;
    const double yaw = orbit_angle + spin_rate_ * t;

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp       = ros::Time::now();
    tf_msg.header.frame_id    = parent_frame_;
    tf_msg.child_frame_id     = child_frame_;
    tf_msg.transform.translation.x = radius_ * std::cos(orbit_angle);
    tf_msg.transform.translation.y = radius_ * std::sin(orbit_angle);
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    tf_msg.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_.sendTransform(tf_msg);
  }

  ros::Time                    start_time_;
  ros::Timer                   timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string parent_frame_, child_frame_;
  double radius_, orbit_period_, spin_rate_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_map_to_odom_tf_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  TestMapToOdomTfNode node(nh, pnh);
  ros::spin();
  return 0;
}