#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

class RvizClickSelectorNode
{
public:
  RvizClickSelectorNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  : expect_start_(true), has_start_(false), has_goal_(false)
  {
    std::string clicked_topic, marker_topic, start_topic, goal_topic;
    pnh.param<std::string>("clicked_topic", clicked_topic, "/clicked_point");
    pnh.param<std::string>("marker_topic", marker_topic, "/selection_markers");
    pnh.param<std::string>("start_topic", start_topic, "/start_point");
    pnh.param<std::string>("goal_topic", goal_topic, "/goal_point");
    pnh.param<double>("arrow_height", arrow_height_, 0.6);
    pnh.param<double>("arrow_length", arrow_length_, 0.7);
    pnh.param<double>("shaft_diameter", shaft_diameter_, 0.16);
    pnh.param<double>("head_diameter", head_diameter_, 0.32);
    pnh.param<double>("head_length", head_length_, 0.44);
    pnh.param<double>("cube_size", cube_size_, 0.20);

    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(marker_topic, 1, true);
    start_pub_ = nh.advertise<geometry_msgs::PointStamped>(start_topic, 1, true);
    goal_pub_ = nh.advertise<geometry_msgs::PointStamped>(goal_topic, 1, true);
    clicked_sub_ = nh.subscribe(clicked_topic, 10,
                                &RvizClickSelectorNode::onClickedPoint, this);
    ROS_INFO("rviz_click_selector started. clicked=%s marker=%s",
             clicked_topic.c_str(), marker_topic.c_str());
  }

private:
  void onClickedPoint(const geometry_msgs::PointStamped::ConstPtr & msg)
  {
    if (expect_start_) {
      start_point_ = *msg;
      has_start_ = true;
      ROS_INFO("Set START point: [%.3f, %.3f, %.3f]",
               msg->point.x, msg->point.y, msg->point.z);
      start_pub_.publish(*msg);
    } else {
      goal_point_ = *msg;
      has_goal_ = true;
      ROS_INFO("Set GOAL point: [%.3f, %.3f, %.3f]",
               msg->point.x, msg->point.y, msg->point.z);
      goal_pub_.publish(*msg);
    }
    expect_start_ = !expect_start_;
    publishMarkers();
  }

  visualization_msgs::Marker makeArrow(int id,
                                       const geometry_msgs::PointStamped & p,
                                       float r, float g, float b) const
  {
    visualization_msgs::Marker m;
    m.header = p.header;
    m.ns = "rviz_selector";
    m.id = id;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = shaft_diameter_;
    m.scale.y = head_diameter_;
    m.scale.z = head_length_;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0f;
    m.pose.orientation.w = 1.0;
    geometry_msgs::Point base = p.point;
    base.z += arrow_height_;
    geometry_msgs::Point tip = base;
    tip.z -= arrow_length_;
    m.points.push_back(base);
    m.points.push_back(tip);
    return m;
  }

  visualization_msgs::Marker makeCube(int id,
                                      const geometry_msgs::PointStamped & p,
                                      float r, float g, float b) const
  {
    visualization_msgs::Marker m;
    m.header = p.header;
    m.ns = "rviz_selector";
    m.id = id;
    m.type = visualization_msgs::Marker::CUBE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position = p.point;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = cube_size_;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.95f;
    return m;
  }

  void publishMarkers()
  {
    visualization_msgs::MarkerArray arr;
    if (has_start_) {
      arr.markers.push_back(makeArrow(0, start_point_, 0.1f, 0.95f, 0.1f));
      arr.markers.push_back(makeCube(2, start_point_, 0.1f, 0.95f, 0.1f));
    }
    if (has_goal_) {
      arr.markers.push_back(makeArrow(1, goal_point_, 0.95f, 0.1f, 0.1f));
      arr.markers.push_back(makeCube(3, goal_point_, 0.95f, 0.1f, 0.1f));
    }
    marker_pub_.publish(arr);
  }

  bool expect_start_, has_start_, has_goal_;
  geometry_msgs::PointStamped start_point_, goal_point_;
  ros::Subscriber clicked_sub_;
  ros::Publisher marker_pub_, start_pub_, goal_pub_;
  double arrow_height_, arrow_length_, shaft_diameter_, head_diameter_, head_length_, cube_size_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "rviz_click_selector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  RvizClickSelectorNode node(nh, pnh);
  ros::spin();
  return 0;
}