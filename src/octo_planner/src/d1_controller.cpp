// d1_controller.cpp  —  ROS 1 port of the original ROS 2 implementation
#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

class D1ControllerNode
{
public:
  D1ControllerNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  : nh_(nh), pnh_(pnh),
    tf_listener_(tf_buffer_),
    target_index_(0),
    pose_adjusting_(false),
    goal_reached_(true),
    debug_view_disabled_(false)
  {
    // ── parameters ──────────────────────────────────────────────────────
    pnh_.param<std::string>("path_topic",                   path_topic_,                   "/planned_path");
    pnh_.param<std::string>("start_navigation_topic",       start_navigation_topic_,       "/start_navigation");
    pnh_.param<std::string>("stop_navigation_topic",        stop_navigation_topic_,        "/stop_navigation");
    pnh_.param<std::string>("cmd_vel_topic",                cmd_vel_topic_,                "/cmd_vel");
    pnh_.param<std::string>("manual_cmd_vel_topic",         manual_cmd_vel_topic_,         "/web_cmd_vel");
    pnh_.param<std::string>("tracking_point_marker_topic",  tracking_marker_topic_,        "/tracking_point_marker");
    pnh_.param<std::string>("map_frame",                    map_frame_,                    "map");
    pnh_.param<std::string>("base_frame",                   base_frame_,                   "base_footprint");
    pnh_.param<std::string>("base_frame_candidates",        base_frame_candidates_str_,    "odin1_base_link,base_link,base_footprint");
    pnh_.param<std::string>("robot_center_offset_frame",    robot_center_offset_frame_,    "odin1_base_link");
    pnh_.param<double>     ("robot_center_offset_x",        robot_center_offset_x_,        -0.18);
    pnh_.param<double>     ("robot_center_offset_y",        robot_center_offset_y_,         0.0);
    pnh_.param<double>     ("robot_center_offset_z",        robot_center_offset_z_,         0.0);
    pnh_.param<bool>       ("require_start_command",        require_start_command_,         true);
    pnh_.param<double>     ("control_frequency",            control_frequency_,             20.0);
    pnh_.param<double>     ("lookahead_distance",           lookahead_distance_,            0.20);
    pnh_.param<double>     ("tracking_point_reached_xy_tolerance", tracking_xy_tol_,        0.20);
    pnh_.param<double>     ("tracking_point_marker_scale",  tracking_marker_scale_,         0.28);
    pnh_.param<bool>       ("enable_tracking_debug_view",   enable_debug_view_,             true);
    pnh_.param<int>        ("tracking_debug_view_size_px",  debug_view_size_px_,            640);
    pnh_.param<double>     ("tracking_debug_view_pixels_per_meter", debug_ppm_,             80.0);
    pnh_.param<double>     ("tracking_debug_view_frequency",debug_view_frequency_,          10.0);
    pnh_.param<double>     ("goal_position_tolerance",      goal_pos_tol_,                  0.05);
    pnh_.param<double>     ("goal_yaw_tolerance",           goal_yaw_tol_,                  0.10);
    pnh_.param<double>     ("linear_gain",                  linear_gain_,                   1.5);
    pnh_.param<double>     ("lateral_gain",                 lateral_gain_,                  1.5);
    pnh_.param<double>     ("heading_gain",                 heading_gain_,                  2.5);
    pnh_.param<double>     ("cross_track_angular_gain",     cross_track_angular_gain_,      1.0);
    pnh_.param<double>     ("final_yaw_gain",               final_yaw_gain_,                0.5);
    pnh_.param<bool>       ("enable_lateral_motion",        enable_lateral_motion_,         true);
    pnh_.param<double>     ("max_linear_speed",             max_linear_speed_,              0.60);
    pnh_.param<double>     ("max_lateral_speed",            max_lateral_speed_,             0.60);
    pnh_.param<double>     ("max_angular_speed",            max_angular_speed_,             1.50);
    pnh_.param<bool>       ("align_final_yaw",              align_final_yaw_,               true);
    pnh_.param<double>     ("linear_deadband",              linear_deadband_,               0.05);
    pnh_.param<double>     ("lateral_deadband",             lateral_deadband_,              0.05);
    pnh_.param<double>     ("angular_deadband",             angular_deadband_,              0.05);

    // ── subscribers / publishers ─────────────────────────────────────────
    path_sub_     = nh_.subscribe(path_topic_,             1,  &D1ControllerNode::onPath,            this);
    start_nav_sub_= nh_.subscribe(start_navigation_topic_, 10, &D1ControllerNode::onStartNavigation, this);
    stop_nav_sub_ = nh_.subscribe(stop_navigation_topic_,  10, &D1ControllerNode::onStopNavigation,  this);
    manual_sub_   = nh_.subscribe(manual_cmd_vel_topic_,   10, &D1ControllerNode::onManualCmdVel,    this);

    cmd_pub_     = nh_.advertise<geometry_msgs::Twist>       (cmd_vel_topic_,         10);
    marker_pub_  = nh_.advertise<visualization_msgs::Marker> (tracking_marker_topic_,  1, /*latch=*/true);

    // ── timers ───────────────────────────────────────────────────────────
    const double ctrl_period  = 1.0 / std::max(1.0, control_frequency_);
    const double debug_period = 1.0 / std::max(1.0, debug_view_frequency_);
    control_timer_    = nh_.createTimer(ros::Duration(ctrl_period),  &D1ControllerNode::onControlTimer,    this);
    debug_view_timer_ = nh_.createTimer(ros::Duration(debug_period), &D1ControllerNode::renderTrackingDebugView, this);

    ROS_INFO(
      "d1_controller started. path=%s start_navigation=%s stop_navigation=%s "
      "cmd_vel=%s manual_cmd_vel=%s tracking_marker=%s map_frame=%s base_frame=%s "
      "require_start_command=%s",
      path_topic_.c_str(), start_navigation_topic_.c_str(), stop_navigation_topic_.c_str(),
      cmd_vel_topic_.c_str(), manual_cmd_vel_topic_.c_str(), tracking_marker_topic_.c_str(),
      map_frame_.c_str(), base_frame_.c_str(),
      require_start_command_ ? "true" : "false");
  }

private:
  // ── subscriber callbacks ─────────────────────────────────────────────
  void onPath(const nav_msgs::Path::ConstPtr & msg)
  {
    if (msg->poses.empty()) {
      pending_plan_.clear(); clearActivePlan();
      ROS_WARN("Received empty planned_path.");
      return;
    }
    if (require_start_command_) {
      pending_plan_ = msg->poses; clearActivePlan();
      publishCmd(geometry_msgs::Twist());
      ROS_INFO("Received planned_path with %zu poses. Waiting for start_navigation confirmation.",
        pending_plan_.size());
      return;
    }
    activatePlan(msg->poses);
  }

  void onStartNavigation(const std_msgs::Bool::ConstPtr & msg)
  {
    if (!msg->data) { stopNavigation("Navigation start denied/cancelled. Holding position."); return; }
    if (pending_plan_.empty()) {
      ROS_WARN("Start navigation requested, but no pending planned_path available."); return;
    }
    try {
      activatePlan(pending_plan_); pending_plan_.clear();
    } catch (const std::exception & ex) {
      stopNavigation("Start navigation failed. Holding position.");
      ROS_ERROR("Start navigation exception: %s", ex.what());
    }
  }

  void onStopNavigation(const std_msgs::Bool::ConstPtr & msg)
  {
    if (!msg->data) return;
    stopNavigation("Stop navigation requested. Path tracking aborted and zero velocity sent.");
  }

  void onManualCmdVel(const geometry_msgs::Twist::ConstPtr & msg)
  {
    if (isNavigationActive() && isNonZeroTwist(*msg)) {
      pending_plan_.clear(); clearActivePlan();
      ROS_INFO("Manual web velocity received while navigating. Path tracking aborted.");
    } else if (isNavigationActive()) {
      return;
    }
    publishCmd(*msg);
  }

  // ── plan management ───────────────────────────────────────────────────
  void activatePlan(const std::vector<geometry_msgs::PoseStamped> & plan)
  {
    global_plan_   = plan;
    target_index_  = findInitialTargetIndex3D();
    pose_adjusting_= false;
    goal_reached_  = global_plan_.empty();
    publishTrackingPointMarker();
    ROS_INFO("Navigation execution started with %zu poses. initial_target_index=%d",
      global_plan_.size(), target_index_);
  }

  void clearActivePlan()
  {
    global_plan_.clear(); target_index_ = 0;
    pose_adjusting_ = false; goal_reached_ = true;
    clearTrackingPointMarker();
  }

  void stopNavigation(const char * log_message)
  {
    pending_plan_.clear(); clearActivePlan();
    publishZeroBurst();
    ROS_INFO("%s", log_message);
  }

  bool isNavigationActive() const { return !global_plan_.empty() && !goal_reached_; }

  static bool isNonZeroTwist(const geometry_msgs::Twist & t)
  {
    constexpr double eps = 1.0e-6;
    return std::abs(t.linear.x)  > eps || std::abs(t.linear.y)  > eps ||
           std::abs(t.linear.z)  > eps || std::abs(t.angular.x) > eps ||
           std::abs(t.angular.y) > eps || std::abs(t.angular.z) > eps;
  }

  void publishZeroBurst()
  {
    const geometry_msgs::Twist zero;
    for (int i = 0; i < 5; ++i) publishCmd(zero);
  }

  // ── control timer ─────────────────────────────────────────────────────
  void onControlTimer(const ros::TimerEvent &)
  {
    try { onControlTimerImpl(); }
    catch (const std::exception & ex) {
      stopNavigation("Control loop exception. Path tracking aborted.");
      ROS_ERROR("Control loop exception: %s", ex.what());
    }
  }

  void onControlTimerImpl()
  {
    if (global_plan_.empty()) return;

    if (pose_adjusting_) {
      geometry_msgs::PoseStamped final_pose_base;
      if (!transformToBase(global_plan_.back(), final_pose_base)) return;
      trackFinalPose(final_pose_base, global_plan_.back());
      return;
    }

    TrackingTarget target;
    if (!selectTrackingTarget(target)) return;

    if (isFinalTrackingPointReached(target)) {
      pose_adjusting_ = true;
      ROS_INFO("Final tracking point reached. Switching to final yaw adjustment.");
      geometry_msgs::PoseStamped final_pose_base;
      if (!transformToBase(global_plan_.back(), final_pose_base)) return;
      trackFinalPose(final_pose_base, global_plan_.back());
      return;
    }

    geometry_msgs::Twist cmd_vel;
    const double heading_error = std::atan2(target.base_y, std::max(1.0e-6, target.base_x));
    cmd_vel.linear.x  = clamp(target.base_x * linear_gain_,  -max_linear_speed_,  max_linear_speed_);
    cmd_vel.linear.y  = enable_lateral_motion_
                        ? clamp(target.base_y * lateral_gain_, -max_lateral_speed_, max_lateral_speed_)
                        : 0.0;
    cmd_vel.angular.z = clamp(heading_error * heading_gain_ + target.base_y * cross_track_angular_gain_,
                              -max_angular_speed_, max_angular_speed_);
    cmd_vel.linear.x  = applyDeadband(cmd_vel.linear.x,  linear_deadband_);
    cmd_vel.linear.y  = applyDeadband(cmd_vel.linear.y,  lateral_deadband_);
    cmd_vel.angular.z = applyDeadband(cmd_vel.angular.z, angular_deadband_);

    ROS_INFO_THROTTLE(1.0,
      "Track target in %s: x=%.3f y=%.3f heading_err=%.3f cmd=(%.3f, %.3f, %.3f)",
      base_frame_.c_str(), target.base_x, target.base_y, heading_error,
      cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
    publishCmd(cmd_vel);
  }

  // ── tracking helpers ──────────────────────────────────────────────────
  struct RobotPose2D { double x, y, z, yaw; };
  struct TrackingTarget { double base_x, base_y; };

  bool isFinalTrackingPointReached(const TrackingTarget & target) const
  {
    if (global_plan_.empty() || target_index_ != static_cast<int>(global_plan_.size()) - 1)
      return false;
    return std::hypot(target.base_x, target.base_y) < tracking_xy_tol_;
  }

  int findInitialTargetIndex3D()
  {
    if (global_plan_.empty()) return 0;
    RobotPose2D robot_pose;
    if (!lookupRobotPose2D(robot_pose)) {
      ROS_WARN("Failed to get robot pose. Start tracking from path index 0."); return 0;
    }
    int nearest = 0; double nearest_sq = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < global_plan_.size(); ++i) {
      const auto & p = global_plan_[i].pose.position;
      const double dx = p.x - robot_pose.x, dy = p.y - robot_pose.y, dz = p.z - robot_pose.z;
      const double sq = dx*dx + dy*dy + dz*dz;
      if (sq < nearest_sq) { nearest_sq = sq; nearest = static_cast<int>(i); }
    }
    return nearest;
  }

  bool selectTrackingTarget(TrackingTarget & target)
  {
    if (global_plan_.empty()) return false;
    RobotPose2D robot_pose;
    if (!lookupRobotPose2D(robot_pose)) return false;

    const double reached_tol = tracking_xy_tol_;
    if (xyDistanceToPlanPoint(robot_pose, target_index_) < reached_tol
        && target_index_ < static_cast<int>(global_plan_.size()) - 1)
    {
      int next = target_index_;
      for (int i = target_index_ + 1; i < static_cast<int>(global_plan_.size()); ++i) {
        if (xyDistanceToPlanPoint(robot_pose, i) > reached_tol) { next = i; break; }
        if (i == static_cast<int>(global_plan_.size()) - 1) next = i;
      }
      if (next != target_index_) { target_index_ = next; publishTrackingPointMarker(); }
    }

    const auto & tp = global_plan_[static_cast<std::size_t>(target_index_)].pose.position;
    const double dx_map = tp.x - robot_pose.x, dy_map = tp.y - robot_pose.y;
    const double cy = std::cos(robot_pose.yaw), sy = std::sin(robot_pose.yaw);
    target.base_x =  cy * dx_map + sy * dy_map;
    target.base_y = -sy * dx_map + cy * dy_map;
    return true;
  }

  bool lookupRobotPose2D(RobotPose2D & robot_pose)
  {
    std::string last_error;
    for (const auto & base_frame : getBaseFrameCandidates()) {
      try {
        const auto tf = tf_buffer_.lookupTransform(map_frame_, base_frame, ros::Time(0), ros::Duration(0.05));
        if (active_base_frame_ != base_frame) {
          active_base_frame_ = base_frame;
          ROS_INFO("Using robot base frame for tracking: %s", base_frame.c_str());
        }
        robot_pose.x   = tf.transform.translation.x;
        robot_pose.y   = tf.transform.translation.y;
        robot_pose.z   = tf.transform.translation.z;
        robot_pose.yaw = tf2::getYaw(tf.transform.rotation);
        applyRobotCenterOffset(base_frame, robot_pose);
        return true;
      } catch (const tf2::TransformException & ex) {
        last_error = ex.what();
      }
    }
    ROS_WARN_THROTTLE(2.0, "Lookup robot pose from %s failed for all base_frame candidates. Last: %s",
      map_frame_.c_str(), last_error.c_str());
    return false;
  }

  double xyDistanceToPlanPoint(const RobotPose2D & rp, int idx) const
  {
    const auto & p = global_plan_[static_cast<std::size_t>(idx)].pose.position;
    return std::hypot(p.x - rp.x, p.y - rp.y);
  }

  // ── final pose tracking ───────────────────────────────────────────────
  void trackFinalPose(
    const geometry_msgs::PoseStamped & final_pose_base,
    const geometry_msgs::PoseStamped & final_pose_map)
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = clamp(final_pose_base.pose.position.x * linear_gain_,  -max_linear_speed_,  max_linear_speed_);
    cmd_vel.linear.y = enable_lateral_motion_
                       ? clamp(final_pose_base.pose.position.y * lateral_gain_, -max_lateral_speed_, max_lateral_speed_)
                       : 0.0;
    cmd_vel.linear.x = applyDeadband(cmd_vel.linear.x, linear_deadband_);
    cmd_vel.linear.y = applyDeadband(cmd_vel.linear.y, lateral_deadband_);

    double final_yaw_error = 0.0;
    if (align_final_yaw_) {
      if (!computeFinalYawErrorXY(final_pose_map, final_yaw_error)) return;
      cmd_vel.angular.z = clamp(final_yaw_error * final_yaw_gain_, -max_angular_speed_, max_angular_speed_);
      cmd_vel.angular.z = applyDeadband(cmd_vel.angular.z, angular_deadband_);
    }

    const bool pos_ok = std::hypot(final_pose_base.pose.position.x, final_pose_base.pose.position.y) < goal_pos_tol_;
    const bool yaw_ok = !align_final_yaw_ || std::abs(final_yaw_error) < goal_yaw_tol_;
    if (pos_ok && yaw_ok) { finishNavigationAtGoal(); return; }
    publishCmd(cmd_vel);
  }

  void finishNavigationAtGoal()
  {
    pending_plan_.clear(); clearActivePlan(); publishZeroBurst();
    ROS_INFO("Goal reached within position and yaw tolerances. Navigation finished.");
  }

  bool computeFinalYawErrorXY(const geometry_msgs::PoseStamped & final_pose_in, double & yaw_error)
  {
    RobotPose2D robot_pose;
    if (!lookupRobotPose2D(robot_pose)) return false;
    geometry_msgs::PoseStamped final_pose = final_pose_in;
    if (final_pose.header.frame_id.empty()) final_pose.header.frame_id = map_frame_;
    final_pose.header.stamp = ros::Time(0);
    try {
      if (final_pose.header.frame_id != map_frame_)
        tf_buffer_.transform(final_pose, final_pose, map_frame_, ros::Duration(0.05));
    } catch (const tf2::TransformException & ex) {
      ROS_WARN_THROTTLE(2.0, "Transform final pose yaw %s -> %s failed: %s",
        final_pose.header.frame_id.c_str(), map_frame_.c_str(), ex.what());
      return false;
    }
    yaw_error = normalizeAngle(tf2::getYaw(final_pose.pose.orientation) - robot_pose.yaw);
    return true;
  }

  // ── transform helpers ─────────────────────────────────────────────────
  bool transformToBase(const geometry_msgs::PoseStamped & pose_in, geometry_msgs::PoseStamped & pose_out)
  {
    RobotPose2D unused;
    if (active_base_frame_.empty() && !lookupRobotPose2D(unused)) return false;
    const std::string base_frame = active_base_frame_.empty() ? base_frame_ : active_base_frame_;
    geometry_msgs::PoseStamped stamped = pose_in;
    if (stamped.header.frame_id.empty()) stamped.header.frame_id = map_frame_;
    stamped.header.stamp = ros::Time(0);
    try {
      tf_buffer_.transform(stamped, pose_out, base_frame, ros::Duration(0.05));
      applyRobotCenterOffsetToRelativePose(base_frame, pose_out);
      return true;
    } catch (const tf2::TransformException & ex) {
      ROS_WARN_THROTTLE(2.0, "Transform %s -> %s failed: %s",
        stamped.header.frame_id.c_str(), base_frame.c_str(), ex.what());
      return false;
    }
  }

  std::vector<std::string> getBaseFrameCandidates() const
  {
    std::vector<std::string> candidates;
    const auto add = [&](const std::string & f) {
      if (!f.empty() && std::find(candidates.begin(), candidates.end(), f) == candidates.end())
        candidates.push_back(f);
    };
    add(base_frame_);
    for (const auto & f : splitCsv(base_frame_candidates_str_)) add(f);
    return candidates;
  }

  bool shouldApplyRobotCenterOffset(const std::string & frame) const
  { return frame == robot_center_offset_frame_; }

  void applyRobotCenterOffset(const std::string & frame, RobotPose2D & rp) const
  {
    if (!shouldApplyRobotCenterOffset(frame)) return;
    const double cy = std::cos(rp.yaw), sy = std::sin(rp.yaw);
    rp.x += cy * robot_center_offset_x_ - sy * robot_center_offset_y_;
    rp.y += sy * robot_center_offset_x_ + cy * robot_center_offset_y_;
    rp.z += robot_center_offset_z_;
  }

  void applyRobotCenterOffsetToRelativePose(const std::string & frame, geometry_msgs::PoseStamped & pose) const
  {
    if (!shouldApplyRobotCenterOffset(frame)) return;
    pose.pose.position.x -= robot_center_offset_x_;
    pose.pose.position.y -= robot_center_offset_y_;
    pose.pose.position.z -= robot_center_offset_z_;
  }

  // ── marker ────────────────────────────────────────────────────────────
  void publishTrackingPointMarker()
  {
    if (global_plan_.empty()) { clearTrackingPointMarker(); return; }
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp    = ros::Time::now();
    marker.ns     = "d1_tracking_point";
    marker.id     = 0;
    marker.type   = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose   = global_plan_[static_cast<std::size_t>(target_index_)].pose;
    marker.scale.x = marker.scale.y = marker.scale.z = tracking_marker_scale_;
    marker.color.r = 0.1f; marker.color.g = 0.65f; marker.color.b = 1.0f; marker.color.a = 0.95f;
    marker_pub_.publish(marker);
  }

  void clearTrackingPointMarker()
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp    = ros::Time::now();
    marker.ns     = "d1_tracking_point";
    marker.id     = 0;
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub_.publish(marker);
  }

  // ── OpenCV debug view ─────────────────────────────────────────────────
  void renderTrackingDebugView(const ros::TimerEvent &)
  {
    try { renderTrackingDebugViewImpl(); }
    catch (const std::exception & ex) {
      ROS_WARN_THROTTLE(2.0, "OpenCV tracking debug view exception: %s", ex.what());
    }
  }

  void renderTrackingDebugViewImpl()
  {
    if (!enable_debug_view_ || debug_view_disabled_ || global_plan_.empty()) return;
    RobotPose2D robot_pose;
    if (!lookupRobotPose2D(robot_pose)) return;

    const int sz = std::max(240, debug_view_size_px_);
    const double ppm = std::max(10.0, debug_ppm_);
    const cv::Point center(sz / 2, sz / 2);
    cv::Mat image(sz, sz, CV_8UC3, cv::Scalar(18, 24, 28));

    cv::line(image, {center.x, 0}, {center.x, sz}, cv::Scalar(48, 64, 70), 1);
    cv::line(image, {0, center.y}, {sz, center.y}, cv::Scalar(48, 64, 70), 1);
    cv::arrowedLine(image, center, {center.x, center.y - 58}, cv::Scalar(230, 230, 230), 2, cv::LINE_AA, 0, 0.25);
    cv::circle(image, center, 8, cv::Scalar(230, 230, 230), -1, cv::LINE_AA);
    cv::putText(image, "robot +X", {center.x + 10, center.y - 62},
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);

    std::vector<cv::Point> proj;
    proj.reserve(global_plan_.size());
    for (const auto & pose : global_plan_)
      proj.push_back(projectPlanPoint(robot_pose, pose, center, ppm));

    for (std::size_t i = 1; i < proj.size(); ++i)
      cv::line(image, proj[i - 1], proj[i], cv::Scalar(120, 120, 120), 1, cv::LINE_AA);
    for (const auto & pt : proj)
      cv::circle(image, pt, 3, cv::Scalar(90, 210, 90), -1, cv::LINE_AA);

    if (target_index_ >= 0 && target_index_ < static_cast<int>(proj.size())) {
      cv::circle(image, proj[target_index_], 12, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
      cv::circle(image, proj[target_index_],  4, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
    }

    double final_yaw_error = 0.0;
    if (drawFinalGoalYaw(image, robot_pose, center, ppm, final_yaw_error)) {
      char buf[160];
      std::snprintf(buf, sizeof(buf), "goal yaw err: %.1f deg", final_yaw_error * 180.0 / M_PI);
      cv::putText(image, buf, {16, 108}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 230, 255), 1, cv::LINE_AA);
    }

    cv::putText(image, "tracking index: " + std::to_string(target_index_),
      {16, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(80, 190, 255), 2, cv::LINE_AA);
    char pose_text[160];
    std::snprintf(pose_text, sizeof(pose_text), "robot map: x=%.2f y=%.2f yaw=%.1f deg",
      robot_pose.x, robot_pose.y, robot_pose.yaw * 180.0 / M_PI);
    cv::putText(image, pose_text, {16, 56}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    char cmd_text[160];
    std::snprintf(cmd_text, sizeof(cmd_text), "cmd vel: x=%.3f y=%.3f wz=%.3f",
      last_cmd_vel_.linear.x, last_cmd_vel_.linear.y, last_cmd_vel_.angular.z);
    cv::putText(image, cmd_text, {16, 82}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(120, 230, 255), 1, cv::LINE_AA);
    cv::putText(image, "top = robot forward, red = current target",
      {16, sz - 18}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(180, 200, 210), 1, cv::LINE_AA);

    try {
      cv::imshow("d1_controller_xy_tracking", image);
      cv::waitKey(1);
    } catch (const cv::Exception & ex) {
      debug_view_disabled_ = true;
      ROS_WARN("Disable OpenCV tracking debug view: %s", ex.what());
    }
  }

  cv::Point projectPlanPoint(
    const RobotPose2D & rp,
    const geometry_msgs::PoseStamped & pose,
    const cv::Point & center, double ppm) const
  {
    const double dx = pose.pose.position.x - rp.x, dy = pose.pose.position.y - rp.y;
    const double cy = std::cos(rp.yaw), sy = std::sin(rp.yaw);
    const double base_x = cy * dx + sy * dy, base_y = -sy * dx + cy * dy;
    return cv::Point(
      static_cast<int>(std::round(center.x - base_y * ppm)),
      static_cast<int>(std::round(center.y - base_x * ppm)));
  }

  bool drawFinalGoalYaw(
    cv::Mat & image, const RobotPose2D & rp,
    const cv::Point & center, double ppm, double & yaw_error) const
  {
    if (global_plan_.empty()) return false;
    const auto & fp = global_plan_.back();
    const cv::Point fp_px = projectPlanPoint(rp, fp, center, ppm);
    const double goal_yaw = tf2::getYaw(fp.pose.orientation);
    yaw_error = normalizeAngle(goal_yaw - rp.yaw);
    const double arrow_len = std::max(26.0, ppm * 0.35);
    const cv::Point arrow_end(
      static_cast<int>(std::round(fp_px.x - std::sin(yaw_error) * arrow_len)),
      static_cast<int>(std::round(fp_px.y - std::cos(yaw_error) * arrow_len)));
    cv::circle(image, fp_px, 10, cv::Scalar(0, 230, 255), 2, cv::LINE_AA);
    cv::arrowedLine(image, fp_px, arrow_end, cv::Scalar(0, 230, 255), 2, cv::LINE_AA, 0, 0.30);
    return true;
  }

  // ── publish ───────────────────────────────────────────────────────────
  void publishCmd(const geometry_msgs::Twist & cmd_vel)
  {
    last_cmd_vel_ = cmd_vel;
    cmd_pub_.publish(cmd_vel);
  }

  // ── static helpers ────────────────────────────────────────────────────
  static std::vector<std::string> splitCsv(const std::string & text)
  {
    std::vector<std::string> parts; std::string cur;
    for (const char ch : text) {
      if (ch == ',') { const auto t = trim(cur); if (!t.empty()) parts.push_back(t); cur.clear(); }
      else cur.push_back(ch);
    }
    const auto t = trim(cur); if (!t.empty()) parts.push_back(t);
    return parts;
  }

  static std::string trim(const std::string & text)
  {
    std::size_t first = 0;
    while (first < text.size() && std::isspace(static_cast<unsigned char>(text[first]))) ++first;
    std::size_t last = text.size();
    while (last > first && std::isspace(static_cast<unsigned char>(text[last - 1]))) --last;
    return text.substr(first, last - first);
  }

  static double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
  static double applyDeadband(double v, double db) { return std::abs(v) < db ? 0.0 : v; }
  static double normalizeAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

  // ── members ───────────────────────────────────────────────────────────
  ros::NodeHandle & nh_;
  ros::NodeHandle & pnh_;
  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber path_sub_, start_nav_sub_, stop_nav_sub_, manual_sub_;
  ros::Publisher  cmd_pub_, marker_pub_;
  ros::Timer      control_timer_, debug_view_timer_;

  // parameters
  std::string path_topic_, start_navigation_topic_, stop_navigation_topic_;
  std::string cmd_vel_topic_, manual_cmd_vel_topic_, tracking_marker_topic_;
  std::string map_frame_, base_frame_, base_frame_candidates_str_;
  std::string robot_center_offset_frame_;
  double robot_center_offset_x_, robot_center_offset_y_, robot_center_offset_z_;
  bool   require_start_command_;
  double control_frequency_, lookahead_distance_, tracking_xy_tol_, tracking_marker_scale_;
  bool   enable_debug_view_;
  int    debug_view_size_px_;
  double debug_ppm_, debug_view_frequency_;
  double goal_pos_tol_, goal_yaw_tol_;
  double linear_gain_, lateral_gain_, heading_gain_, cross_track_angular_gain_, final_yaw_gain_;
  bool   enable_lateral_motion_;
  double max_linear_speed_, max_lateral_speed_, max_angular_speed_;
  bool   align_final_yaw_;
  double linear_deadband_, lateral_deadband_, angular_deadband_;

  // state
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  std::vector<geometry_msgs::PoseStamped> pending_plan_;
  int    target_index_;
  bool   pose_adjusting_;
  bool   goal_reached_;
  bool   debug_view_disabled_;
  std::string active_base_frame_;
  geometry_msgs::Twist last_cmd_vel_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "d1_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  D1ControllerNode node(nh, pnh);
  ros::spin();
  return 0;
}