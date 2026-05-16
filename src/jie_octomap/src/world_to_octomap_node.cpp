#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tinyxml2.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

namespace {
struct Transform3 {
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t{Eigen::Vector3d::Zero()};
};
Transform3 compose(const Transform3 & a, const Transform3 & b) {
  Transform3 out; out.R = a.R * b.R; out.t = a.R * b.t + a.t; return out;
}
Eigen::Vector3d apply(const Transform3 & tf, const Eigen::Vector3d & p) {
  return tf.R * p + tf.t;
}
std::vector<double> parseDoubles(const std::string & s) {
  std::istringstream iss(s); std::vector<double> v; double x;
  while (iss >> x) v.push_back(x); return v;
}
Transform3 parsePose(const tinyxml2::XMLElement * e) {
  Transform3 tf; if (!e || !e->GetText()) return tf;
  auto v = parseDoubles(e->GetText()); if (v.size() < 6) return tf;
  const Eigen::AngleAxisd rx(v[3],Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd ry(v[4],Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rz(v[5],Eigen::Vector3d::UnitZ());
  tf.R = (rz*ry*rx).toRotationMatrix();
  tf.t = Eigen::Vector3d(v[0],v[1],v[2]); return tf;
}
} // namespace

class WorldToOctomapNode {
public:
  WorldToOctomapNode(ros::NodeHandle & nh, ros::NodeHandle & pnh) {
    std::string world_file, octomap_topic, marker_topic, world_file_cmd_topic;
    pnh.param<std::string>("world_file", world_file, "");
    pnh.param<double>("resolution", resolution_, 0.2);
    pnh.param<double>("xy_window_size_m", half_xy_, 11.0); half_xy_ *= 0.5;
    pnh.param<double>("ground_surface_max_thickness_m", ground_thick_, 0.6);
    pnh.param<bool>("enable_stair_step_surface_mode", enable_stair_, true);
    pnh.param<double>("stair_step_max_height_m", stair_h_, 0.5);
    pnh.param<double>("stair_step_max_depth_m", stair_d_, 0.8);
    pnh.param<double>("stair_step_min_width_m", stair_w_, 1.0);
    pnh.param<std::string>("frame_id", frame_id_, "map");
    pnh.param<std::string>("octomap_topic", octomap_topic, "/octomap");
    pnh.param<std::string>("marker_topic", marker_topic, "/octomap_occupied_markers");
    pnh.param<std::string>("world_file_cmd_topic", world_file_cmd_topic, "/world_file_cmd");

    octomap_pub_ = nh.advertise<octomap_msgs::Octomap>(octomap_topic, 1, true);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>(marker_topic, 1, true);
    world_file_sub_ = nh.subscribe(world_file_cmd_topic, 1,
                                   &WorldToOctomapNode::onWorldFileCmd, this);
    if (!world_file.empty()) loadWorld(world_file);
    else ROS_WARN("No initial world_file set. Waiting for /world_file_cmd.");
    timer_ = nh.createTimer(ros::Duration(1.0), &WorldToOctomapNode::onTimer, this);
  }

private:
  void onTimer(const ros::TimerEvent &) { publishAll(); }
  void onWorldFileCmd(const std_msgs::String::ConstPtr & msg) {
    if (!msg->data.empty()) loadWorld(msg->data);
  }
  void loadWorld(const std::string & wf) {
    try { generateFromWorld(wf, resolution_); loaded_world_ = wf; publishAll();
          ROS_INFO("Loaded world file: %s", wf.c_str());
    } catch (const std::exception & e) {
      ROS_ERROR("Load world failed: %s", e.what());
    }
  }
  void markPoint(const Eigen::Vector3d & p) {
    if (std::abs(p.x()) > half_xy_ || std::abs(p.y()) > half_xy_) return;
    octomap::OcTreeKey key;
    const octomap::point3d q(p.x(), p.y(), p.z());
    if (!tree_->coordToKeyChecked(q, key)) return;
    tree_->updateNode(tree_->keyToCoord(key), true);
  }
  void fillBox(const Transform3 & tf, const tinyxml2::XMLElement * box) {
    const auto * se = box->FirstChildElement("size");
    if (!se||!se->GetText()) return;
    auto v = parseDoubles(se->GetText()); if (v.size()<3) return;
    const double sx=v[0], sy=v[1], sz=v[2], r=tree_->getResolution();
    const Eigen::Vector3d lz = tf.R * Eigen::Vector3d::UnitZ();
    const bool near_horiz = std::abs(lz.z()) > 0.9;
    const bool thin_ground = near_horiz && sz <= ground_thick_;
    const double min_xy = std::min(sx, sy), max_xy = std::max(sx, sy);
    const bool stair_like = enable_stair_ && near_horiz && sz<=stair_h_ &&
                            min_xy<=stair_d_ && max_xy>=stair_w_;
    if (thin_ground || stair_like) {
      const double step = std::max(r*0.5, 1e-3);
      const int nx=std::max(1,(int)std::ceil(sx/step)), ny=std::max(1,(int)std::ceil(sy/step));
      for (int ix=0;ix<=nx;++ix) for (int iy=0;iy<=ny;++iy) {
        markPoint(apply(tf, Eigen::Vector3d(
          -sx*0.5+(sx*(double)ix/(double)nx),
          -sy*0.5+(sy*(double)iy/(double)ny), sz*0.5)));
      } return;
    }
    const double min_dim = std::min({sx,sy,sz});
    const double step = (min_dim <= 4.0*r) ? std::max(r*0.5, 1e-3) : r;
    const int nx=std::max(1,(int)std::ceil(sx/step)),
              ny=std::max(1,(int)std::ceil(sy/step)),
              nz=std::max(1,(int)std::ceil(sz/step));
    for (int ix=0;ix<=nx;++ix) for (int iy=0;iy<=ny;++iy) for (int iz=0;iz<=nz;++iz)
      markPoint(apply(tf, Eigen::Vector3d(
        -sx*0.5+(sx*(double)ix/(double)nx),
        -sy*0.5+(sy*(double)iy/(double)ny),
        -sz*0.5+(sz*(double)iz/(double)nz))));
  }
  void fillCylinder(const Transform3 & tf, const tinyxml2::XMLElement * cyl) {
    auto *re=cyl->FirstChildElement("radius"), *le=cyl->FirstChildElement("length");
    if (!re||!le||!re->GetText()||!le->GetText()) return;
    const double radius=std::stod(re->GetText()), length=std::stod(le->GetText()), res=tree_->getResolution();
    for (double x=-radius;x<=radius;x+=res) for (double y=-radius;y<=radius;y+=res) {
      if (x*x+y*y>radius*radius) continue;
      for (double z=-length*0.5;z<=length*0.5;z+=res) markPoint(apply(tf, Eigen::Vector3d(x,y,z)));
    }
  }
  void fillSphere(const Transform3 & tf, const tinyxml2::XMLElement * sph) {
    auto *re=sph->FirstChildElement("radius");
    if (!re||!re->GetText()) return;
    const double radius=std::stod(re->GetText()), res=tree_->getResolution();
    for (double x=-radius;x<=radius;x+=res) for (double y=-radius;y<=radius;y+=res)
      for (double z=-radius;z<=radius;z+=res) {
        if (x*x+y*y+z*z>radius*radius) continue;
        markPoint(apply(tf, Eigen::Vector3d(x,y,z)));
      }
  }
  void fillPlane(const Transform3 & tf, const tinyxml2::XMLElement * plane) {
    auto *se=plane->FirstChildElement("size"); if (!se||!se->GetText()) return;
    auto v=parseDoubles(se->GetText()); if (v.size()<2) return;
    const double sx=v[0], sy=v[1], r=tree_->getResolution();
    const double step=std::max(r*0.5, 1e-3);
    const int nx=std::max(1,(int)std::ceil(sx/step)), ny=std::max(1,(int)std::ceil(sy/step));
    for (int ix=0;ix<=nx;++ix) for (int iy=0;iy<=ny;++iy)
      markPoint(apply(tf, Eigen::Vector3d(
        -sx*0.5+(sx*(double)ix/(double)nx),
        -sy*0.5+(sy*(double)iy/(double)ny), 0.0)));
  }
  void parseModel(const tinyxml2::XMLElement * model, const Transform3 & model_tf, int & cnt) {
    for (auto *link=model->FirstChildElement("link"); link; link=link->NextSiblingElement("link")) {
      const Transform3 link_tf = compose(model_tf, parsePose(link->FirstChildElement("pose")));
      for (auto *col=link->FirstChildElement("collision"); col; col=col->NextSiblingElement("collision")) {
        const Transform3 col_tf = compose(link_tf, parsePose(col->FirstChildElement("pose")));
        auto *geom=col->FirstChildElement("geometry"); if (!geom) continue;
        if (auto *b=geom->FirstChildElement("box")) { fillBox(col_tf,b); ++cnt; }
        else if (auto *c=geom->FirstChildElement("cylinder")) { fillCylinder(col_tf,c); ++cnt; }
        else if (auto *s=geom->FirstChildElement("sphere")) { fillSphere(col_tf,s); ++cnt; }
        else if (auto *p=geom->FirstChildElement("plane")) { fillPlane(col_tf,p); ++cnt; }
      }
    }
  }
  void generateFromWorld(const std::string & wf, double res) {
    tree_ = std::make_shared<octomap::OcTree>(res);
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(wf.c_str()) != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("failed to load world file: " + wf);
    auto *sdf=doc.FirstChildElement("sdf"); if (!sdf) throw std::runtime_error("no <sdf>");
    auto *world=sdf->FirstChildElement("world"); if (!world) throw std::runtime_error("no <world>");
    Transform3 world_tf; int cnt=0;
    for (auto *m=world->FirstChildElement("model"); m; m=m->NextSiblingElement("model"))
      parseModel(m, compose(world_tf, parsePose(m->FirstChildElement("pose"))), cnt);
    tree_->updateInnerOccupancy();
    ROS_INFO("World done. shapes=%d occupied=%zu", cnt, tree_->size());
  }
  void publishAll() {
    if (!tree_) return;
    const ros::Time stamp = ros::Time::now();
    octomap_msgs::Octomap map_msg;
    if (!octomap_msgs::binaryMapToMsg(*tree_, map_msg)) { ROS_ERROR("Failed serialize."); return; }
    map_msg.header.stamp = stamp; map_msg.header.frame_id = frame_id_;
    octomap_pub_.publish(map_msg);
    visualization_msgs::Marker marker;
    marker.header.stamp = stamp; marker.header.frame_id = frame_id_;
    marker.ns = "occupied_voxels"; marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = tree_->getResolution();
    marker.color.r=0.95f; marker.color.g=0.45f; marker.color.b=0.15f; marker.color.a=0.95f;
    for (auto it=tree_->begin_leafs(); it!=tree_->end_leafs(); ++it) {
      if (!tree_->isNodeOccupied(*it)) continue;
      geometry_msgs::Point p; p.x=it.getX(); p.y=it.getY(); p.z=it.getZ();
      marker.points.push_back(p);
    }
    marker_pub_.publish(marker);
  }

  std::shared_ptr<octomap::OcTree> tree_;
  std::string loaded_world_, frame_id_;
  double resolution_, half_xy_, ground_thick_, stair_h_, stair_d_, stair_w_;
  bool enable_stair_;
  ros::Timer timer_;
  ros::Publisher octomap_pub_, marker_pub_;
  ros::Subscriber world_file_sub_;
};

int main(int argc, char ** argv) {
  ros::init(argc, argv, "world_to_octomap");
  ros::NodeHandle nh; ros::NodeHandle pnh("~");
  WorldToOctomapNode node(nh, pnh);
  ros::spin(); return 0;
}