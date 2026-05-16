#include <array>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <open3d/Open3D.h>

#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <std_msgs/String.h>

struct PcdKey
{
  unsigned int k[3];
  bool operator==(const PcdKey & o) const
  { return k[0]==o.k[0] && k[1]==o.k[1] && k[2]==o.k[2]; }
};
struct PcdKeyHash
{
  std::size_t operator()(const PcdKey & k) const {
    std::size_t seed = std::hash<unsigned int>{}(k.k[0]);
    seed ^= std::hash<unsigned int>{}(k.k[1]) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    seed ^= std::hash<unsigned int>{}(k.k[2]) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    return seed;
  }
};

class PcdToOctomapNode
{
public:
  PcdToOctomapNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  {
    std::string pcd_file, pcd_file_cmd_topic, octomap_topic;
    pnh.param<std::string>("pcd_file", pcd_file, "");
    pnh.param<std::string>("pcd_file_cmd_topic", pcd_file_cmd_topic, "/pcd_file_cmd");
    pnh.param<std::string>("octomap_topic", octomap_topic, "/octomap");
    pnh.param<std::string>("frame_id", frame_id_, "map");
    pnh.param<double>("resolution", resolution_, 0.2);
    pnh.param<double>("voxel_downsample_m", voxel_downsample_, 0.0);
    pnh.param<int>("min_points_per_voxel", min_points_per_voxel_, 3);
    pnh.param<int>("min_cluster_voxels", min_cluster_voxels_, 4);

    octomap_pub_ = nh.advertise<octomap_msgs::Octomap>(octomap_topic, 1, true);
    pcd_file_sub_ = nh.subscribe(pcd_file_cmd_topic, 1,
                                  &PcdToOctomapNode::onPcdFileCmd, this);
    timer_ = nh.createTimer(ros::Duration(1.0), &PcdToOctomapNode::onTimer, this);

    if (!pcd_file.empty()) loadPcd(pcd_file);
    else ROS_INFO("No initial pcd_file set. Waiting for pcd_file_cmd.");
  }

private:
  void onTimer(const ros::TimerEvent &) { publishMap(); }

  void onPcdFileCmd(const std_msgs::String::ConstPtr & msg)
  {
    if (!msg->data.empty()) loadPcd(msg->data);
  }

  void loadPcd(const std::string & pcd_file)
  {
    open3d::geometry::PointCloud pc;
    if (!open3d::io::ReadPointCloud(pcd_file, pc)) {
      ROS_ERROR("Failed to read PCD file: %s", pcd_file.c_str());
      return;
    }
    if (voxel_downsample_ > 0.0) pc = *pc.VoxelDownSample(voxel_downsample_);

    tree_ = std::make_shared<octomap::OcTree>(resolution_);
    const int min_ppv = std::max(1, min_points_per_voxel_);
    const int min_cv = std::max(1, min_cluster_voxels_);

    std::unordered_map<PcdKey, std::size_t, PcdKeyHash> voxel_counts;
    voxel_counts.reserve(pc.points_.size());
    for (const auto & pt : pc.points_) {
      octomap::OcTreeKey raw;
      if (!tree_->coordToKeyChecked(
            static_cast<float>(pt.x()), static_cast<float>(pt.y()),
            static_cast<float>(pt.z()), raw)) continue;
      ++voxel_counts[PcdKey{{raw.k[0], raw.k[1], raw.k[2]}}];
    }

    std::unordered_set<PcdKey, PcdKeyHash> occupied;
    for (const auto & e : voxel_counts)
      if (static_cast<int>(e.second) >= min_ppv) occupied.insert(e.first);

    std::size_t removed = 0;
    if (min_cv > 1 && !occupied.empty()) {
      std::unordered_set<PcdKey, PcdKeyHash> filtered, visited;
      for (const auto & seed : occupied) {
        if (visited.count(seed)) continue;
        std::deque<PcdKey> queue;
        std::vector<PcdKey> cluster;
        queue.push_back(seed);
        visited.insert(seed);
        while (!queue.empty()) {
          const PcdKey cur = queue.front(); queue.pop_front();
          cluster.push_back(cur);
          for (int dx=-1;dx<=1;++dx) for (int dy=-1;dy<=1;++dy) for (int dz=-1;dz<=1;++dz) {
            if (!dx&&!dy&&!dz) continue;
            const auto nx=(int64_t)cur.k[0]+dx, ny=(int64_t)cur.k[1]+dy, nz=(int64_t)cur.k[2]+dz;
            if (nx<0||ny<0||nz<0) continue;
            PcdKey nb{{(unsigned)nx,(unsigned)ny,(unsigned)nz}};
            if (!occupied.count(nb)||visited.count(nb)) continue;
            visited.insert(nb); queue.push_back(nb);
          }
        }
        if ((int)cluster.size() >= min_cv) filtered.insert(cluster.begin(), cluster.end());
        else removed += cluster.size();
      }
      occupied = std::move(filtered);
    }

    for (const auto & key : occupied) {
      octomap::OcTreeKey ok; ok.k[0]=key.k[0]; ok.k[1]=key.k[1]; ok.k[2]=key.k[2];
      tree_->updateNode(tree_->keyToCoord(ok), true);
    }
    tree_->updateInnerOccupancy();
    publishMap();
    ROS_INFO("Loaded PCD: %s, kept_voxels=%zu, removed_small=%zu",
             pcd_file.c_str(), occupied.size(), removed);
  }

  void publishMap()
  {
    if (!tree_) return;
    octomap_msgs::Octomap msg;
    if (!octomap_msgs::binaryMapToMsg(*tree_, msg)) {
      ROS_ERROR("Failed to convert OcTree to octomap message."); return;
    }
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    octomap_pub_.publish(msg);
  }

  std::shared_ptr<octomap::OcTree> tree_;
  ros::Timer timer_;
  ros::Subscriber pcd_file_sub_;
  ros::Publisher octomap_pub_;
  std::string frame_id_;
  double resolution_, voxel_downsample_;
  int min_points_per_voxel_, min_cluster_voxels_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "pcd_to_octomap");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  PcdToOctomapNode node(nh, pnh);
  ros::spin();
  return 0;
}