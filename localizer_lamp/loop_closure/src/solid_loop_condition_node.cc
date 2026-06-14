/**
 * @file solid_loop_condition_node.cc
 * @brief Filters LAMP loop candidates with SOLiD descriptor similarity.
 *
 * This node does not compute loop closures. It only decides whether candidate
 * scan pairs are worth sending to the existing LAMP geometric verification
 * stage.
 */

#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <unordered_map>

#include <Eigen/Dense>
#include <gtsam/inference/Symbol.h>
#include <lamp_utils/PrefixHandling.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pose_graph_msgs/KeyedScan.h>
#include <pose_graph_msgs/LoopCandidateArray.h>
#include <ros/ros.h>
#include <solid_module.h>

namespace {

struct DescriptorEntry {
  Eigen::VectorXd descriptor;
  ros::Time stamp;
  std::size_t filtered_points = 0;
};

class SolidLoopConditionNode {
public:
  explicit SolidLoopConditionNode(const ros::NodeHandle& node_handle)
      : nh_(node_handle) {}

  bool Initialize() {
    LoadParameters();

    keyed_scan_sub_ = nh_.subscribe("keyed_scans",
                                    100000,
                                    &SolidLoopConditionNode::KeyedScanCallback,
                                    this);
    candidate_sub_ = nh_.subscribe(
        "input_loop_candidates",
        1000,
        &SolidLoopConditionNode::CandidateCallback,
        this);
    candidate_pub_ = nh_.advertise<pose_graph_msgs::LoopCandidateArray>(
        "output_loop_candidates", 10, false);

    retry_timer_ = nh_.createTimer(
        ros::Duration(retry_period_sec_),
        &SolidLoopConditionNode::RetryTimerCallback,
        this);

    ROS_INFO_STREAM("SOLiD loop condition initialized."
                    << " enable=" << enable_
                    << " threshold=" << similarity_threshold_
                    << " min_points=" << min_filtered_points_
                    << " max_wait=" << max_candidate_wait_sec_
                    << " pass_missing_after_timeout="
                    << pass_missing_after_timeout_);
    return true;
  }

private:
  void LoadParameters() {
    LoadParamGroup("solid_loop_condition");

    const std::string param_ns = ResolveLampParamNamespace();
    if (!param_ns.empty()) {
      LoadParamGroup(param_ns + "/solid_loop_condition");
    }

    // Flat private params are useful for direct ad-hoc rosrun testing.
    nh_.param("enable", enable_, enable_);
    nh_.param("similarity_threshold",
              similarity_threshold_,
              similarity_threshold_);
    nh_.param("min_filtered_points", min_filtered_points_, min_filtered_points_);
    nh_.param("max_candidate_wait_sec",
              max_candidate_wait_sec_,
              max_candidate_wait_sec_);
    nh_.param("retry_period_sec", retry_period_sec_, retry_period_sec_);
    nh_.param("pass_missing_after_timeout",
              pass_missing_after_timeout_,
              pass_missing_after_timeout_);
    nh_.param("publish_rejected_debug",
              publish_rejected_debug_,
              publish_rejected_debug_);

    similarity_threshold_ = std::max(0.0, std::min(1.0, similarity_threshold_));
    retry_period_sec_ = std::max(0.05, retry_period_sec_);
    max_candidate_wait_sec_ = std::max(0.0, max_candidate_wait_sec_);
  }

  void LoadParamGroup(const std::string& prefix) {
    nh_.param(prefix + "/enable", enable_, enable_);
    nh_.param(prefix + "/similarity_threshold",
              similarity_threshold_,
              similarity_threshold_);
    nh_.param(prefix + "/min_filtered_points",
              min_filtered_points_,
              min_filtered_points_);
    nh_.param(prefix + "/max_candidate_wait_sec",
              max_candidate_wait_sec_,
              max_candidate_wait_sec_);
    nh_.param(prefix + "/retry_period_sec", retry_period_sec_, retry_period_sec_);
    nh_.param(prefix + "/pass_missing_after_timeout",
              pass_missing_after_timeout_,
              pass_missing_after_timeout_);
    nh_.param(prefix + "/publish_rejected_debug",
              publish_rejected_debug_,
              publish_rejected_debug_);
  }

  std::string ResolveLampParamNamespace() const {
    const std::string ns = nh_.getNamespace();
    if (ns.find("base") != std::string::npos) {
      return "base";
    }

    for (const auto& robot_prefix : lamp_utils::ROBOT_PREFIXES) {
      if (robot_prefix.first == "base") {
        continue;
      }
      if (ns.find(robot_prefix.first) != std::string::npos) {
        return "robot";
      }
    }

    return "";
  }

  void KeyedScanCallback(const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg) {
    if (!enable_) {
      return;
    }

    pcl::PointCloud<PointType> raw_cloud;
    try {
      pcl::fromROSMsg(scan_msg->scan, raw_cloud);
    } catch (const std::exception& ex) {
      ROS_WARN_STREAM_THROTTLE(
          2.0,
          "SOLiD failed to convert KeyedScan "
              << gtsam::DefaultKeyFormatter(scan_msg->key) << ": "
              << ex.what());
      return;
    }

    pcl::PointCloud<PointType> finite_cloud;
    finite_cloud.points.reserve(raw_cloud.points.size());
    for (const auto& point : raw_cloud.points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) &&
          std::isfinite(point.z)) {
        finite_cloud.points.push_back(point);
      }
    }

    pcl::PointCloud<PointType>::Ptr near_cloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr clipped_cloud(
        new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr downsampled_cloud(
        new pcl::PointCloud<PointType>);

    solid_.remove_far_points(finite_cloud, near_cloud);
    solid_.remove_closest_points(*near_cloud, clipped_cloud);
    solid_.down_sampling(*clipped_cloud, downsampled_cloud);

    if (downsampled_cloud->points.size() <
        static_cast<std::size_t>(min_filtered_points_)) {
      ROS_WARN_STREAM_THROTTLE(
          5.0,
          "SOLiD skipped KeyedScan "
              << gtsam::DefaultKeyFormatter(scan_msg->key)
              << ": only " << downsampled_cloud->points.size()
              << " filtered points.");
      return;
    }

    Eigen::VectorXd descriptor = solid_.makeSolid(*downsampled_cloud);
    if (descriptor.size() == 0 || !descriptor.allFinite() ||
        descriptor.norm() <= 0.0) {
      ROS_WARN_STREAM_THROTTLE(
          5.0,
          "SOLiD skipped KeyedScan "
              << gtsam::DefaultKeyFormatter(scan_msg->key)
              << ": descriptor is empty or non-finite.");
      return;
    }

    DescriptorEntry entry;
    entry.descriptor = descriptor;
    entry.stamp = scan_msg->scan.header.stamp;
    entry.filtered_points = downsampled_cloud->points.size();

    std::lock_guard<std::mutex> lock(mutex_);
    descriptors_[scan_msg->key] = entry;
  }

  void CandidateCallback(
      const pose_graph_msgs::LoopCandidateArray::ConstPtr& candidates_msg) {
    if (!enable_) {
      candidate_pub_.publish(*candidates_msg);
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& candidate : candidates_msg->candidates) {
      PendingCandidate pending;
      pending.candidate = candidate;
      pending.originator = candidates_msg->originator;
      pending.enqueued_at = ros::Time::now();
      pending_candidates_.push_back(pending);
    }
    ProcessPendingCandidatesLocked();
  }

  void RetryTimerCallback(const ros::TimerEvent&) {
    if (!enable_) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    ProcessPendingCandidatesLocked();
  }

  bool HasDescriptor(uint64_t key) const {
    return descriptors_.find(key) != descriptors_.end();
  }

  bool CandidatePassesLocked(
      const pose_graph_msgs::LoopCandidate& candidate,
      double* score_out) const {
    const auto from_it = descriptors_.find(candidate.key_from);
    const auto to_it = descriptors_.find(candidate.key_to);
    if (from_it == descriptors_.end() || to_it == descriptors_.end()) {
      return false;
    }

    const double score = solid_.loop_detection(from_it->second.descriptor,
                                               to_it->second.descriptor);
    if (score_out != nullptr) {
      *score_out = score;
    }

    return std::isfinite(score) && score >= similarity_threshold_;
  }

  void ProcessPendingCandidatesLocked() {
    if (pending_candidates_.empty()) {
      return;
    }

    std::unordered_map<int, pose_graph_msgs::LoopCandidateArray> outputs;
    std::deque<PendingCandidate> still_waiting;
    const ros::Time now = ros::Time::now();

    for (const auto& pending : pending_candidates_) {
      const bool has_from = HasDescriptor(pending.candidate.key_from);
      const bool has_to = HasDescriptor(pending.candidate.key_to);
      if (!has_from || !has_to) {
        const double waited = (now - pending.enqueued_at).toSec();
        if (waited < max_candidate_wait_sec_) {
          still_waiting.push_back(pending);
          continue;
        }

        if (pass_missing_after_timeout_) {
          outputs[pending.originator].originator = pending.originator;
          outputs[pending.originator].candidates.push_back(pending.candidate);
          ROS_WARN_STREAM_THROTTLE(
              2.0,
              "SOLiD passing candidate after descriptor timeout: "
                  << gtsam::DefaultKeyFormatter(pending.candidate.key_from)
                  << " -> "
                  << gtsam::DefaultKeyFormatter(pending.candidate.key_to));
        } else {
          ROS_WARN_STREAM_THROTTLE(
              2.0,
              "SOLiD dropped candidate with missing descriptor: "
                  << gtsam::DefaultKeyFormatter(pending.candidate.key_from)
                  << " -> "
                  << gtsam::DefaultKeyFormatter(pending.candidate.key_to));
        }
        continue;
      }

      double score = 0.0;
      if (CandidatePassesLocked(pending.candidate, &score)) {
        outputs[pending.originator].originator = pending.originator;
        outputs[pending.originator].candidates.push_back(pending.candidate);
        ROS_INFO_STREAM_THROTTLE(
            1.0,
            "SOLiD accepted candidate "
                << gtsam::DefaultKeyFormatter(pending.candidate.key_from)
                << " -> "
                << gtsam::DefaultKeyFormatter(pending.candidate.key_to)
                << " score=" << score);
      } else if (publish_rejected_debug_) {
        ROS_INFO_STREAM(
            "SOLiD rejected candidate "
            << gtsam::DefaultKeyFormatter(pending.candidate.key_from)
            << " -> " << gtsam::DefaultKeyFormatter(pending.candidate.key_to)
            << " score=" << score);
      }
    }

    pending_candidates_.swap(still_waiting);

    for (auto& output_pair : outputs) {
      if (!output_pair.second.candidates.empty()) {
        output_pair.second.header.stamp = now;
        candidate_pub_.publish(output_pair.second);
      }
    }
  }

  struct PendingCandidate {
    pose_graph_msgs::LoopCandidate candidate;
    int originator = 0;
    ros::Time enqueued_at;
  };

  ros::NodeHandle nh_;
  ros::Subscriber keyed_scan_sub_;
  ros::Subscriber candidate_sub_;
  ros::Publisher candidate_pub_;
  ros::Timer retry_timer_;

  mutable SOLiDModule solid_;
  std::unordered_map<uint64_t, DescriptorEntry> descriptors_;
  std::deque<PendingCandidate> pending_candidates_;
  std::mutex mutex_;

  bool enable_ = false;
  bool pass_missing_after_timeout_ = true;
  bool publish_rejected_debug_ = false;
  double similarity_threshold_ = 0.80;
  double max_candidate_wait_sec_ = 5.0;
  double retry_period_sec_ = 0.25;
  int min_filtered_points_ = 30;
};

} // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "solid_loop_condition");
  ros::NodeHandle nh("~");

  SolidLoopConditionNode node(nh);
  if (!node.Initialize()) {
    return EXIT_FAILURE;
  }

  ros::spin();
  return EXIT_SUCCESS;
}
