// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#define SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace small_gicp_relocalization
{

class SmallGicpRelocalizationNode : public rclcpp::Node
{
public:
  explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  void performRegistration();
  void publishTransform();
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void publishDiagnostics(double inlier_ratio, double correction_m, double correction_yaw);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  int num_threads_;
  int num_neighbors_;
  float global_leaf_size_;
  float registered_leaf_size_;
  float max_dist_sq_;
  std::vector<double> init_pose_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  std::string current_scan_frame_id_;
  std::string input_cloud_topic_;
  rclcpp::Time last_scan_time_;
  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d previous_result_t_;

  // 配准验收（2026-11-13 可靠性调优）：不合格的结果不更新 TF，连续拒绝进入降级冻结。
  double min_inlier_ratio_;
  double min_inlier_ratio_recovery_;
  double relocalize_min_inlier_gap_;
  double max_correction_trans_;
  double max_correction_yaw_;
  int degraded_after_rejects_;
  bool have_accepted_{false};
  bool degraded_{false};
  int consecutive_rejects_{0};
  double last_inlier_ratio_{0.0};
  double last_correction_m_{0.0};
  double last_correction_yaw_{0.0};
  // 连续多次配准收敛到同一个「大修正」时，判为真实位移并重新锚定（防止被撞后永久冻结）。
  double reanchor_consistency_trans_;
  double reanchor_consistency_yaw_;
  int reanchor_after_stable_;
  Eigen::Isometry3d candidate_t_;
  bool have_candidate_{false};
  int candidate_stable_{0};

  // 降级后的恢复（2026-11-13 T2c 实测：大位移下修正抖动 20 cm，严格收敛永远凑不齐）：
  // 1) 宽松重锚：连续 N 次落在首个候选 ±relaxed 内即接受；
  // 2) 兜底全局重定位：DEGRADED 持续超过阈值后，沿当前位姿做多初值 GICP 搜索。
  double reanchor_relaxed_trans_;
  double reanchor_relaxed_yaw_;
  int reanchor_relaxed_after_;
  int relaxed_stable_{0};
  double relocalize_after_s_;
  double relocalize_grid_xy_;
  double relocalize_grid_yaw_;
  int relocalize_steps_xy_;
  int relocalize_steps_yaw_;
  int relocalize_min_agreement_;
  rclcpp::Time degraded_since_;
  bool relocalize_attempted_{false};
  bool globalRelocalize();
  void captureTrustedPose();

  // 手动初值（/initialpose）先校验再接受（2026-11-13 R2）。
  Eigen::Isometry3d pending_init_t_;
  bool have_pending_init_{false};

  // DEGRADED 期间冻结的是「全局位姿」而不是 map->odom：用 odom->base 的实时值补偿
  // （遮挡期间 LIO 的 IMU 积分会让 odom->base 漂几米，只冻结 map->odom 会把全局位姿拖走）。
  bool have_frozen_base_{false};
  Eigen::Isometry3d frozen_map_base_{Eigen::Isometry3d::Identity()};
  bool have_trusted_{false};
  Eigen::Isometry3d last_trusted_map_base_{Eigen::Isometry3d::Identity()};
  rclcpp::Time last_accept_time_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
