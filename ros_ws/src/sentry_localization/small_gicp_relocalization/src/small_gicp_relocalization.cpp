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

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <array>
#include <cmath>
#include <string>

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2/time.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");
  // 配准验收（见 docs/localization_roadmap.md）：不合格不更新 TF，连续拒绝进入降级冻结。
  this->declare_parameter("min_inlier_ratio", 0.60);
  // 恢复路径（大修正/重锚/全局重定位）的强门限：T2 实测中 0.61-0.72 的 inlier 也能连续稳定，
  // 但那是走廊里的错误匹配，必须用更高门槛拒绝。
  this->declare_parameter("min_inlier_ratio_recovery", 0.85);
  this->declare_parameter("relocalize_min_inlier_gap", 0.10);
  // 观测覆盖：inlier 是「匹配点/源点」，单侧被挡时源点变少但仍全匹配，比例不会掉。
  this->declare_parameter("min_sector_coverage", 0.60);
  this->declare_parameter("min_source_points", 300);
  this->declare_parameter("max_correction_trans", 0.30);
  this->declare_parameter("max_correction_yaw", 0.20);
  this->declare_parameter("degraded_after_rejects", 6);
  this->declare_parameter("reanchor_consistency_trans", 0.05);
  this->declare_parameter("reanchor_consistency_yaw", 0.03);
  this->declare_parameter("reanchor_after_stable", 3);
  this->declare_parameter("reanchor_relaxed_trans", 0.20);
  this->declare_parameter("reanchor_relaxed_yaw", 0.05);
  this->declare_parameter("reanchor_relaxed_after", 0);  // 0 = 关闭
  this->declare_parameter("relocalize_after_s", 10.0);
  this->declare_parameter("relocalize_grid_xy", 1.0);
  this->declare_parameter("relocalize_grid_yaw", 0.5);
  this->declare_parameter("relocalize_steps_xy", 5);
  this->declare_parameter("relocalize_steps_yaw", 3);
  this->declare_parameter("relocalize_min_agreement", 2);

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);
  this->get_parameter("min_inlier_ratio", min_inlier_ratio_);
  this->get_parameter("min_inlier_ratio_recovery", min_inlier_ratio_recovery_);
  this->get_parameter("relocalize_min_inlier_gap", relocalize_min_inlier_gap_);
  this->get_parameter("min_sector_coverage", min_sector_coverage_);
  this->get_parameter("min_source_points", min_source_points_);
  this->get_parameter("max_correction_trans", max_correction_trans_);
  this->get_parameter("max_correction_yaw", max_correction_yaw_);
  this->get_parameter("degraded_after_rejects", degraded_after_rejects_);
  this->get_parameter("reanchor_consistency_trans", reanchor_consistency_trans_);
  this->get_parameter("reanchor_consistency_yaw", reanchor_consistency_yaw_);
  this->get_parameter("reanchor_after_stable", reanchor_after_stable_);
  this->get_parameter("reanchor_relaxed_trans", reanchor_relaxed_trans_);
  this->get_parameter("reanchor_relaxed_yaw", reanchor_relaxed_yaw_);
  this->get_parameter("reanchor_relaxed_after", reanchor_relaxed_after_);
  this->get_parameter("relocalize_after_s", relocalize_after_s_);
  this->get_parameter("relocalize_grid_xy", relocalize_grid_xy_);
  this->get_parameter("relocalize_grid_yaw", relocalize_grid_yaw_);
  this->get_parameter("relocalize_steps_xy", relocalize_steps_xy_);
  this->get_parameter("relocalize_steps_yaw", relocalize_steps_yaw_);
  this->get_parameter("relocalize_min_agreement", relocalize_min_agreement_);

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  last_accept_time_ = this->now();

  loadGlobalMap(prior_pcd_file_);

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 2 Hz
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
  Eigen::Affine3d odom_to_lidar_odom;
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                              << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  odom_to_lidar_odom.linear().setIdentity();
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  *accumulated_cloud_ += *scan;
}

void SmallGicpRelocalizationNode::performRegistration()
{
  if (accumulated_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  // 观测覆盖检查（OBS_WEAK 只上报，不冻结）
  double coverage = 1.0;
  if (computeObservationCoverage(coverage)) {
    last_coverage_ = coverage;
    obs_weak_ =
      coverage < min_sector_coverage_ || static_cast<int>(source_->size()) < min_source_points_;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  // 手动初值可能离真值较远，给更多迭代；正常跟踪保持 10
  // 首次接受/手动初值给更多迭代：车不在原点附近时 10 次迭代可能不收敛
  register_->optimizer.max_iterations = (have_pending_init_ || !have_accepted_) ? 30 : 10;

  // 有手动初值时从初值出发配准（R2：先校验再接受）
  const Eigen::Isometry3d init = have_pending_init_ ? pending_init_t_ : previous_result_t_;
  auto result = register_->align(*target_, *source_, *target_tree_, init);

  // ---- 配准验收 ----
  const size_t n_source = source_ ? source_->size() : 0u;
  const double inlier_ratio =
    (n_source > 0u) ? static_cast<double>(result.num_inliers) / static_cast<double>(n_source) : 0.0;

  // ---- 手动初值（/initialpose）校验：不通过则保持原锚点 ----
  if (have_pending_init_) {
    have_pending_init_ = false;
    if (result.converged && inlier_ratio >= min_inlier_ratio_recovery_) {
      result_t_ = previous_result_t_ = result.T_target_source;
      have_accepted_ = true;
      captureTrustedPose();
      degraded_ = false;
      consecutive_rejects_ = 0;
      have_candidate_ = false;
      candidate_stable_ = 0;
      relaxed_stable_ = 0;
      relocalize_attempted_ = false;
      last_accept_time_ = this->now();
      RCLCPP_WARN(
        this->get_logger(), "手动初值校验通过（inlier=%.2f），接受为新锚点", inlier_ratio);
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "手动初值校验失败（converged=%d, inlier=%.2f，要求 ≥%.2f），拒绝并保持原锚点",
        static_cast<int>(result.converged), inlier_ratio, min_inlier_ratio_recovery_);
    }
    publishDiagnostics(inlier_ratio, 0.0, 0.0);
    accumulated_cloud_->clear();
    return;
  }

  // 相对上一次「已接受」位姿的修正量（首次接受不做修正量门限）
  const Eigen::Isometry3d delta = result_t_.inverse() * result.T_target_source;
  const double correction_m = delta.translation().norm();
  const double correction_yaw =
    std::fabs(std::atan2(delta.rotation()(1, 0), delta.rotation()(0, 0)));

  // 高 inlier 时可以接受未"形式收敛"的结果（GICP 迭代到上限但匹配已很好）
  const bool converged_ok = result.converged || inlier_ratio >= min_inlier_ratio_recovery_;
  bool accepted = converged_ok && inlier_ratio >= min_inlier_ratio_;
  if (
    accepted && have_accepted_ &&
    (correction_m > max_correction_trans_ || correction_yaw > max_correction_yaw_)) {
    accepted = false;
  }

  if (accepted) {
    result_t_ = previous_result_t_ = result.T_target_source;
    have_accepted_ = true;
    captureTrustedPose();
    consecutive_rejects_ = 0;
    have_candidate_ = false;
    candidate_stable_ = 0;
    relaxed_stable_ = 0;
    relocalize_attempted_ = false;
    last_accept_time_ = this->now();
    if (degraded_) {
      degraded_ = false;
      RCLCPP_WARN(this->get_logger(), "配准恢复：回到 NORMAL（inlier=%.2f）", inlier_ratio);
    }
  } else {
    ++consecutive_rejects_;

    // 连续多次配准都收敛到同一「大修正」⇒ 判为真实位移（被撞/被推），重新锚定。
    bool strict_close = false;
    bool relaxed_close = false;
    if (have_candidate_) {
      const Eigen::Isometry3d dc = candidate_t_.inverse() * result.T_target_source;
      const double dtrans = dc.translation().norm();
      const double dyaw = std::fabs(std::atan2(dc.rotation()(1, 0), dc.rotation()(0, 0)));
      strict_close = dtrans <= reanchor_consistency_trans_ && dyaw <= reanchor_consistency_yaw_;
      relaxed_close = dtrans <= reanchor_relaxed_trans_ && dyaw <= reanchor_relaxed_yaw_;
    }
    // 恢复门限：inlier 必须达到强门限，且偏航变化受限（T2 中 1.2 m / 38° 的错误匹配就是靠这条拒绝）
    const bool usable_recovery = result.converged && inlier_ratio >= min_inlier_ratio_recovery_ &&
                                 correction_yaw <= max_correction_yaw_;
    const bool relaxed_enabled = reanchor_relaxed_after_ > 0;
    if (usable_recovery && strict_close) {
      ++candidate_stable_;
      ++relaxed_stable_;
    } else if (usable_recovery && relaxed_enabled && relaxed_close) {
      // 大位移下修正在 ±relaxed 内抖动：宽松计数，仍以第一个候选为基准
      candidate_stable_ = 0;
      ++relaxed_stable_;
    } else {
      candidate_t_ = result.T_target_source;
      candidate_stable_ = 1;
      relaxed_stable_ = 1;
      have_candidate_ = true;
    }

    bool reanchored = false;
    const bool strict_ok = candidate_stable_ >= reanchor_after_stable_;
    const bool relaxed_ok = relaxed_enabled && relaxed_stable_ >= reanchor_relaxed_after_;
    if (usable_recovery && (strict_ok || relaxed_ok)) {
      RCLCPP_WARN(
        this->get_logger(),
        "连续 %d 次配准收敛到同一结果（严格）/%d 次（宽松），inlier=%.2f，判为真实位移，重新锚定",
        candidate_stable_, relaxed_stable_, inlier_ratio);
      result_t_ = previous_result_t_ = candidate_t_;
      have_accepted_ = true;
      captureTrustedPose();
      degraded_ = false;
      consecutive_rejects_ = 0;
      have_candidate_ = false;
      candidate_stable_ = 0;
      relaxed_stable_ = 0;
      relocalize_attempted_ = false;
      last_accept_time_ = this->now();
      reanchored = true;
    }

    if (!reanchored) {
      if (!degraded_ && consecutive_rejects_ >= degraded_after_rejects_) {
        degraded_ = true;
        degraded_since_ = this->now();
        RCLCPP_ERROR(
          this->get_logger(),
          "连续 %d 次配准未通过验收，进入 DEGRADED：冻结 map->odom，继续沿用最后可信位姿",
          consecutive_rejects_);
      }
      if (
        degraded_ && !relocalize_attempted_ &&
        (this->now() - degraded_since_).seconds() >= relocalize_after_s_) {
        relocalize_attempted_ = true;
        if (globalRelocalize()) {
          RCLCPP_WARN(this->get_logger(), "全局重定位成功，重新锚定 map->odom 并回到 NORMAL");
          degraded_ = false;
          consecutive_rejects_ = 0;
          have_candidate_ = false;
          candidate_stable_ = 0;
          relaxed_stable_ = 0;
          last_accept_time_ = this->now();
        } else {
          RCLCPP_ERROR(this->get_logger(), "全局重定位失败，继续冻结 map->odom");
        }
      }
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "配准未通过验收：converged=%d inlier=%.2f (min %.2f) 修正 %.3f m / %.3f rad "
      "(max %.2f / %.2f)，累计拒绝 %d 次，候选稳定 %d/%d",
      static_cast<int>(result.converged), inlier_ratio, min_inlier_ratio_, correction_m,
      correction_yaw, max_correction_trans_, max_correction_yaw_, consecutive_rejects_,
      candidate_stable_, reanchor_after_stable_);
  }

  last_inlier_ratio_ = inlier_ratio;
  last_correction_m_ = correction_m;
  last_correction_yaw_ = correction_yaw;
  publishDiagnostics(inlier_ratio, correction_m, correction_yaw);

  accumulated_cloud_->clear();
}

bool SmallGicpRelocalizationNode::globalRelocalize()
{
  if (!source_ || !target_ || !target_tree_ || source_->empty()) {
    return false;
  }

  struct Cand
  {
    Eigen::Isometry3d T;
    double inlier;
  };
  std::vector<Cand> good;

  const int nxy = std::max(1, relocalize_steps_xy_);
  const int nyaw = std::max(1, relocalize_steps_yaw_);
  const double n_source = static_cast<double>(source_->size());
  register_->optimizer.max_iterations = 15;

  // 搜索中心用「最后可信的全局位姿」而不是当前 result_t_：
  // LIO 断流后 odom 可能漂出几米，围着当前位姿搜永远够不到真值（2026-11-13 实测）。
  Eigen::Isometry3d center = result_t_;
  if (have_trusted_) {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
      center = last_trusted_map_base_ * tf2::transformToEigen(tf.transform).inverse();
    } catch (const tf2::TransformException &) {
      // 拿不到 TF 就用当前 result_t_
    }
  }

  for (int ix = 0; ix < nxy; ++ix) {
    const double dx = (nxy == 1) ? 0.0
                                 : -relocalize_grid_xy_ + 2.0 * relocalize_grid_xy_ *
                                                            static_cast<double>(ix) / (nxy - 1);
    for (int iy = 0; iy < nxy; ++iy) {
      const double dy = (nxy == 1) ? 0.0
                                   : -relocalize_grid_xy_ + 2.0 * relocalize_grid_xy_ *
                                                              static_cast<double>(iy) / (nxy - 1);
      for (int ia = 0; ia < nyaw; ++ia) {
        const double dyaw = (nyaw == 1)
                              ? 0.0
                              : -relocalize_grid_yaw_ +
                                  2.0 * relocalize_grid_yaw_ * static_cast<double>(ia) / (nyaw - 1);
        Eigen::Isometry3d init = center;
        init.translation() += Eigen::Vector3d(dx, dy, 0.0);
        init.linear() =
          Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * init.linear();
        auto res = register_->align(*target_, *source_, *target_tree_, init);
        const double inl = (n_source > 0.0) ? static_cast<double>(res.num_inliers) / n_source : 0.0;
        if (res.converged && inl >= min_inlier_ratio_recovery_) {
          good.push_back({res.T_target_source, inl});
        }
      }
    }
  }

  // 一致性：取与最多其它候选相近的那个（避免对称场景的误匹配）
  int best_i = -1;
  int best_agree = 0;
  double best_inl = 0.0;
  for (size_t i = 0; i < good.size(); ++i) {
    int agree = 0;
    for (size_t j = 0; j < good.size(); ++j) {
      if (i == j) {
        continue;
      }
      const Eigen::Isometry3d dc = good[i].T.inverse() * good[j].T;
      const double dtrans = dc.translation().norm();
      const double dyaw = std::fabs(std::atan2(dc.rotation()(1, 0), dc.rotation()(0, 0)));
      if (dtrans <= 0.10 && dyaw <= 0.05) {
        ++agree;
      }
    }
    if (agree > best_agree || (agree == best_agree && good[i].inlier > best_inl)) {
      best_agree = agree;
      best_i = static_cast<int>(i);
      best_inl = good[i].inlier;
    }
  }

  // 歧义检查：与最佳候选「不同解」的最高 inlier，必须明显低于最佳解
  double other_inl = 0.0;
  if (best_i >= 0) {
    for (size_t j = 0; j < good.size(); ++j) {
      if (static_cast<int>(j) == best_i) {
        continue;
      }
      const Eigen::Isometry3d dc = good[best_i].T.inverse() * good[j].T;
      const double dtrans = dc.translation().norm();
      const double dyaw = std::fabs(std::atan2(dc.rotation()(1, 0), dc.rotation()(0, 0)));
      if (dtrans > 0.10 || dyaw > 0.05) {
        other_inl = std::max(other_inl, good[j].inlier);
      }
    }
  }
  const bool unambiguous = (best_inl - other_inl) >= relocalize_min_inlier_gap_;
  const bool strong_enough = best_inl >= min_inlier_ratio_recovery_;

  RCLCPP_WARN(
    this->get_logger(),
    "全局重定位：%zu/%d 个初值通过验收，最佳一致性 %d（要求 %d），inlier %.2f 次优解 %.2f "
    "(差 %.2f，要求 %.2f)，强门限 %.2f",
    good.size(), nxy * nxy * nyaw, best_agree + 1, relocalize_min_agreement_, best_inl, other_inl,
    best_inl - other_inl, relocalize_min_inlier_gap_, min_inlier_ratio_recovery_);

  if (best_i >= 0 && best_agree + 1 >= relocalize_min_agreement_ && strong_enough && unambiguous) {
    result_t_ = previous_result_t_ = good[best_i].T;
    have_accepted_ = true;
    captureTrustedPose();
    return true;
  }
  return false;
}

void SmallGicpRelocalizationNode::publishDiagnostics(
  double inlier_ratio, double correction_m, double correction_yaw)
{
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "localization/gicp";
  st.hardware_id = "small_gicp";
  st.level = degraded_ ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                       : diagnostic_msgs::msg::DiagnosticStatus::OK;
  const char * state = degraded_ ? "DEGRADED" : (obs_weak_ ? "OBS_WEAK" : "NORMAL");
  st.message = degraded_ ? "DEGRADED (map->odom frozen)"
                         : (obs_weak_ ? "OBS_WEAK (partial observation)" : "NORMAL");
  st.level = (degraded_ || obs_weak_) ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                                      : diagnostic_msgs::msg::DiagnosticStatus::OK;

  auto add = [&st](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    st.values.push_back(kv);
  };
  add("state", state);
  add("sector_coverage", std::to_string(last_coverage_));
  add("source_points", std::to_string(source_ ? source_->size() : 0));
  add("inlier_ratio", std::to_string(inlier_ratio));
  add("correction_m", std::to_string(correction_m));
  add("correction_yaw_rad", std::to_string(correction_yaw));
  add("consecutive_rejects", std::to_string(consecutive_rejects_));
  add("last_accept_age_s", std::to_string((this->now() - last_accept_time_).seconds()));

  arr.status.push_back(st);
  diag_pub_->publish(arr);
}

bool SmallGicpRelocalizationNode::computeObservationCoverage(double & coverage)
{
  coverage = 1.0;
  if (!source_ || source_->empty()) {
    coverage = 0.0;
    return true;
  }
  try {
    const auto tf = tf_buffer_->lookupTransform(
      odom_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
    const Eigen::Vector3d r = tf2::transformToEigen(tf.transform).translation();
    constexpr int kSectors = 36;
    std::array<bool, kSectors> hit{};
    for (const auto & p : source_->points) {
      const double dx = p.x - r.x();
      const double dy = p.y - r.y();
      if (dx * dx + dy * dy < 0.25) {
        continue;  // 0.5 m 内忽略（车体/近场）
      }
      int k = static_cast<int>((std::atan2(dy, dx) + M_PI) / (2.0 * M_PI) * kSectors) % kSectors;
      if (k < 0) k += kSectors;
      hit[k] = true;
    }
    int c = 0;
    for (bool h : hit) c += h ? 1 : 0;
    coverage = static_cast<double>(c) / kSectors;
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

void SmallGicpRelocalizationNode::captureTrustedPose()
{
  // 只在配准被接受时采样「最后可信的全局位姿」：odom 可能在 LIO 恢复瞬间跳变，
  // 若在发布时采样会把跳变后的错误位姿当成可信值。
  try {
    const auto tf = tf_buffer_->lookupTransform(
      odom_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
    last_trusted_map_base_ = result_t_ * tf2::transformToEigen(tf.transform);
    have_trusted_ = true;
  } catch (const tf2::TransformException &) {
    // 拿不到 TF 就保留上一次
  }
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) {
    return;
  }

  Eigen::Isometry3d output = result_t_;
  try {
    const auto tf = tf_buffer_->lookupTransform(
      odom_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
    const Eigen::Isometry3d odom_base = tf2::transformToEigen(tf.transform);
    if (!degraded_) {
      have_frozen_base_ = false;
    } else if (have_trusted_) {
      // 冻结全局位姿：map->odom = 冻结的 map->base · (odom->base)^-1
      if (!have_frozen_base_) {
        frozen_map_base_ = last_trusted_map_base_;
        have_frozen_base_ = true;
        RCLCPP_WARN(
          this->get_logger(), "DEGRADED：冻结全局位姿 (%.2f, %.2f)",
          frozen_map_base_.translation().x(), frozen_map_base_.translation().y());
      }
      output = frozen_map_base_ * odom_base.inverse();
    }
  } catch (const tf2::TransformException &) {
    // TF 拿不到就沿用上一次的 map->odom
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // `+ 0.1` means transform into future. according to https://robotics.stackexchange.com/a/96615
  transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = output.translation();
  const Eigen::Quaterniond rotation(output.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    // 不直接接受：交给下一次配准校验（inlier ≥ min_inlier_ratio_recovery 才采纳）
    pending_init_t_ = map_to_odom;
    have_pending_init_ = true;
    RCLCPP_INFO(
      this->get_logger(), "收到手动初值，等待下一次配准校验（inlier ≥ %.2f）",
      min_inlier_ratio_recovery_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
