#ifndef SENTRY_DECISION__SENTRY_DECISION_NODE_HPP_
#define SENTRY_DECISION__SENTRY_DECISION_NODE_HPP_

#include <memory>
#include <vector>
#include <string>
#include <deque>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "sentry_interfaces/msg/robot_status.hpp"
#include "sentry_interfaces/msg/gimbal_data.hpp"
#include "sentry_interfaces/msg/auto_aim_cmd.hpp"
#include "sentry_decision/msg/cmd_vel_udp.hpp"

namespace sentry_decision
{

class SentryDecisionNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  explicit SentryDecisionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 回调函数
  void robotStatusCallback(const sentry_interfaces::msg::RobotStatus::SharedPtr msg);
  void gimbalDataCallback(const sentry_interfaces::msg::GimbalData::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void autoAimCmdCallback(const sentry_interfaces::msg::AutoAimCmd::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();
  void cmdVelUdpTimerCallback();

  // === NavigateToPose 逐点导航 ===
  // 启动导航序列：将途径点+最终目标加入队列，依次发送
  void startNavigationSequence(
    const std::vector<double> & target_pose,
    const std::vector<std::vector<double>> & waypoints);

  // 取消当前正在执行的导航目标（异步），完成后自动从队列发送下一个
  void cancelCurrentGoalAndSendNext(bool start_new_sequence = false);

  // 从队列发送下一个目标点（navigate_to_pose）
  void sendNextWaypoint();

  // navigate_to_pose 回调
  void navToPoseGoalResponseCallback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
  void navToPoseFeedbackCallback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void navToPoseResultCallback(const GoalHandleNavigateToPose::WrappedResult & result);

  // 构建 PoseStamped（从 [x, y, yaw] 向量）
  geometry_msgs::msg::PoseStamped makePoseStamped(const std::vector<double> & pose3);

  // 状态枚举
  enum class State {
    IDLE,
    GOING_TO_X,  // 正在导航到X
    GOING_TO_Y,  // 正在导航到Y
    AT_X,        // 已到达X，静止等待
    AT_Y         // 已到达Y，静止等待（补血中）
  };

  // 订阅者
  rclcpp::Subscription<sentry_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<sentry_interfaces::msg::GimbalData>::SharedPtr gimbal_data_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sentry_interfaces::msg::AutoAimCmd>::SharedPtr auto_aim_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // 发布者
  rclcpp::Publisher<sentry_decision::msg::CmdVelUdp>::SharedPtr cmd_vel_udp_pub_;

  // navigate_to_pose action 客户端
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr cmd_vel_pub_timer_;

  // 当前 navigate_to_pose goal handle
  GoalHandleNavigateToPose::SharedPtr current_goal_handle_;

  State current_state_;
  uint16_t current_hp_;
  uint16_t maximum_hp_;
  uint8_t  game_progress_;
  uint16_t stage_remain_time_;

  // 参数
  int hp_threshold_low_;
  int hp_threshold_high_;
  double yaw_drift_tolerance_;
  std::vector<double> target_x_;
  std::vector<double> target_y_;
  std::vector<std::vector<double>> waypoints_to_x_;
  std::vector<std::vector<double>> waypoints_to_y_;

  // 逐点导航队列（每个元素是 [x, y, yaw]）
  std::deque<std::vector<double>> waypoint_queue_;
  // 标记当前导航序列是否在执行中
  bool nav_sequence_active_{false};
  // 标记是否有挂起的新序列请求（取消旧目标期间到来的新目标）
  bool has_pending_sequence_{false};
  std::vector<double> pending_target_pose_;
  std::vector<std::vector<double>> pending_waypoints_;

  // 最新视觉自瞄命令
  sentry_interfaces::msg::AutoAimCmd::SharedPtr latest_auto_aim_cmd_;

  // 缓存最新 cmd_vel
  geometry_msgs::msg::Twist latest_cmd_vel_;
  bool goal_reached_{false};

  // 朝向漂移检测
  double current_yaw_{0.0};
  bool   has_pose_{false};
};

}  // namespace sentry_decision

#endif  // SENTRY_DECISION__SENTRY_DECISION_NODE_HPP_
