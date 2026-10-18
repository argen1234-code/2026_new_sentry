#include "sentry_decision/sentry_decision_node.hpp"

#include <cmath>

namespace sentry_decision
{

SentryDecisionNode::SentryDecisionNode(const rclcpp::NodeOptions & options)
: Node("sentry_decision_node", options),
  current_state_(State::IDLE),
  current_goal_handle_(nullptr),
  current_hp_(0),
  maximum_hp_(0),
  game_progress_(0),
  stage_remain_time_(0)
{
  // 声明参数
  this->declare_parameter("hp_threshold_low", 100);
  this->declare_parameter("hp_threshold_high", 200);
  this->declare_parameter("yaw_drift_tolerance", 0.3);
  this->declare_parameter("target_x", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("target_y", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("waypoints_to_x", std::vector<double>{});
  this->declare_parameter("waypoints_to_y", std::vector<double>{});
  this->declare_parameter("robot_status_topic", "/robot_status");
  this->declare_parameter("gimbal_data_topic", "/gimbal_data");
  this->declare_parameter("auto_aim_cmd_topic", "/auto_aim_cmd");
  this->declare_parameter("check_interval", 1.0);

  // 获取参数
  hp_threshold_low_    = this->get_parameter("hp_threshold_low").as_int();
  hp_threshold_high_   = this->get_parameter("hp_threshold_high").as_int();
  yaw_drift_tolerance_ = this->get_parameter("yaw_drift_tolerance").as_double();
  target_x_ = this->get_parameter("target_x").as_double_array();
  target_y_ = this->get_parameter("target_y").as_double_array();

  auto waypoints_to_x_flat = this->get_parameter("waypoints_to_x").as_double_array();
  if (waypoints_to_x_flat.size() % 3 == 0) {
    for (size_t i = 0; i < waypoints_to_x_flat.size(); i += 3) {
      waypoints_to_x_.push_back({
        waypoints_to_x_flat[i],
        waypoints_to_x_flat[i + 1],
        waypoints_to_x_flat[i + 2]
      });
    }
  }

  auto waypoints_to_y_flat = this->get_parameter("waypoints_to_y").as_double_array();
  if (waypoints_to_y_flat.size() % 3 == 0) {
    for (size_t i = 0; i < waypoints_to_y_flat.size(); i += 3) {
      waypoints_to_y_.push_back({
        waypoints_to_y_flat[i],
        waypoints_to_y_flat[i + 1],
        waypoints_to_y_flat[i + 2]
      });
    }
  }

  // 创建订阅者
  std::string robot_status_topic = this->get_parameter("robot_status_topic").as_string();
  robot_status_sub_ = this->create_subscription<sentry_interfaces::msg::RobotStatus>(
    robot_status_topic, 10,
    std::bind(&SentryDecisionNode::robotStatusCallback, this, std::placeholders::_1));

  std::string gimbal_data_topic = this->get_parameter("gimbal_data_topic").as_string();
  gimbal_data_sub_ = this->create_subscription<sentry_interfaces::msg::GimbalData>(
    gimbal_data_topic, 10,
    std::bind(&SentryDecisionNode::gimbalDataCallback, this, std::placeholders::_1));

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&SentryDecisionNode::cmdVelCallback, this, std::placeholders::_1));

  std::string auto_aim_cmd_topic = this->get_parameter("auto_aim_cmd_topic").as_string();
  auto_aim_cmd_sub_ = this->create_subscription<sentry_interfaces::msg::AutoAimCmd>(
    auto_aim_cmd_topic, 10,
    std::bind(&SentryDecisionNode::autoAimCmdCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry", 10,
    std::bind(&SentryDecisionNode::odometryCallback, this, std::placeholders::_1));

  // 创建发布者
  cmd_vel_udp_pub_ = this->create_publisher<sentry_decision::msg::CmdVelUdp>(
    "/cmd_vel_udp", 10);

  // 创建 navigate_to_pose action 客户端
  nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
    this, "navigate_to_pose");

  // 创建定时器
  double check_interval = this->get_parameter("check_interval").as_double();
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(check_interval),
    std::bind(&SentryDecisionNode::timerCallback, this));

  // 100Hz 持续发布 cmd_vel_udp
  cmd_vel_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&SentryDecisionNode::cmdVelUdpTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "哨兵决策节点已启动（逐点导航模式）");
  RCLCPP_INFO(this->get_logger(), "血量低阈值: %d  高阈值: %d", hp_threshold_low_, hp_threshold_high_);
  RCLCPP_INFO(this->get_logger(), "目标点X: [%.2f, %.2f, %.2f]", target_x_[0], target_x_[1], target_x_[2]);
  RCLCPP_INFO(this->get_logger(), "目标点Y: [%.2f, %.2f, %.2f]", target_y_[0], target_y_[1], target_y_[2]);
  RCLCPP_INFO(this->get_logger(), "去往X途径点数: %zu  去往Y途径点数: %zu",
    waypoints_to_x_.size(), waypoints_to_y_.size());
}

// ==============================
// 传感器回调
// ==============================

void SentryDecisionNode::robotStatusCallback(
  const sentry_interfaces::msg::RobotStatus::SharedPtr msg)
{
  current_hp_ = msg->current_hp;
  maximum_hp_ = msg->maximum_hp;
}

void SentryDecisionNode::gimbalDataCallback(
  const sentry_interfaces::msg::GimbalData::SharedPtr msg)
{
  // 如果 game_progress_ 已经为 4，则保持不变，不再更新
  if (game_progress_ != 4) {
    game_progress_ = msg->game_progress;
  }
  stage_remain_time_ = msg->stage_remain_time;
  current_hp_        = msg->current_hp;
}

void SentryDecisionNode::autoAimCmdCallback(
  const sentry_interfaces::msg::AutoAimCmd::SharedPtr msg)
{
  latest_auto_aim_cmd_ = msg;
}

void SentryDecisionNode::cmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cmd_vel_ = *msg;
  goal_reached_ = false;
}

void SentryDecisionNode::odometryCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & q = msg->pose.pose.orientation;
  current_yaw_ = atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  has_pose_ = true;
}

// ==============================
// 100Hz 发布 cmd_vel_udp
// ==============================

void SentryDecisionNode::cmdVelUdpTimerCallback()
{
  sentry_decision::msg::CmdVelUdp cmd_vel_udp_msg;

  if (game_progress_ < 4) {
    cmd_vel_udp_msg.flag_wz = 2;
  } else if (game_progress_ == 4) {
    cmd_vel_udp_msg.flag_wz = 1;
  } else {
    cmd_vel_udp_msg.flag_wz = 2;
  }

  if (goal_reached_) {
    cmd_vel_udp_msg.vx = 0.0f;
    cmd_vel_udp_msg.vy = 0.0f;
    cmd_vel_udp_msg.wz = 0.0f;
  } else {
    cmd_vel_udp_msg.vx = static_cast<float>(latest_cmd_vel_.linear.x);
    cmd_vel_udp_msg.vy = static_cast<float>(latest_cmd_vel_.linear.y);
    cmd_vel_udp_msg.wz = static_cast<float>(latest_cmd_vel_.angular.z);
  }

  cmd_vel_udp_pub_->publish(cmd_vel_udp_msg);
}

// ==============================
// 决策定时器
// ==============================

void SentryDecisionNode::timerCallback()
{
  // 如果有挂起的新导航序列（等待旧目标取消中），等待取消完成
  if (has_pending_sequence_) {
    if (current_goal_handle_ == nullptr) {
      RCLCPP_INFO(this->get_logger(), "旧目标已取消，开始执行挂起的新导航序列");
      has_pending_sequence_ = false;
      startNavigationSequence(pending_target_pose_, pending_waypoints_);
    }
    return;
  }

  // 比赛状态检查
  if (game_progress_ != 4 && game_progress_ != 68) {
    if (current_state_ != State::IDLE) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "比赛未开始 (progress=%u)，等待中", game_progress_);
      // 比赛结束时取消当前导航
      if (nav_sequence_active_ && current_goal_handle_ != nullptr) {
        nav_to_pose_client_->async_cancel_goal(current_goal_handle_);
        current_goal_handle_ = nullptr;
      }
      nav_sequence_active_ = false;
      waypoint_queue_.clear();
      current_state_ = State::IDLE;
    }
    return;
  }

  // 双阈值血量决策
  if (current_hp_ < hp_threshold_low_) {
    if (current_state_ != State::GOING_TO_Y && current_state_ != State::AT_Y) {
      RCLCPP_WARN(this->get_logger(),
        "血量过低 (%d < %d)，前往目标点Y补血！", current_hp_, hp_threshold_low_);
      startNavigationSequence(target_y_, waypoints_to_y_);
      current_state_ = State::GOING_TO_Y;
    }
  } else if ((current_state_ == State::GOING_TO_Y || current_state_ == State::AT_Y) &&
             current_hp_ < hp_threshold_high_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "补血中 (%d / %d)，等待血量恢复到 %d", current_hp_, maximum_hp_, hp_threshold_high_);
  } else if (current_hp_ >= hp_threshold_high_ ||
             (current_state_ != State::GOING_TO_Y && current_state_ != State::AT_Y &&
              current_hp_ >= hp_threshold_low_)) {
    if (current_state_ != State::GOING_TO_X && current_state_ != State::AT_X) {
      RCLCPP_INFO(this->get_logger(),
        "血量充足 (%d >= %d)，前往目标点X [剩余%us]", current_hp_, hp_threshold_high_, stage_remain_time_);
      startNavigationSequence(target_x_, waypoints_to_x_);
      current_state_ = State::GOING_TO_X;
    }
  }
}

// ==============================
// 逐点导航逻辑
// ==============================

geometry_msgs::msg::PoseStamped SentryDecisionNode::makePoseStamped(
  const std::vector<double> & pose3)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = this->now();
  pose.pose.position.x = pose3[0];
  pose.pose.position.y = pose3[1];
  pose.pose.position.z = 0.0;
  double yaw = pose3[2];
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = sin(yaw / 2.0);
  pose.pose.orientation.w = cos(yaw / 2.0);
  return pose;
}

void SentryDecisionNode::startNavigationSequence(
  const std::vector<double> & target_pose,
  const std::vector<std::vector<double>> & waypoints)
{
  // 如果当前有正在执行的目标，先取消，再发起新序列
  if (current_goal_handle_ != nullptr) {
    auto status = current_goal_handle_->get_status();
    if (status == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
        status == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
      RCLCPP_INFO(this->get_logger(), "取消旧导航目标，准备启动新导航序列");
      pending_target_pose_ = target_pose;
      pending_waypoints_   = waypoints;
      has_pending_sequence_ = true;
      nav_to_pose_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_ = nullptr;
      return;
    }
    current_goal_handle_ = nullptr;
  }

  // 构建导航队列：途径点 + 最终目标
  waypoint_queue_.clear();
  for (const auto & wp : waypoints) {
    waypoint_queue_.push_back(wp);
  }
  waypoint_queue_.push_back(target_pose);

  nav_sequence_active_ = true;
  goal_reached_ = false;
  RCLCPP_INFO(this->get_logger(), "启动逐点导航序列，共 %zu 个点（含途径点和终点）",
    waypoint_queue_.size());

  sendNextWaypoint();
}

void SentryDecisionNode::sendNextWaypoint()
{
  if (waypoint_queue_.empty()) {
    // 队列已空，整个序列完成
    nav_sequence_active_ = false;
    goal_reached_ = true;
    latest_cmd_vel_ = geometry_msgs::msg::Twist();

    if (current_state_ == State::GOING_TO_X) {
      RCLCPP_INFO(this->get_logger(), "所有途径点+终点导航完成，已到达X点，原地待机");
      current_state_ = State::AT_X;
    } else if (current_state_ == State::GOING_TO_Y) {
      RCLCPP_INFO(this->get_logger(), "所有途径点+终点导航完成，已到达Y点（补血点），原地待机");
      current_state_ = State::AT_Y;
    } else {
      current_state_ = State::IDLE;
    }
    return;
  }

  if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "navigate_to_pose action 服务器不可用");
    nav_sequence_active_ = false;
    current_state_ = State::IDLE;
    return;
  }

  auto next_pose = waypoint_queue_.front();
  waypoint_queue_.pop_front();

  bool is_final = waypoint_queue_.empty();
  RCLCPP_INFO(this->get_logger(), "发送%s点: [%.2f, %.2f, %.2f]（剩余队列：%zu）",
    is_final ? "终点" : "途径",
    next_pose[0], next_pose[1], next_pose[2],
    waypoint_queue_.size());

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = makePoseStamped(next_pose);

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&SentryDecisionNode::navToPoseGoalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&SentryDecisionNode::navToPoseFeedbackCallback, this,
      std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&SentryDecisionNode::navToPoseResultCallback, this, std::placeholders::_1);

  nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
}

void SentryDecisionNode::navToPoseGoalResponseCallback(
  const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "导航目标被拒绝，重置状态");
    nav_sequence_active_ = false;
    waypoint_queue_.clear();
    current_state_ = State::IDLE;
    current_goal_handle_ = nullptr;
  } else {
    RCLCPP_INFO(this->get_logger(), "导航目标已接受");
    current_goal_handle_ = goal_handle;
  }
}

void SentryDecisionNode::navToPoseFeedbackCallback(
  GoalHandleNavigateToPose::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  RCLCPP_DEBUG(this->get_logger(), "导航中 - 当前位置: [%.2f, %.2f]，剩余距离: %.2f m",
    feedback->current_pose.pose.position.x,
    feedback->current_pose.pose.position.y,
    feedback->distance_remaining);
}

void SentryDecisionNode::navToPoseResultCallback(
  const GoalHandleNavigateToPose::WrappedResult & result)
{
  current_goal_handle_ = nullptr;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "到达当前点，继续发送下一个点（剩余 %zu 个）",
        waypoint_queue_.size());
      // 到达当前点后立即发送下一个点，无额外停顿
      // （Nav2 的 goal checker 判定到达后 action 就 SUCCEEDED，
      //  机器人会在该点附近短暂停止，然后立即收到下一个目标继续运动）
      sendNextWaypoint();
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "导航被中止（规划失败），跳过当前点，继续发送下一个点（剩余 %zu 个）",
        waypoint_queue_.size());
      // 不重置 state，跳过当前失败点，继续发送队列中下一个点
      sendNextWaypoint();
      break;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "导航被取消");
      // 如果是为了切换到新序列而主动取消，不清空队列（新序列会重建队列）
      if (!has_pending_sequence_) {
        nav_sequence_active_ = false;
        waypoint_queue_.clear();
        current_state_ = State::IDLE;
      }
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "导航结果未知，重置状态");
      nav_sequence_active_ = false;
      waypoint_queue_.clear();
      current_state_ = State::IDLE;
      break;
  }
}

}  // namespace sentry_decision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sentry_decision::SentryDecisionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
