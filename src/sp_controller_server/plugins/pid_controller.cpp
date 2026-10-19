
#include "pid_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/exceptions.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace {

[[noreturn]] void sp_nav_param_error(
  const rclcpp::Node & node,
  const std::string & name)
{
  std::ostringstream oss;
  oss << "[参数缺失] 节点 '" << node.get_name()
      << "' 缺少参数 '" << name
      << "'，请在对应 yaml 的 ros__parameters 中填写。";
  throw std::runtime_error(oss.str());
}

template<typename T>
T require_param(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name)
{
  try {
    if (node->has_parameter(name)) {
      return node->get_parameter(name).get_value<T>();
    }
    return node->declare_parameter<T>(name);
  } catch (
    const rclcpp::exceptions::
      UninitializedStaticallyTypedParameterException &)
  {
    sp_nav_param_error(*node, name);
  }
}

}  // namespace

namespace pid_controller {

// 1. 初始化 P 控制器
void PidController::configure(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  node_ = node;
  plugin_name_ = name;
  tf_buffer_ = tf_buffer;

  if (!node_) {
    throw std::runtime_error(
      "PidController received null node");
  }

  base_frame_id_ = require_param<std::string>(
    node_, plugin_name_ + ".base_frame_id");

  // 读取 P 控制器参数
  kp_ = require_param<double>(
    node_, plugin_name_ + ".kp");

  linear_speed_ = require_param<double>(
    node_, plugin_name_ + ".linear_speed");

  max_angular_speed_ = require_param<double>(
    node_, plugin_name_ + ".max_angular_speed");

  lookahead_distance_ = require_param<double>(
    node_, plugin_name_ + ".lookahead_distance");

  goal_tolerance_ = require_param<double>(
    node_, plugin_name_ + ".goal_tolerance");

  if (kp_ < 0.0 || linear_speed_ < 0.0 ||
      max_angular_speed_ <= 0.0 ||
      lookahead_distance_ <= 0.0 ||
      goal_tolerance_ <= 0.0) {
    throw std::runtime_error(
      "Invalid P controller parameters");
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "P controller initialized, Kp=%.2f", kp_);
}

// 2. 保存 A* 全局路径
void PidController::setPlan(
  const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

// 3. P 控制器：计算机器人速度
geometry_msgs::msg::TwistStamped
PidController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  (void)velocity; // 第一版暂时不用速度反馈

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node_->now();
  cmd_vel.header.frame_id = base_frame_id_;

  // 没有路径就保持停止
  if (global_plan_.poses.empty()) {
    return cmd_vel;
  }

  // 获取机器人当前位姿
  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double yaw = tf2::getYaw(pose.pose.orientation);

  // 获取路径终点
  const auto & goal = global_plan_.poses.back().pose.position;

  double goal_dist = std::hypot(
    goal.x - x, goal.y - y);

  // 已接近终点，停止运动
  if (goal_dist < goal_tolerance_) {
    return cmd_vel;
  }

  // 第一步：寻找距离机器人最近的路径点
  size_t nearest_index = 0;
  double min_dist = 1e9;

  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const auto & p = global_plan_.poses[i].pose.position;

    double dist = std::hypot(p.x - x, p.y - y);

    if (dist < min_dist) {
      min_dist = dist;
      nearest_index = i;
    }
  }

  // 第二步：沿路径向前选择目标点
  size_t target_index = nearest_index;
  double accumulated = 0.0;

  for (size_t i = nearest_index + 1;
       i < global_plan_.poses.size(); ++i) {

    const auto & previous =
      global_plan_.poses[i - 1].pose.position;

    const auto & current =
      global_plan_.poses[i].pose.position;

    accumulated += std::hypot(
      current.x - previous.x,
      current.y - previous.y);

    target_index = i;

    if (accumulated >= lookahead_distance_) {
      break;
    }
  }

  const auto & target =
    global_plan_.poses[target_index].pose.position;

    // 第三步：计算位置误差（map坐标系）
  double error_x = target.x - x;
  double error_y = target.y - y;

  // 第四步：P控制，计算map坐标系下的速度
  double vx_map = kp_ * error_x;
  double vy_map = kp_ * error_y ;

  // 第五步：限制合速度，防止运动过快
  double speed = std::hypot(vx_map, vy_map);

  if (speed > linear_speed_) {
      vx_map *= linear_speed_ / speed;
      vy_map *= linear_speed_ / speed;
  }

  // 第六步：从map坐标系转换到机器人坐标系
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);

  double vx_body =
      cos_yaw * vx_map + sin_yaw * vy_map;

  double vy_body =
      -sin_yaw * vx_map + cos_yaw * vy_map;

  // 第七步：输出二维速度
  cmd_vel.twist.linear.x = vx_body;
  cmd_vel.twist.linear.y = vy_body;
  cmd_vel.twist.angular.z = 0.0;

  return cmd_vel;
}

}  // namespace pid_controller

PLUGINLIB_EXPORT_CLASS(
  pid_controller::PidController,
  sp_controller_server::ControllerPlugin)
