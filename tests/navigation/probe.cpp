// 行协议仅供离线测试：直接执行生产控制器/A*，无需 DDS 网络通信。
#include <iostream>
#include <iomanip>
#include <rcl/time.h>
#include "pid_controller.hpp"
#include "astar_planner.hpp"

class Probe : public pid_controller::PidController {
public:
  void map(const nav_msgs::msg::OccupancyGrid & m) {
    costmap_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(m);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("controller_server");
  auto planner_node = std::make_shared<rclcpp::Node>("planner_server");
  auto clock = node->get_clock()->get_clock_handle();
  if (rcl_enable_ros_time_override(clock) != RCL_RET_OK) return 2;
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  Probe controller;
  controller.configure(node, "PidController", buffer);
  sp_global_planner::AStarPlanner planner;
  planner.configure(planner_node, "AStar");
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  std::cin >> map.info.width >> map.info.height >> map.info.resolution
           >> map.info.origin.position.x >> map.info.origin.position.y;
  map.info.origin.orientation.w = 1.0;
  map.data.resize(map.info.width * map.info.height);
  for (auto & c : map.data) {int value; std::cin >> value; c = value;}
  planner.setMap(map);
  controller.map(map);
  std::string op;
  std::cout << std::setprecision(12);
  while (std::cin >> op) {
    double time, x, y, yaw, vx, vy;
    std::cin >> time >> x >> y >> yaw >> vx >> vy;
    if (rcl_set_ros_time_override(clock, static_cast<int64_t>(time * 1e9)) != RCL_RET_OK) return 3;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x; pose.pose.position.y = y;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    if (op == "plan") {
      auto goal = pose;
      goal.pose.position.x = 14.1; goal.pose.position.y = 14.1;
      auto path = planner.createPlan(pose, goal);
      controller.setPlan(path);
      std::cout << "PATH " << path.poses.size();
      for (const auto & p : path.poses) std::cout << ' ' << p.pose.position.x << ' ' << p.pose.position.y;
      std::cout << std::endl;
    } else {
      geometry_msgs::msg::Twist velocity;
      velocity.linear.x = vx; velocity.linear.y = vy;
      auto cmd = controller.computeVelocityCommands(pose, velocity);
      std::cout << "CMD " << cmd.twist.linear.x << ' ' << cmd.twist.linear.y << std::endl;
    }
  }
  rclcpp::shutdown();
}
