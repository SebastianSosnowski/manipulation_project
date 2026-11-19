#ifndef PICK_AND_PLACE_HPP_
#define PICK_AND_PLACE_HPP_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

class PickAndPlace {
public:
  explicit PickAndPlace(rclcpp::Node::SharedPtr base_node);

  ~PickAndPlace();

  bool execute_trajectory_plan(
      const geometry_msgs::msg::PoseStamped &pre_grasp_pose);

private:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;

  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("PickAndPlace");

  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor move_group_executor_;
  std::thread move_group_thread_;

  Plan trajectory_plan_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_robot_;

  Plan trajectory_plan_gripper_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;

  std::vector<Pose> cartesian_waypoints_;
  moveit_msgs::msg::RobotTrajectory cartesian_trajectory_plan_;

  const double end_effector_step_{0.01};
  const double jump_threshold_{0.0};
  const double approach_distance_{0.060};

  bool plan_trajectory(MoveGroupInterface &move_group, Plan &trajectory_plan);
  bool execute_trajectory(MoveGroupInterface &move_group,
                           const Plan &trajectory_plan);
  void setup_named_target(MoveGroupInterface &move_group,
                          const std::string &target_name);
  void setup_joint_target(MoveGroupInterface &move_group,
                          const std::vector<double> &joint_positions);
  void setup_pose_target(MoveGroupInterface &move_group,
                         const geometry_msgs::msg::PoseStamped &target_pose);
  void setup_cartesian_waypoints(double dx, double dy, double dz);
  bool plan_cartesian_trajectory(
      MoveGroupInterface &move_group,
      moveit_msgs::msg::RobotTrajectory &trajectory);
  bool execute_cartesian_trajectory(
      MoveGroupInterface &move_group,
      const moveit_msgs::msg::RobotTrajectory &trajectory);
};

#endif // PICK_AND_PLACE_HPP_