#include "pick_and_place.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {
const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
const std::string PLANNING_GROUP_GRIPPER = "gripper";
} // namespace

PickAndPlace::PickAndPlace(rclcpp::Node::SharedPtr base_node)
    : base_node_(std::move(base_node)), logger_(base_node_->get_logger()) {
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  move_group_node_ =
      rclcpp::Node::make_shared("move_group_interface_node", node_options);
  move_group_executor_.add_node(move_group_node_);
  move_group_thread_ = std::thread([this]() { move_group_executor_.spin(); });

  try {
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);

    const auto current_state_robot = move_group_robot_->getCurrentState(10.0);
    const auto current_state_gripper =
        move_group_gripper_->getCurrentState(10.0);

    if (!current_state_robot || !current_state_gripper) {
      throw std::runtime_error("Failed to get current robot state");
    }

    if (!current_state_robot->getJointModelGroup(PLANNING_GROUP_ROBOT) ||
        !current_state_gripper->getJointModelGroup(PLANNING_GROUP_GRIPPER)) {
      throw std::runtime_error("Failed to get joint model groups");
    }
  } catch (...) {
    move_group_executor_.cancel();
    if (move_group_thread_.joinable()) {
      move_group_thread_.join();
    }
    throw;
  }
}

PickAndPlace::~PickAndPlace() {
  move_group_executor_.cancel();

  if (move_group_thread_.joinable()) {
    move_group_thread_.join();
  }
}

bool PickAndPlace::execute_trajectory_plan(
    const geometry_msgs::msg::PoseStamped &object_pose) {
  if (object_pose.header.frame_id.empty()) {
    RCLCPP_ERROR(logger_, "Object pose frame_id must not be empty");
    return false;
  }

  RCLCPP_INFO(logger_, "Starting pick and place sequence");

  setup_named_target(*move_group_robot_, "home");
  if (!plan_trajectory(*move_group_robot_, trajectory_plan_robot_) ||
      !execute_trajectory(*move_group_robot_, trajectory_plan_robot_)) {
    RCLCPP_ERROR(logger_, "Failed to move robot to home position");
    return false;
  }

  auto pre_grasp_pose = object_pose;
  pre_grasp_pose.pose.position.z +=
      box_height_ / 2.0 + approach_distance_ + gripper_tcp_offset_;
  pre_grasp_pose.pose.orientation.x = -1.0;
  pre_grasp_pose.pose.orientation.y = 0.0;
  pre_grasp_pose.pose.orientation.z = 0.0;
  pre_grasp_pose.pose.orientation.w = 0.0;

  setup_pose_target(*move_group_robot_, pre_grasp_pose);
  if (!plan_trajectory(*move_group_robot_, trajectory_plan_robot_) ||
      !execute_trajectory(*move_group_robot_, trajectory_plan_robot_)) {
    RCLCPP_ERROR(logger_, "Failed to move to pre-grasp pose");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  setup_named_target(*move_group_gripper_, "gripper_open");
  if (!plan_trajectory(*move_group_gripper_, trajectory_plan_gripper_) ||
      !execute_trajectory(*move_group_gripper_, trajectory_plan_gripper_)) {
    RCLCPP_ERROR(logger_, "Failed to open gripper");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  setup_cartesian_waypoints(0.0, 0.0,
                            -1 * (box_height_ / 2.0 + approach_distance_));
  if (!plan_cartesian_trajectory(*move_group_robot_,
                                 cartesian_trajectory_plan_) ||
      !execute_cartesian_trajectory(*move_group_robot_,
                                    cartesian_trajectory_plan_)) {
    RCLCPP_ERROR(logger_, "Failed to descend towards object");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  setup_named_target(*move_group_gripper_, "gripper_close");
  if (!plan_trajectory(*move_group_gripper_, trajectory_plan_gripper_) ||
      !execute_trajectory(*move_group_gripper_, trajectory_plan_gripper_)) {
    RCLCPP_ERROR(logger_, "Failed to close gripper");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  setup_cartesian_waypoints(0.0, 0.0, box_height_ / 2.0 + approach_distance_);
  if (!plan_cartesian_trajectory(*move_group_robot_,
                                 cartesian_trajectory_plan_) ||
      !execute_cartesian_trajectory(*move_group_robot_,
                                    cartesian_trajectory_plan_)) {
    RCLCPP_ERROR(logger_, "Failed to lift object");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  const std::map<std::string, double> place_joint_positions = {
      {"shoulder_pan_joint", 2.6959}, {"shoulder_lift_joint", -0.6850},
      {"elbow_joint", 0.5442},        {"wrist_2_joint", 1.5713},
      {"wrist_3_joint", 1.1172},      {"wrist_1_joint", 1.7109}};

  setup_joint_target(*move_group_robot_, place_joint_positions);
  if (!plan_trajectory(*move_group_robot_, trajectory_plan_robot_) ||
      !execute_trajectory(*move_group_robot_, trajectory_plan_robot_)) {
    RCLCPP_ERROR(logger_, "Failed to move to place position");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  setup_named_target(*move_group_gripper_, "gripper_open");
  if (!plan_trajectory(*move_group_gripper_, trajectory_plan_gripper_) ||
      !execute_trajectory(*move_group_gripper_, trajectory_plan_gripper_)) {
    RCLCPP_ERROR(logger_, "Failed to release object");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  setup_named_target(*move_group_robot_, "home");
  if (!plan_trajectory(*move_group_robot_, trajectory_plan_robot_) ||
      !execute_trajectory(*move_group_robot_, trajectory_plan_robot_)) {
    RCLCPP_ERROR(logger_, "Failed to return to home position");
    return false;
  }

  RCLCPP_INFO(logger_, "Pick and place sequence completed");
  return true;
}

bool PickAndPlace::plan_trajectory(MoveGroupInterface &move_group, Plan &plan) {
  move_group.setStartStateToCurrentState();
  return move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool PickAndPlace::execute_trajectory(MoveGroupInterface &move_group,
                                      const Plan &plan) {
  return move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

void PickAndPlace::setup_named_target(MoveGroupInterface &move_group,
                                      const std::string &target_name) {
  move_group.setNamedTarget(target_name);
}

void PickAndPlace::setup_joint_target(
    MoveGroupInterface &move_group,
    const std::map<std::string, double> &joint_positions) {
  move_group.setJointValueTarget(joint_positions);
}

void PickAndPlace::setup_pose_target(
    MoveGroupInterface &move_group,
    const geometry_msgs::msg::PoseStamped &target_pose) {
  move_group.setPoseTarget(target_pose);
}

void PickAndPlace::setup_cartesian_waypoints(double dx, double dy, double dz) {
  cartesian_waypoints_.clear();

  auto target_pose = move_group_robot_->getCurrentPose().pose;
  target_pose.position.x += dx;
  target_pose.position.y += dy;
  target_pose.position.z += dz;

  cartesian_waypoints_.push_back(target_pose);
}

bool PickAndPlace::plan_cartesian_trajectory(
    MoveGroupInterface &move_group,
    moveit_msgs::msg::RobotTrajectory &trajectory) {
  const double plan_fraction = move_group.computeCartesianPath(
      cartesian_waypoints_, end_effector_step_, jump_threshold_, trajectory);

  if (plan_fraction < 0.999) {
    RCLCPP_ERROR(logger_,
                 "Cartesian path planning failed. Completed fraction: %.3f",
                 plan_fraction);
    return false;
  }

  return true;
}

bool PickAndPlace::execute_cartesian_trajectory(
    MoveGroupInterface &move_group,
    const moveit_msgs::msg::RobotTrajectory &trajectory) {
  return move_group.execute(trajectory) ==
         moveit::core::MoveItErrorCode::SUCCESS;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  int result = 0;
  try {
    auto base_node = std::make_shared<rclcpp::Node>("pick_and_place");
    PickAndPlace pick_and_place(base_node);

    geometry_msgs::msg::PoseStamped object_pose;
    object_pose.header.frame_id = base_node->declare_parameter<std::string>(
        "object_pose.frame_id", "base_link");
    object_pose.pose.position.x =
        base_node->declare_parameter<double>("object_pose.x", 0.34);
    object_pose.pose.position.y =
        base_node->declare_parameter<double>("object_pose.y", -0.02);
    object_pose.pose.position.z =
        base_node->declare_parameter<double>("object_pose.z", 0.02);
    object_pose.pose.orientation.w = 1.0;

    result = pick_and_place.execute_trajectory_plan(object_pose) ? 0 : 1;
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("pick_and_place"), "%s", error.what());
    result = 1;
  }

  rclcpp::shutdown();
  return result;
}