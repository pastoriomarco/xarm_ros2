/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================
*/

#include "xarm_planner/xarm_planner.h"

namespace xarm_planner
{
const double jump_threshold = 0.0;
const double eef_step = 0.005;
// Removed the constants to make them runtime modifiable
// const double max_velocity_scaling_factor = 0.3;  // [move_group_interface] default is 0.1
// const double max_acceleration_scaling_factor = 0.1;  // [move_group_interface] default is 0.1

XArmPlanner::XArmPlanner(const rclcpp::Node::SharedPtr& node, const std::string& group_name)
    : node_(node),
      max_velocity_scaling_factor_(0.3),
      max_acceleration_scaling_factor_(0.1)
{
    init(group_name);
}

XArmPlanner::XArmPlanner(const std::string& group_name)
    : max_velocity_scaling_factor_(0.3),
      max_acceleration_scaling_factor_(0.1)
{
    node_ = rclcpp::Node::make_shared("xarm_planner_move_group_node");
    init(group_name);
}

void XArmPlanner::init(const std::string& group_name) 
{
    is_trajectory_ = false;
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
    RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
    RCLCPP_INFO(node_->get_logger(), "Available Planning Groups:");
    std::copy(move_group_->getJointModelGroupNames().begin(), move_group_->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);
}

bool XArmPlanner::planJointTarget(const std::vector<double>& joint_target)
{
    bool success = move_group_->setJointValueTarget(joint_target);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setJointValueTarget: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planJointTarget: plan failed");
    is_trajectory_ = false;
    return success;
}

bool XArmPlanner::planPoseTarget(const geometry_msgs::msg::Pose& pose_target)
{
    bool success = move_group_->setPoseTarget(pose_target);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setPoseTarget: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planPoseTarget: plan failed");
    is_trajectory_ = false;
    return success;
}

bool XArmPlanner::planPoseTargets(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector)
{
    bool success = move_group_->setPoseTargets(pose_target_vector);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setPoseTargets: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planPoseTargets: plan failed");
    is_trajectory_ = false;
    return success;
}

bool XArmPlanner::planCartesianPath(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector)
{   
    // moveit_msgs::msg::RobotTrajectory trajectory;
    
    double fraction = move_group_->computeCartesianPath(pose_target_vector, eef_step, jump_threshold, trajectory_);
    bool success = true;
    if(fraction < 0.9) {
        RCLCPP_ERROR(node_->get_logger(), "planCartesianPath: plan failed, fraction=%lf", fraction);
        return false;
    }
    is_trajectory_ = true;
    // https://github.com/ros-planning/moveit2/commit/8bfe782d6254997d185644fa3eb358d2b79d69b2
    // (struct Plan) trajectory_ => trajectory
    // xarm_plan_.trajectory_ = trajectory;
    return true;
}

bool XArmPlanner::executePath(bool wait)
{
    moveit::core::MoveItErrorCode code;
    if (wait)
        code = is_trajectory_ ? move_group_->execute(trajectory_) : move_group_->execute(xarm_plan_);
    else
        code =  is_trajectory_ ? move_group_->asyncExecute(trajectory_) : move_group_->asyncExecute(xarm_plan_);
    bool success = (code == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "executePath: execute failed, wait=%d, MoveItErrorCode=%d", wait, code.val);
    return success;
}

// New setter method to update scaling factors
bool XArmPlanner::setScalingFactors(const std::vector<float>& datas)
{
    if (datas.size() != 2) {
        RCLCPP_WARN(node_->get_logger(), "setScalingFactors: expected 2 elements, received %zu", datas.size());
        return false;
    }
    double new_velocity = static_cast<double>(datas[0]);
    double new_acceleration = static_cast<double>(datas[1]);

    // Validate the scaling factors
    if (new_velocity <= 0.0 || new_velocity > 1.0) {
        RCLCPP_WARN(node_->get_logger(), "setScalingFactors: invalid velocity scaling factor: %lf", new_velocity);
        return false;
    }
    if (new_acceleration <= 0.0 || new_acceleration > 1.0) {
        RCLCPP_WARN(node_->get_logger(), "setScalingFactors: invalid acceleration scaling factor: %lf", new_acceleration);
        return false;
    }

    // Update the member variables
    max_velocity_scaling_factor_ = new_velocity;
    max_acceleration_scaling_factor_ = new_acceleration;

    // Apply the new scaling factors to the move group
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);

    RCLCPP_INFO(node_->get_logger(), "setScalingFactors: velocity=%.3lf, acceleration=%.3lf",
                max_velocity_scaling_factor_, max_acceleration_scaling_factor_);
    return true;
}

}
