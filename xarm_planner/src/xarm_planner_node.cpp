/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================
*/

#include <signal.h>
#include <algorithm> // Include for std::clamp
#include <rclcpp/rclcpp.hpp>
#include "xarm_planner/xarm_planner.h"
#include <std_msgs/msg/bool.hpp>
#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <xarm_msgs/srv/plan_single_straight.hpp>
#include <xarm_msgs/srv/set_float32_list.hpp>  // Include the SetFloat32List service for scaling factors

#define BIND_CLS_CB(func) std::bind(func, this, std::placeholders::_1, std::placeholders::_2)

class XArmPlannerRunner
{
public:
    XArmPlannerRunner(rclcpp::Node::SharedPtr& node);
    ~XArmPlannerRunner() {};

private:
    bool do_pose_plan(const std::shared_ptr<xarm_msgs::srv::PlanPose::Request> req, std::shared_ptr<xarm_msgs::srv::PlanPose::Response> res);
    bool do_joint_plan(const std::shared_ptr<xarm_msgs::srv::PlanJoint::Request> req, std::shared_ptr<xarm_msgs::srv::PlanJoint::Response> res);
    bool do_single_cartesian_plan(const std::shared_ptr<xarm_msgs::srv::PlanSingleStraight::Request> req, std::shared_ptr<xarm_msgs::srv::PlanSingleStraight::Response> res);
    bool exec_plan_cb(const std::shared_ptr<xarm_msgs::srv::PlanExec::Request> req, std::shared_ptr<xarm_msgs::srv::PlanExec::Response> res);
    
    // New service callback for setting scaling factors
    bool set_scaling_factors_cb(const std::shared_ptr<xarm_msgs::srv::SetFloat32List::Request> req,
                                std::shared_ptr<xarm_msgs::srv::SetFloat32List::Response> res);

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<xarm_planner::XArmPlanner> xarm_planner_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exec_plan_sub_;

    rclcpp::Service<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_server_;
    rclcpp::Service<xarm_msgs::srv::PlanPose>::SharedPtr pose_plan_server_;
    rclcpp::Service<xarm_msgs::srv::PlanJoint>::SharedPtr joint_plan_server_;
    rclcpp::Service<xarm_msgs::srv::PlanSingleStraight>::SharedPtr single_straight_plan_server_;
    
    // New service server for setting scaling factors
    rclcpp::Service<xarm_msgs::srv::SetFloat32List>::SharedPtr set_scaling_factors_server_;
};

XArmPlannerRunner::XArmPlannerRunner(rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    int dof;
    node_->get_parameter_or("dof", dof, 7);
    std::string robot_type;
    node_->get_parameter_or("robot_type", robot_type, std::string("xarm"));
    std::string group_name = robot_type;
    if (robot_type == "xarm" || robot_type == "lite")
        group_name = robot_type + std::to_string(dof);
    std::string prefix;
    node->get_parameter_or("prefix", prefix, std::string(""));
    if (prefix != "") {
        group_name = prefix + group_name;
    }

    RCLCPP_INFO(node_->get_logger(), "namespace: %s, group_name: %s", node->get_namespace(), group_name.c_str());

    xarm_planner_ = std::make_shared<xarm_planner::XArmPlanner>(group_name);

    exec_plan_server_ = node_->create_service<xarm_msgs::srv::PlanExec>("xarm_exec_plan", BIND_CLS_CB(&XArmPlannerRunner::exec_plan_cb));
    pose_plan_server_ = node_->create_service<xarm_msgs::srv::PlanPose>("xarm_pose_plan", BIND_CLS_CB(&XArmPlannerRunner::do_pose_plan));
    joint_plan_server_ = node_->create_service<xarm_msgs::srv::PlanJoint>("xarm_joint_plan", BIND_CLS_CB(&XArmPlannerRunner::do_joint_plan));
    single_straight_plan_server_ = node_->create_service<xarm_msgs::srv::PlanSingleStraight>("xarm_straight_plan", BIND_CLS_CB(&XArmPlannerRunner::do_single_cartesian_plan));
    
    // Initialize the new service server
    set_scaling_factors_server_ = node_->create_service<xarm_msgs::srv::SetFloat32List>(
        "xarm_set_scaling_factors",
        std::bind(&XArmPlannerRunner::set_scaling_factors_cb, this, std::placeholders::_1, std::placeholders::_2));
}

bool XArmPlannerRunner::do_pose_plan(const std::shared_ptr<xarm_msgs::srv::PlanPose::Request> req, std::shared_ptr<xarm_msgs::srv::PlanPose::Response> res)
{
    bool success = xarm_planner_->planPoseTarget(req->target);
    res->success = success;
    return success;
}

bool XArmPlannerRunner::do_joint_plan(const std::shared_ptr<xarm_msgs::srv::PlanJoint::Request> req, std::shared_ptr<xarm_msgs::srv::PlanJoint::Response> res)
{
    bool success = xarm_planner_->planJointTarget(req->target);
    res->success = success;
    return success;
}

bool XArmPlannerRunner::do_single_cartesian_plan(const std::shared_ptr<xarm_msgs::srv::PlanSingleStraight::Request> req, std::shared_ptr<xarm_msgs::srv::PlanSingleStraight::Response> res)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(req->target);
    bool success = xarm_planner_->planCartesianPath(waypoints);
    res->success = success;
    return success;
}

bool XArmPlannerRunner::exec_plan_cb(const std::shared_ptr<xarm_msgs::srv::PlanExec::Request> req, std::shared_ptr<xarm_msgs::srv::PlanExec::Response> res)
{
    bool success = xarm_planner_->executePath(req->wait);
    res->success = success;
    return success;
}

bool XArmPlannerRunner::set_scaling_factors_cb(const std::shared_ptr<xarm_msgs::srv::SetFloat32List::Request> req,
                                              std::shared_ptr<xarm_msgs::srv::SetFloat32List::Response> res)
{
    if (req->datas.size() != 2) {
        res->ret = -1;
        res->message = "Expected exactly 2 float32 values: [max_velocity_scaling_factor, max_acceleration_scaling_factor]";
        RCLCPP_WARN(node_->get_logger(), "set_scaling_factors_cb: %s", res->message.c_str());
        return true;
    }

    // Clone the input data to modify
    std::vector<float> clamped_datas = req->datas;

    // Clamp each value to the range [0.0, 1.0]
    clamped_datas[0] = std::clamp(clamped_datas[0], 0.0f, 1.0f);
    clamped_datas[1] = std::clamp(clamped_datas[1], 0.0f, 1.0f);

    // Inform the user if any values were clamped
    if (clamped_datas != req->datas) {
        res->ret = -1;
        res->message = "Input values were out of range and have been clamped to [0.0, 1.0].";
        RCLCPP_WARN(node_->get_logger(), "set_scaling_factors_cb: %s", res->message.c_str());
        // Proceed to set the clamped values
    }

    // Attempt to set the scaling factors with clamped values
    bool success = xarm_planner_->setScalingFactors(clamped_datas);
    if (success) {
        if (clamped_datas != req->datas) {
            res->ret = 0;
            res->message = "Scaling factors clamped and updated successfully.";
        } else {
            res->ret = 0;
            res->message = "Scaling factors updated successfully.";
        }
    } else {
        res->ret = -1;
        res->message = "Failed to update scaling factors.";
        RCLCPP_ERROR(node_->get_logger(), "set_scaling_factors_cb: %s", res->message.c_str());
    }
    return true;
}

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_planner_node] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_planner_node", node_options);
    RCLCPP_INFO(node->get_logger(), "xarm_planner_node start");
    signal(SIGINT, exit_sig_handler);

    XArmPlannerRunner xarm_planner_runner(node);
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    RCLCPP_INFO(node->get_logger(), "xarm_planner_node over");
    return 0;
}
