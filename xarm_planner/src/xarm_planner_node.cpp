// src/xarm_ros2/xarm_planner/src/xarm_planner_node.cpp

#include <signal.h>
#include <algorithm> // for std::clamp
#include <rclcpp/rclcpp.hpp>
#include "xarm_planner/xarm_planner.h"
#include <std_msgs/msg/bool.hpp>
#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <xarm_msgs/srv/plan_single_straight.hpp>
#include <xarm_msgs/srv/set_float32_list.hpp>
#include <xarm_msgs/srv/call.hpp>

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
    
    // Service callback for setting scaling factors
    bool set_scaling_factors_cb(const std::shared_ptr<xarm_msgs::srv::SetFloat32List::Request> req,
                                std::shared_ptr<xarm_msgs::srv::SetFloat32List::Response> res);
    
    // Service callbacks for base and eef links
    bool get_base_link_cb(const std::shared_ptr<xarm_msgs::srv::Call::Request> req,
                         std::shared_ptr<xarm_msgs::srv::Call::Response> res);
                         
    bool get_eef_link_cb(const std::shared_ptr<xarm_msgs::srv::Call::Request> req,
                        std::shared_ptr<xarm_msgs::srv::Call::Response> res);

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<xarm_planner::XArmPlanner> xarm_planner_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exec_plan_sub_;

    rclcpp::Service<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_server_;
    rclcpp::Service<xarm_msgs::srv::PlanPose>::SharedPtr pose_plan_server_;
    rclcpp::Service<xarm_msgs::srv::PlanJoint>::SharedPtr joint_plan_server_;
    rclcpp::Service<xarm_msgs::srv::PlanSingleStraight>::SharedPtr single_straight_plan_server_;
    
    // Service server for setting scaling factors
    rclcpp::Service<xarm_msgs::srv::SetFloat32List>::SharedPtr set_scaling_factors_server_;
    
    // Service servers for base and eef links
    rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr get_base_link_server_;
    rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr get_eef_link_server_;
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
    
    // Initialize the service server for setting scaling factors
    set_scaling_factors_server_ = node_->create_service<xarm_msgs::srv::SetFloat32List>(
        "xarm_set_scaling_factors",
        std::bind(&XArmPlannerRunner::set_scaling_factors_cb, this, std::placeholders::_1, std::placeholders::_2));
    
    // Initialize the service servers for base and eef links
    get_base_link_server_ = node_->create_service<xarm_msgs::srv::Call>(
        "get_base_link",
        std::bind(&XArmPlannerRunner::get_base_link_cb, this, std::placeholders::_1, std::placeholders::_2));
        
    get_eef_link_server_ = node_->create_service<xarm_msgs::srv::Call>(
        "get_eef_link",
        std::bind(&XArmPlannerRunner::get_eef_link_cb, this, std::placeholders::_1, std::placeholders::_2));
}

bool XArmPlannerRunner::get_base_link_cb(const std::shared_ptr<xarm_msgs::srv::Call::Request> req,
                                         std::shared_ptr<xarm_msgs::srv::Call::Response> res)
{
    (void)req; // Unused
    std::string base_link = xarm_planner_->getMoveGroup()->getPlanningFrame();
    res->ret = 0;
    res->message = base_link;
    RCLCPP_INFO(node_->get_logger(), "get_base_link: %s", base_link.c_str());
    return true;
}

bool XArmPlannerRunner::get_eef_link_cb(const std::shared_ptr<xarm_msgs::srv::Call::Request> req,
                                        std::shared_ptr<xarm_msgs::srv::Call::Response> res)
{
    (void)req; // Unused
    std::string eef_link = xarm_planner_->getMoveGroup()->getEndEffectorLink();
    res->ret = 0;
    res->message = eef_link;
    RCLCPP_INFO(node_->get_logger(), "get_eef_link: %s", eef_link.c_str());
    return true;
}

// Existing methods...

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
