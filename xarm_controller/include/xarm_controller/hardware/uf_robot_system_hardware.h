/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#ifndef __UF_ROBOT_SYSTEM_HARDWARE_INTERFACE_H__
#define __UF_ROBOT_SYSTEM_HARDWARE_INTERFACE_H__

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
// #include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
// #include "hardware_interface/visibility_control.h"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/msg/controller_manager_activity.hpp"
#include "xarm_api/supervised_driver.h"
#include "xarm_api/xarm_driver.h"
#include "xarm_controller/hardware/supervised_lifecycle_contract.h"
#include "xarm_msgs/msg/supervised_lifecycle_state.hpp"
#include "xarm_msgs/srv/execute_supervised_lifecycle_command.hpp"
#include "xarm_msgs/srv/set_supervised_command_gate.hpp"
#include "xarm_msgs/srv/shutdown_supervised_controller.hpp"


namespace uf_robot_hardware
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class UFRobotSystemHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(UFRobotSystemHardware)

        ~UFRobotSystemHardware() override;
        CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) final;
        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) final;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) final;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) final;
        CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) final;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration &period) final;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration &period) final;

        // hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
        //                                                             const std::vector<std::string>& stop_interfaces) final;

        // hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
        //                                                             const std::vector<std::string>& stop_interfaces) final;

    protected:
        hardware_interface::HardwareInfo info_;
    
    private:
        int read_code_;
        int write_code_;

        std::string robot_ip_;

        float prev_cmds_float_[7];
		float cmds_float_[7];
        std::vector<double> position_cmds_;
        std::vector<double> velocity_cmds_;
        std::vector<double> position_states_;
        std::vector<double> velocity_states_;

        bool velocity_control_;
        bool initialized_;
        bool read_ready_;
        bool reactivate_controller_later_;

        long int read_cnts_;
        long int read_failed_cnts_;
        double read_max_time_;
        double read_total_time_;
        
        // float prev_read_position_[7];
		// float curr_read_position_[7];
		// float curr_read_velocity_[7];
		// float curr_read_effort_[7];
        
        // rclcpp::Time prev_read_time_;
        // rclcpp::Time curr_read_time_;
        rclcpp::Time curr_write_time_;
        rclcpp::Time prev_write_time_;

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<rclcpp::Node> hw_node_;
        xarm_api::XArmDriver xarm_driver_;
        sensor_msgs::msg::JointState *joint_state_msg_;

        std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request> req_list_controller_;
	    std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> res_list_controller_;
        std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> req_switch_controller_;
        std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> res_switch_controller_;

        rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr client_list_controller_;
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr client_switch_controller_;

        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr update_goal_state_pub_;
        std_msgs::msg::Empty update_goal_state_msg_;

        bool _check_cmds_is_change(float *prev, float *cur, double threshold = 0.0001);
        bool _xarm_is_ready_read(void);
        bool _xarm_is_ready_write(void);

        bool _need_reset(void);

        void _deactivate_controller(void);
        void _activate_controller(void);

        bool _init_ufactory_driver(void);
        bool _configure_supervised_driver(void);
        void _release_supervised_driver(void) noexcept;
        void _init_supervised_ros_boundary(void);
        void _stop_supervised_ros_boundary(void) noexcept;
        void _publish_supervised_state(void);
        void _execute_supervised_command(
            const std::shared_ptr<xarm_msgs::srv::ExecuteSupervisedLifecycleCommand::Request> request,
            std::shared_ptr<xarm_msgs::srv::ExecuteSupervisedLifecycleCommand::Response> response);
        void _set_supervised_command_gate(
            const std::shared_ptr<xarm_msgs::srv::SetSupervisedCommandGate::Request> request,
            std::shared_ptr<xarm_msgs::srv::SetSupervisedCommandGate::Response> response);
        void _shutdown_supervised_controller(
            const std::shared_ptr<xarm_msgs::srv::ShutdownSupervisedController::Request> request,
            std::shared_ptr<xarm_msgs::srv::ShutdownSupervisedController::Response> response);
        static std::int64_t _steady_now_ns(void);
        static std::string _new_supervised_session_id(const std::string& owner_id);

        template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr, typename SharedResponse = typename ServiceT::Response::SharedPtr>
        int _call_request(std::shared_ptr<ServiceT> client, SharedRequest req, SharedResponse& res);

        bool supervised_lifecycle_ = false;
        std::string supervised_owner_id_;
        std::string supervised_session_id_;
        std::string supervised_report_type_ = "rich";
        int supervised_expected_device_type_ = -1;
        std::int64_t supervised_observation_lease_ns_ = 250000000;
        std::int64_t supervised_joint_state_lease_ns_ = 100000000;
        std::int64_t supervised_io_period_ns_ = 5000000;
        std::int64_t supervised_transport_loss_timeout_ns_ = 2000000000;
        std::int64_t supervised_max_gate_lease_ns_ = 250000000;
        std::int64_t supervised_shutdown_stationary_dwell_ns_ = 1000000000;
        double supervised_source_agreement_tolerance_rad_ = 0.002;
        double supervised_shutdown_stationary_tolerance_rad_ = 0.001;
        std::size_t supervised_position_initialization_samples_ = 3;
        std::string supervised_controller_activity_topic_;
        std::string supervised_trajectory_controller_name_;
        std::unique_ptr<xarm_api::SupervisedDriver> supervised_driver_;
        std::atomic<xarm_api::SupervisedDriver*> supervised_rt_driver_{nullptr};
        std::mutex supervised_nrt_mutex_;
        std::atomic<bool> supervised_hardware_active_{false};
        SupervisedCommandReplayCache supervised_replay_{64};
        SupervisedCommandReplayCache supervised_shutdown_replay_{16};
        bool supervised_controller_activity_known_ = false;
        bool supervised_trajectory_controller_active_ = false;
        rclcpp::executors::SingleThreadedExecutor::SharedPtr supervised_executor_;
        std::thread supervised_executor_thread_;
        rclcpp::Publisher<
            xarm_msgs::msg::SupervisedLifecycleState>::SharedPtr
            supervised_state_publisher_;
        rclcpp::Service<
            xarm_msgs::srv::ExecuteSupervisedLifecycleCommand>::SharedPtr
            supervised_command_service_;
        rclcpp::Service<
            xarm_msgs::srv::SetSupervisedCommandGate>::SharedPtr
            supervised_gate_service_;
        rclcpp::Service<
            xarm_msgs::srv::ShutdownSupervisedController>::SharedPtr
            supervised_shutdown_service_;
        rclcpp::Subscription<
            controller_manager_msgs::msg::ControllerManagerActivity>::SharedPtr
            supervised_controller_activity_subscription_;
        rclcpp::TimerBase::SharedPtr supervised_state_timer_;
    };
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(uf_robot_hardware::UFRobotSystemHardware, hardware_interface::SystemInterface)

#endif // __UF_ROBOT_SYSTEM_HARDWARE_INTERFACE_H__
