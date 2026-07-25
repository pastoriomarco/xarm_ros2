/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include "xarm_controller/hardware/uf_robot_system_hardware.h"

#include <arpa/inet.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <utility>

#define SERVICE_CALL_FAILED 999
#define SERVICE_IS_PERSISTENT_BUT_INVALID 998
#define ROBOT_IS_DISCONNECTED -1
#define WAIT_SERVICE_TIMEOUT 996
#define VELO_DURATION 1

namespace uf_robot_hardware
{
    static rclcpp::Logger LOGGER = rclcpp::get_logger("UFACTORY.RobotHW");

    namespace
    {
        constexpr std::size_t LITE6_JOINT_COUNT = 6;
        constexpr int LITE6_DEVICE_TYPE = 9;
        constexpr std::int64_t NANOS_PER_MILLISECOND = 1000000;
        constexpr std::int64_t MIN_GATE_LEASE_NS = 1000000;
        std::atomic<std::uint64_t> SESSION_COUNTER{0};

        std::string decode_xacro_string(const std::string & value)
        {
            if (!value.empty() && value.front() == 'R') {
                return value.substr(1);
            }
            return value;
        }

        bool valid_unicast_ipv4(const std::string & value)
        {
            in_addr address{};
            if (inet_pton(AF_INET, value.c_str(), &address) != 1) {
                return false;
            }
            const std::uint32_t host_address = ntohl(address.s_addr);
            const std::uint32_t first_octet = host_address >> 24U;
            return host_address != 0U &&
                   host_address != std::numeric_limits<std::uint32_t>::max() &&
                   first_octet != 0U &&
                   first_octet != 127U &&
                   first_octet < 224U;
        }

        bool parse_positive_int64(
            const std::string & value,
            std::int64_t & result)
        {
            try {
                std::size_t consumed = 0;
                const long long parsed = std::stoll(value, &consumed);
                if (consumed != value.size() || parsed <= 0) {
                    return false;
                }
                result = static_cast<std::int64_t>(parsed);
                return true;
            }
            catch (...) {
                return false;
            }
        }

        bool parse_positive_size(
            const std::string & value,
            std::size_t & result)
        {
            std::int64_t parsed = 0;
            if (!parse_positive_int64(value, parsed)) {
                return false;
            }
            result = static_cast<std::size_t>(parsed);
            return true;
        }

        bool parse_positive_double(
            const std::string & value,
            double & result)
        {
            try {
                std::size_t consumed = 0;
                const double parsed = std::stod(value, &consumed);
                if (consumed != value.size() ||
                    !std::isfinite(parsed) || parsed <= 0.0)
                {
                    return false;
                }
                result = parsed;
                return true;
            }
            catch (...) {
                return false;
            }
        }
    }  // namespace

    template<typename ServiceT, typename SharedRequest, typename SharedResponse>
    int UFRobotSystemHardware::_call_request(std::shared_ptr<ServiceT> client, SharedRequest req, SharedResponse& res)
    {
        bool is_try_again = false;
        int failed_cnts = 0;
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(LOGGER, "[%s] Interrupted while waiting for the service. Exiting.", robot_ip_.c_str());
                exit(1);
            }
            if (!is_try_again) {
                is_try_again = true;
                RCLCPP_WARN(LOGGER, "[%s] service %s not available, waiting ...", robot_ip_.c_str(), client->get_service_name());
            }
            failed_cnts += 1;
            if (failed_cnts >= 5) return WAIT_SERVICE_TIMEOUT;
        }
        auto result_future = client->async_send_request(req);
        if (rclcpp::spin_until_future_complete(hw_node_, result_future, std::chrono::seconds(1)) != rclcpp::FutureReturnCode::SUCCESS)
        {
            // RCLCPP_ERROR(LOGGER, "[%s] Failed to call service %s", robot_ip_.c_str(), client->get_service_name());
            return SERVICE_CALL_FAILED;
        }
        res = result_future.get();
        return 0;
    }

    UFRobotSystemHardware::~UFRobotSystemHardware()
    {
        _release_supervised_driver();
        _stop_supervised_ros_boundary();
    }

    bool UFRobotSystemHardware::_init_ufactory_driver(void)
    {
        rclcpp::NodeOptions node_options;
        node_options.allow_undeclared_parameters(true);
        node_options.automatically_declare_parameters_from_overrides(true);
        node_ = rclcpp::Node::make_shared("ufactory_driver", node_options);
        hw_node_ = rclcpp::Node::make_shared("ufactory_robot_hw", node_options);

        update_goal_state_pub_ = hw_node_->create_publisher<std_msgs::msg::Empty>("/rviz/moveit/update_goal_state", 1);

        robot_ip_ = "";
        auto it = info_.hardware_parameters.find("robot_ip");
        if (it != info_.hardware_parameters.end()) {
            robot_ip_ = decode_xacro_string(it->second);
        }
        if (robot_ip_ == "") {
            RCLCPP_ERROR(LOGGER, "[%s] No param named 'robot_ip'", robot_ip_.c_str());
            return false;
        }

        std::string hw_ns = "xarm";
        it = info_.hardware_parameters.find("hw_ns");
        if (it != info_.hardware_parameters.end()) {
            hw_ns = it->second;
        }
        node_->set_parameter(rclcpp::Parameter("hw_ns", hw_ns));

        std::string prefix = "";
        it = info_.hardware_parameters.find("prefix");
        if (it != info_.hardware_parameters.end()) {
            prefix = decode_xacro_string(it->second);
        }
        if (prefix != "") {
            LOGGER = rclcpp::get_logger("UFACTORY." + prefix + "RobotHW");
        }
        node_->set_parameter(rclcpp::Parameter("prefix", prefix));

        std::string report_type = "normal";
        it = info_.hardware_parameters.find("report_type");
        if (it != info_.hardware_parameters.end()) {
            report_type = it->second;
        }
        node_->set_parameter(rclcpp::Parameter("report_type", report_type));

        std::string access_mode = "legacy_control";
        it = info_.hardware_parameters.find("access_mode");
        if (it != info_.hardware_parameters.end()) {
            access_mode = it->second;
        }
        supervised_lifecycle_ = access_mode == "supervised_lifecycle";
        node_->set_parameter(rclcpp::Parameter("access_mode", access_mode));

        std::string robot_type = "xarm";
        it = info_.hardware_parameters.find("robot_type");
        if (it != info_.hardware_parameters.end()) {
            robot_type = it->second;
        }

        RCLCPP_INFO(LOGGER, "[%s] namespace: %s", robot_ip_.c_str(), node_->get_namespace());
        RCLCPP_INFO(LOGGER, "[%s] robot_type: %s, hw_ns: %s, prefix: %s, report_type: %s", 
            robot_ip_.c_str(), robot_type.c_str(), hw_ns.c_str(), prefix.c_str(), report_type.c_str());

        int dof = 7;
        it = info_.hardware_parameters.find("dof");
        if (it != info_.hardware_parameters.end()) {
            dof = atoi(it->second.c_str());
        }
        node_->set_parameter(rclcpp::Parameter("dof", dof));

        int expected_robot_device_type = -1;
        it = info_.hardware_parameters.find("expected_robot_device_type");
        if (it != info_.hardware_parameters.end()) {
            expected_robot_device_type = atoi(it->second.c_str());
        }
        node_->set_parameter(
            rclcpp::Parameter(
                "expected_robot_device_type",
                expected_robot_device_type));

        int default_gripper_baud = 2000000;
        it = info_.hardware_parameters.find("default_gripper_baud");
        if (it != info_.hardware_parameters.end()) {
            default_gripper_baud = atoi(it->second.c_str());
        }
        node_->set_parameter(rclcpp::Parameter("default_gripper_baud", default_gripper_baud));

        bool baud_checkset = true;
        it = info_.hardware_parameters.find("baud_checkset");
        if (it != info_.hardware_parameters.end()) {
            baud_checkset = (it->second == "True" || it->second == "true");
        }
        node_->set_parameter(rclcpp::Parameter("baud_checkset", baud_checkset));

        bool add_gripper = true;
        it = info_.hardware_parameters.find("add_gripper");
        if (it != info_.hardware_parameters.end()) {
            add_gripper = (it->second == "True" || it->second == "true");
        }
        
        if (robot_type == "lite") add_gripper = false;
        node_->set_parameter(rclcpp::Parameter("add_gripper", add_gripper));

        bool add_bio_gripper = true;
        it = info_.hardware_parameters.find("add_bio_gripper");
        if (it != info_.hardware_parameters.end()) {
            add_bio_gripper = (it->second == "True" || it->second == "true");
        }
        
        if (robot_type == "lite") add_bio_gripper = false;
        node_->set_parameter(rclcpp::Parameter("add_bio_gripper", add_bio_gripper));

        it = info_.hardware_parameters.find("velocity_control");
        if (it != info_.hardware_parameters.end()) {
            velocity_control_ = (it->second == "True" || it->second == "true");
        }
        RCLCPP_INFO(LOGGER, "[%s] dof: %d, velocity_control: %d, add_gripper: %d, add_bio_gripper: %d, baud_checkset: %d, default_gripper_baud: %d", 
            robot_ip_.c_str(), dof, velocity_control_, add_gripper, add_bio_gripper, baud_checkset, default_gripper_baud);

        if (supervised_lifecycle_) {
            if (!valid_unicast_ipv4(robot_ip_) ||
                report_type != "rich" ||
                robot_type != "lite" ||
                dof != static_cast<int>(LITE6_JOINT_COUNT) ||
                expected_robot_device_type != LITE6_DEVICE_TYPE ||
                velocity_control_ || add_gripper || add_bio_gripper)
            {
                RCLCPP_ERROR(
                    LOGGER,
                    "Supervised lifecycle requires an exact command-suppressed "
                    "Lite6 binding");
                return false;
            }

            it = info_.hardware_parameters.find("transport_owner_id");
            if (it == info_.hardware_parameters.end() ||
                it->second.empty() ||
                it->second.find("__MANYFORGE_") != std::string::npos)
            {
                RCLCPP_ERROR(
                    LOGGER,
                    "Supervised lifecycle requires a resolved transport owner");
                return false;
            }
            supervised_owner_id_ = it->second;
            supervised_expected_device_type_ = expected_robot_device_type;
            supervised_report_type_ = report_type;

            const auto parse_milliseconds =
                [this](const char * name, std::int64_t & target) {
                    const auto found = info_.hardware_parameters.find(name);
                    if (found == info_.hardware_parameters.end()) {
                        return true;
                    }
                    std::int64_t milliseconds = 0;
                    if (!parse_positive_int64(found->second, milliseconds) ||
                        milliseconds >
                        std::numeric_limits<std::int64_t>::max() /
                        NANOS_PER_MILLISECOND)
                    {
                        return false;
                    }
                    target = milliseconds * NANOS_PER_MILLISECOND;
                    return true;
                };
            if (!parse_milliseconds(
                    "observation_lease_ms",
                    supervised_observation_lease_ns_) ||
                !parse_milliseconds(
                    "joint_state_lease_ms",
                    supervised_joint_state_lease_ns_) ||
                !parse_milliseconds(
                    "io_period_ms",
                    supervised_io_period_ns_) ||
                !parse_milliseconds(
                    "command_gate_max_lease_ms",
                    supervised_max_gate_lease_ns_))
            {
                RCLCPP_ERROR(
                    LOGGER,
                    "Supervised lifecycle timing parameters are invalid");
                return false;
            }
            it = info_.hardware_parameters.find(
                "source_agreement_tolerance_rad");
            if (it != info_.hardware_parameters.end() &&
                !parse_positive_double(
                    it->second,
                    supervised_source_agreement_tolerance_rad_))
            {
                RCLCPP_ERROR(
                    LOGGER,
                    "Supervised source agreement tolerance is invalid");
                return false;
            }
            it = info_.hardware_parameters.find(
                "position_initialization_samples");
            if (it != info_.hardware_parameters.end() &&
                !parse_positive_size(
                    it->second,
                    supervised_position_initialization_samples_))
            {
                RCLCPP_ERROR(
                    LOGGER,
                    "Supervised position initialization count is invalid");
                return false;
            }
            _init_supervised_ros_boundary();
            return true;
        }

        std::thread th([this]() -> void {
            rclcpp::spin(node_);
            rclcpp::shutdown();
        });
        th.detach();

        // 20250318, disable xarm_driver publish joint_states
        xarm_driver_.init(node_, robot_ip_, true);
        // 20250318, get joint_states msg reference from xarm_driver
        joint_state_msg_ = xarm_driver_.get_joint_states();
        return true;
    }

    CallbackReturn UFRobotSystemHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
    {
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        info_ = params.hardware_info;
        velocity_control_ = false;
        read_code_ = 0;
        write_code_ = 0;

        initialized_ = false;
        reactivate_controller_later_ = false;

        read_cnts_ = 0;
        read_max_time_ = 0;
        read_total_time_ = 0;
        read_failed_cnts_ = 0;
        memset(cmds_float_, 0, sizeof(cmds_float_));
        memset(prev_cmds_float_, 0, sizeof(prev_cmds_float_));

        if (!_init_ufactory_driver()) {
            _stop_supervised_ros_boundary();
            return CallbackReturn::ERROR;
        }
        
        position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        position_cmds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        velocity_cmds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());


        for (const hardware_interface::ComponentInfo & joint : info_.joints) {
            bool has_pos_cmd_interface = false;
            for (auto i = 0u; i < joint.command_interfaces.size(); ++i) {
                if (joint.command_interfaces[i].name == hardware_interface::HW_IF_POSITION) {
                    has_pos_cmd_interface = true;
                    break;
                }
            }
            if (!has_pos_cmd_interface) {
                RCLCPP_ERROR(LOGGER, "[%s] Joint '%s' has %ld command interfaces found, but not found %s command interface",
                    robot_ip_.c_str(), joint.name.c_str(), joint.command_interfaces.size(), hardware_interface::HW_IF_POSITION
                );
                return CallbackReturn::ERROR;
            }

            bool has_pos_state_interface = false;
            for (auto i = 0u; i < joint.state_interfaces.size(); ++i) {
                if (joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION) {
                    has_pos_state_interface = true;
                    break;
                }
            }
            if (!has_pos_state_interface) {
                RCLCPP_ERROR(LOGGER, "[%s] Joint '%s' has %ld state interfaces found, but not found %s state interface",
                    robot_ip_.c_str(), joint.name.c_str(), joint.state_interfaces.size(), hardware_interface::HW_IF_POSITION
                );
                return CallbackReturn::ERROR;
            }
        }

        RCLCPP_INFO(LOGGER, "[%s] System Sucessfully configured!", robot_ip_.c_str());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn UFRobotSystemHardware::on_configure(
        const rclcpp_lifecycle::State&)
    {
        if (!supervised_lifecycle_) {
            return CallbackReturn::SUCCESS;
        }
        return _configure_supervised_driver() ?
               CallbackReturn::SUCCESS : CallbackReturn::ERROR;
    }

    CallbackReturn UFRobotSystemHardware::on_cleanup(
        const rclcpp_lifecycle::State&)
    {
        if (supervised_lifecycle_) {
            _release_supervised_driver();
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn UFRobotSystemHardware::on_shutdown(
        const rclcpp_lifecycle::State&)
    {
        if (supervised_lifecycle_) {
            _release_supervised_driver();
            _stop_supervised_ros_boundary();
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn UFRobotSystemHardware::on_error(
        const rclcpp_lifecycle::State&)
    {
        if (supervised_lifecycle_) {
            _release_supervised_driver();
        }
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> UFRobotSystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> UFRobotSystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_cmds_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_cmds_[i]));
        }

        return command_interfaces;
    }

    CallbackReturn UFRobotSystemHardware::on_activate(const rclcpp_lifecycle::State& previous_state)
    {
        if (supervised_lifecycle_) {
            std::lock_guard<std::mutex> lock(supervised_nrt_mutex_);
            if (supervised_driver_ == nullptr) {
                return CallbackReturn::ERROR;
            }
            const std::int64_t now_ns = _steady_now_ns();
            const auto observation = supervised_driver_->observe(now_ns);
            xarm_api::SupervisedDriverJointState joint_state;
            const int expected_mask =
                (1 << static_cast<int>(info_.joints.size())) - 1;
            const bool ready =
                observation.connected &&
                observation.report_connected &&
                observation.identity_verified &&
                !observation.process_restart_required &&
                observation.report_received &&
                observation.position_valid &&
                observation.source_timestamp_ns > 0 &&
                observation.source_timestamp_ns <= now_ns &&
                observation.fresh_until_ns > now_ns &&
                observation.report.error_code == 0 &&
                observation.report.warning_code == 0 &&
                (observation.report.brake_mask & expected_mask) ==
                expected_mask &&
                (observation.report.servo_enable_mask & expected_mask) ==
                expected_mask &&
                observation.report.mode == 1 &&
                (observation.report.state == 0 ||
                observation.report.state == 2) &&
                supervised_driver_->read_joint_state(joint_state) &&
                joint_state.joint_count == info_.joints.size();
            if (!ready) {
                static_cast<void>(
                    supervised_driver_->set_command_gate(false, 0));
                supervised_hardware_active_.store(false);
                RCLCPP_WARN(
                    LOGGER,
                    "[%s] Supervised hardware activation refused: "
                    "the observed robot is not command-ready",
                    robot_ip_.c_str());
                return CallbackReturn::ERROR;
            }
            for (std::size_t index = 0;
                index < info_.joints.size(); ++index)
            {
                position_states_[index] = joint_state.positions[index];
                velocity_states_[index] = joint_state.velocities[index];
                position_cmds_[index] = joint_state.positions[index];
                velocity_cmds_[index] = 0.0;
            }
            static_cast<void>(
                supervised_driver_->set_command_gate(false, 0));
            supervised_hardware_active_.store(true);
            initialized_ = true;
            _publish_supervised_state();
            RCLCPP_INFO(
                LOGGER,
                "[%s] Supervised hardware activated without a vendor "
                "lifecycle command",
                robot_ip_.c_str());
            return CallbackReturn::SUCCESS;
        }

        xarm_driver_.arm->clean_error();
        xarm_driver_.arm->clean_warn();
        xarm_driver_.arm->motion_enable(true);
		xarm_driver_.arm->set_mode(velocity_control_ ? XARM_MODE::VELO_JOINT : XARM_MODE::SERVO);
		xarm_driver_.arm->set_state(XARM_STATE::START);

        req_list_controller_ = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
        res_list_controller_ = std::make_shared<controller_manager_msgs::srv::ListControllers::Response>();
        req_switch_controller_ = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        res_switch_controller_ = std::make_shared<controller_manager_msgs::srv::SwitchController::Response>();

        client_list_controller_ = hw_node_->create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
        client_switch_controller_ = hw_node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

        for (uint i = 0; i < position_states_.size(); i++) {
            if (std::isnan(position_states_[i])) {
                position_states_[i] = 0;
                position_cmds_[i] = 0;
            } else {
                position_cmds_[i] = position_states_[i];
            }
        }
        for (uint i = 0; i < velocity_states_.size(); i++) {
            if (std::isnan(velocity_states_[i])) {
                velocity_states_[i] = 0;
                velocity_cmds_[i] = 0;
            } else {
                velocity_cmds_[i] = velocity_states_[i];
            }
        }
        
        RCLCPP_INFO(LOGGER, "[%s] System Sucessfully started!", robot_ip_.c_str());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn UFRobotSystemHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state)
    {
        if (supervised_lifecycle_) {
            std::lock_guard<std::mutex> lock(supervised_nrt_mutex_);
            supervised_hardware_active_.store(false);
            if (supervised_driver_ != nullptr) {
                static_cast<void>(
                    supervised_driver_->set_command_gate(false, 0));
            }
            initialized_ = false;
            _publish_supervised_state();
            RCLCPP_INFO(
                LOGGER,
                "[%s] Supervised hardware deactivated; drives and vendor "
                "state were not changed",
                robot_ip_.c_str());
            return CallbackReturn::SUCCESS;
        }

        RCLCPP_INFO(LOGGER, "[%s] Stopping ...please wait...", robot_ip_.c_str());

        xarm_driver_.arm->set_mode(XARM_MODE::POSE);

        RCLCPP_INFO(LOGGER, "[%s] System sucessfully stopped!", robot_ip_.c_str());
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type UFRobotSystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration &period)
    {
        if (supervised_lifecycle_) {
            xarm_api::SupervisedDriver * driver =
                supervised_rt_driver_.load(std::memory_order_acquire);
            xarm_api::SupervisedDriverJointState joint_state;
            if (driver == nullptr ||
                !driver->read_joint_state(joint_state) ||
                joint_state.joint_count != info_.joints.size())
            {
                return hardware_interface::return_type::ERROR;
            }
            for (std::size_t index = 0;
                index < info_.joints.size(); ++index)
            {
                position_states_[index] = joint_state.positions[index];
                velocity_states_[index] = joint_state.velocities[index];
            }
            return hardware_interface::return_type::OK;
        }

        read_cnts_ += 1;
        read_ready_ = _xarm_is_ready_read();
        rclcpp::Time start = node_->get_clock()->now();

        read_code_ = xarm_driver_.update_joint_states(initialized_);
        // double time_sec = joint_state_msg_->header.stamp.seconds() - start.seconds();
        // read_total_time_ += time_sec;
        // if (time_sec > read_max_time_) {
        //     read_max_time_ = time_sec;
        // }
        // if (read_cnts_ % 6000 == 0) {
        //     RCLCPP_INFO(LOGGER, "[%s] [READ] cnt: %ld, max: %f, mean: %f, failed: %ld", robot_ip_.c_str(), read_cnts_, read_max_time_, read_total_time_ / read_cnts_, read_failed_cnts_);
        // }
        if (read_code_ == 0 && read_ready_) {
            for (int j = 0; j < info_.joints.size(); j++) {
                position_states_[j] = joint_state_msg_->position[j];
                velocity_states_[j] = joint_state_msg_->velocity[j];
                // effort_states_[j] = joint_state_msg_->effort[j];
            }
            if (!initialized_) {
                for (uint i = 0; i < position_states_.size(); i++) {
                    position_cmds_[i] = position_states_[i];
                    velocity_cmds_[i] = 0.0;
                }
            }
        }
        else {
            // initialized_ = read_ready_ && _xarm_is_ready_write();
            if (read_code_) {
                read_failed_cnts_ += 1;
                RCLCPP_INFO(LOGGER, "[%s] Read() returns: %d", robot_ip_.c_str(), read_code_);
                if (read_code_ == ROBOT_IS_DISCONNECTED) {
                    RCLCPP_ERROR(LOGGER, "[%s] Robot is disconnected, ros shutdown", robot_ip_.c_str());
                    rclcpp::shutdown();
                    exit(1);
				}
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type UFRobotSystemHardware::write(const rclcpp::Time & time, const rclcpp::Duration &period)
    {
        if (supervised_lifecycle_) {
            xarm_api::SupervisedDriver * driver =
                supervised_rt_driver_.load(std::memory_order_acquire);
            if (driver == nullptr ||
                !supervised_hardware_active_.load(
                    std::memory_order_acquire) ||
                position_cmds_.size() > xarm_api::kSupervisedDriverMaximumJoints)
            {
                return hardware_interface::return_type::ERROR;
            }
            std::array<
                double,
                xarm_api::kSupervisedDriverMaximumJoints> positions{{0.0}};
            for (std::size_t index = 0;
                index < position_cmds_.size(); ++index)
            {
                if (!std::isfinite(position_cmds_[index])) {
                    return hardware_interface::return_type::ERROR;
                }
                positions[index] = position_cmds_[index];
            }
            return driver->submit_joint_position_command(
                positions, position_cmds_.size()) ?
                hardware_interface::return_type::OK :
                hardware_interface::return_type::ERROR;
        }

        if (_need_reset()) {
            initialized_ = false;
            _deactivate_controller();
            return hardware_interface::return_type::OK;
        }
        initialized_ = true;
        if(reactivate_controller_later_)
        {
            _activate_controller();
            reactivate_controller_later_ = false;
        }
        // std::string pos_str = "[ ";
        // std::string vel_str = "[ ";
        // for (int i = 0; i < position_cmds_.size(); i++) { 
        //     pos_str += std::to_string(position_cmds_[i]); 
        //     pos_str += " ";
        //     vel_str += std::to_string(velocity_cmds_[i]); 
        //     vel_str += " ";
        // }
        // pos_str += "]";
        // vel_str += "]";
        // RCLCPP_INFO(LOGGER, "[%s] positon: %s, velocity: %s", robot_ip_.c_str(), pos_str.c_str(), vel_str.c_str());

        int cmd_ret = 0;
        if (velocity_control_) {
            for (int i = 0; i < velocity_cmds_.size(); i++) { 
                cmds_float_[i] = (float)velocity_cmds_[i];
            }
            // RCLCPP_INFO(LOGGER, "[%s] velocity: %s", robot_ip_.c_str(), vel_str.c_str());
            cmd_ret = xarm_driver_.arm->vc_set_joint_velocity(cmds_float_, true, VELO_DURATION);
            if (cmd_ret != 0) {
                RCLCPP_WARN(LOGGER, "[%s] vc_set_joint_velocity, ret=%d", robot_ip_.c_str(), cmd_ret);
            }
        }
        else {
            for (int i = 0; i < position_cmds_.size(); i++) { 
                cmds_float_[i] = (float)position_cmds_[i];
            }
            curr_write_time_ = node_->get_clock()->now();
            if (curr_write_time_.seconds() - prev_write_time_.seconds() > 1 || _check_cmds_is_change(prev_cmds_float_, cmds_float_)) {
                // RCLCPP_INFO(LOGGER, "[%s] positon: %s", robot_ip_.c_str(), pos_str.c_str());
                cmd_ret = xarm_driver_.arm->set_servo_angle_j(cmds_float_, 0, 0, 0);
                if (cmd_ret != 0) {
                    RCLCPP_WARN(LOGGER, "[%s] set_servo_angle_j, ret= %d", robot_ip_.c_str(), cmd_ret);
                }
                if (cmd_ret == 0) {
                    prev_write_time_ = curr_write_time_;
                    for (int i = 0; i < 7; i++) { 
                        prev_cmds_float_[i] = (float)cmds_float_[i];
                    }
                }
            }
        }

        return hardware_interface::return_type::OK;
    }

    bool UFRobotSystemHardware::_configure_supervised_driver(void)
    {
        std::lock_guard<std::mutex> lock(supervised_nrt_mutex_);
        if (supervised_driver_ != nullptr) {
            RCLCPP_ERROR(
                LOGGER,
                "[%s] Supervised SDK owner is already configured",
                robot_ip_.c_str());
            return false;
        }
        xarm_api::SupervisedDriverConfig config;
        config.robot_address = robot_ip_;
        config.report_type = supervised_report_type_;
        config.joint_count = info_.joints.size();
        config.identity_expectation = {
            static_cast<int>(info_.joints.size()),
            supervised_expected_device_type_};
        config.observation_lease_ns = supervised_observation_lease_ns_;
        config.joint_state_lease_ns = supervised_joint_state_lease_ns_;
        config.io_period_ns = supervised_io_period_ns_;
        config.source_agreement_tolerance_rad =
            supervised_source_agreement_tolerance_rad_;
        config.position_initialization_samples =
            supervised_position_initialization_samples_;
        try {
            supervised_driver_.reset(
                new xarm_api::SupervisedDriver(std::move(config)));
        }
        catch (const std::exception & exception) {
            RCLCPP_ERROR(
                LOGGER,
                "[%s] Supervised SDK owner configuration failed: %s",
                robot_ip_.c_str(),
                exception.what());
            supervised_driver_.reset();
            supervised_rt_driver_.store(nullptr, std::memory_order_release);
            supervised_session_id_.clear();
            _publish_supervised_state();
            return false;
        }
        supervised_session_id_ =
            _new_supervised_session_id(supervised_owner_id_);
        supervised_replay_.clear();
        supervised_hardware_active_.store(false);
        supervised_rt_driver_.store(
            supervised_driver_.get(), std::memory_order_release);
        _publish_supervised_state();
        RCLCPP_INFO(
            LOGGER,
            "[%s] Supervised SDK owner configured with session %s",
            robot_ip_.c_str(),
            supervised_session_id_.c_str());
        return true;
    }

    void UFRobotSystemHardware::_release_supervised_driver(void) noexcept
    {
        if (!supervised_lifecycle_) {
            return;
        }
        try {
            std::lock_guard<std::mutex> lock(supervised_nrt_mutex_);
            supervised_hardware_active_.store(false);
            supervised_rt_driver_.store(nullptr, std::memory_order_release);
            if (supervised_driver_ != nullptr) {
                static_cast<void>(
                    supervised_driver_->set_command_gate(false, 0));
                supervised_driver_->close();
                supervised_driver_.reset();
            }
            supervised_replay_.clear();
            supervised_session_id_.clear();
            _publish_supervised_state();
        }
        catch (...) {
            supervised_rt_driver_.store(nullptr, std::memory_order_release);
            supervised_driver_.reset();
            supervised_session_id_.clear();
        }
    }

    void UFRobotSystemHardware::_init_supervised_ros_boundary(void)
    {
        const auto namespace_found =
            info_.hardware_parameters.find("hw_ns");
        const std::string hardware_namespace =
            namespace_found == info_.hardware_parameters.end() ?
            "xarm" : namespace_found->second;
        const std::string boundary =
            hardware_namespace + "/supervised_lifecycle";
        const rclcpp::QoS state_qos =
            rclcpp::QoS(rclcpp::KeepLast(1)).
            reliable().transient_local();
        supervised_state_publisher_ =
            node_->create_publisher<
            xarm_msgs::msg::SupervisedLifecycleState>(
                boundary + "/state", state_qos);
        supervised_command_service_ =
            node_->create_service<
            xarm_msgs::srv::ExecuteSupervisedLifecycleCommand>(
                boundary + "/execute",
                [this](
                    const std::shared_ptr<
                    xarm_msgs::srv::
                    ExecuteSupervisedLifecycleCommand::Request> request,
                    std::shared_ptr<
                    xarm_msgs::srv::
                    ExecuteSupervisedLifecycleCommand::Response> response)
                {
                    _execute_supervised_command(request, response);
                });
        supervised_gate_service_ =
            node_->create_service<
            xarm_msgs::srv::SetSupervisedCommandGate>(
                boundary + "/set_command_gate",
                [this](
                    const std::shared_ptr<
                    xarm_msgs::srv::
                    SetSupervisedCommandGate::Request> request,
                    std::shared_ptr<
                    xarm_msgs::srv::
                    SetSupervisedCommandGate::Response> response)
                {
                    _set_supervised_command_gate(request, response);
                });
        supervised_state_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() {
                std::lock_guard<std::mutex> lock(supervised_nrt_mutex_);
                _publish_supervised_state();
            });
        supervised_executor_.reset(
            new rclcpp::executors::SingleThreadedExecutor());
        supervised_executor_->add_node(node_);
        supervised_executor_thread_ = std::thread([this]() {
            supervised_executor_->spin();
        });
    }

    void UFRobotSystemHardware::_stop_supervised_ros_boundary(void) noexcept
    {
        if (!supervised_lifecycle_) {
            return;
        }
        supervised_state_timer_.reset();
        supervised_command_service_.reset();
        supervised_gate_service_.reset();
        supervised_state_publisher_.reset();
        if (supervised_executor_ != nullptr) {
            supervised_executor_->cancel();
        }
        if (supervised_executor_thread_.joinable()) {
            supervised_executor_thread_.join();
        }
        if (supervised_executor_ != nullptr && node_ != nullptr) {
            try {
                supervised_executor_->remove_node(node_);
            }
            catch (...) {
            }
        }
        supervised_executor_.reset();
    }

    void UFRobotSystemHardware::_publish_supervised_state(void)
    {
        if (supervised_state_publisher_ == nullptr) {
            return;
        }
        xarm_msgs::msg::SupervisedLifecycleState message;
        message.owner_id = supervised_owner_id_;
        message.session_id = supervised_session_id_;
        message.identity_axis = -1;
        message.identity_device_type = -1;
        message.state = -1;
        message.mode = -1;
        message.command_count = -1;
        message.brake_mask = -1;
        message.servo_enable_mask = -1;
        message.error_code = -1;
        message.warning_code = -1;
        message.last_joint_read_return_code = -1;
        message.hardware_component_active =
            supervised_hardware_active_.load(std::memory_order_acquire);
        if (supervised_driver_ != nullptr) {
            const auto observed =
                supervised_driver_->observe(_steady_now_ns());
            message.generation = observed.generation;
            message.source_steady_time_ns =
                observed.source_timestamp_ns;
            message.fresh_until_steady_time_ns =
                observed.fresh_until_ns;
            message.transport_connected = observed.connected;
            message.report_connected = observed.report_connected;
            message.identity_verified = observed.identity_verified;
            message.process_restart_required =
                observed.process_restart_required;
            message.report_received = observed.report_received;
            message.position_valid = observed.position_valid;
            message.command_gate_open = observed.command_gate_open;
            message.command_gate_valid_until_steady_time_ns =
                observed.command_gate_valid_until_ns;
            message.identity_axis = observed.identity.axis;
            message.identity_device_type =
                observed.identity.device_type;
            message.state = observed.report.state;
            message.mode = observed.report.mode;
            message.command_count = observed.report.command_count;
            message.brake_mask = observed.report.brake_mask;
            message.servo_enable_mask =
                observed.report.servo_enable_mask;
            message.error_code = observed.report.error_code;
            message.warning_code = observed.report.warning_code;
            message.report_sample_count =
                observed.report_sample_count;
            message.joint_read_attempt_count =
                observed.joint_read_attempt_count;
            message.joint_read_success_count =
                observed.joint_read_success_count;
            message.joint_write_attempt_count =
                observed.joint_write_attempt_count;
            message.lifecycle_command_attempt_count =
                observed.lifecycle_command_attempt_count;
            message.last_joint_read_return_code =
                observed.last_joint_read_return_code;
            message.last_joint_write_return_code =
                observed.last_joint_write_return_code;
        }
        supervised_state_publisher_->publish(message);
    }

    void UFRobotSystemHardware::_execute_supervised_command(
        const std::shared_ptr<
        xarm_msgs::srv::ExecuteSupervisedLifecycleCommand::Request> request,
        std::shared_ptr<
        xarm_msgs::srv::ExecuteSupervisedLifecycleCommand::Response> response)
    {
        std::lock_guard<std::mutex> lock(supervised_nrt_mutex_);
        if (supervised_driver_ == nullptr) {
            response->reason = "supervised_driver_unavailable";
            return;
        }
        auto observed = supervised_driver_->observe(_steady_now_ns());
        response->observed_generation = observed.generation;
        SupervisedCommandRecord replay_record;
        const auto replay = supervised_replay_.lookup(
            request->request_id,
            request->command,
            request->expected_generation,
            replay_record);
        if (replay == SupervisedReplayDisposition::kReplay) {
            response->permitted = replay_record.result.permitted;
            response->attempted = replay_record.result.attempted;
            response->return_code = replay_record.result.return_code;
            response->reason = replay_record.result.reason;
            response->observed_generation =
                replay_record.observed_generation;
            return;
        }
        if (replay == SupervisedReplayDisposition::kConflict) {
            response->reason = "request_id_conflict";
            return;
        }
        if (!valid_supervised_request_id(request->request_id)) {
            response->reason = "invalid_request_id";
            return;
        }
        if (observed.process_restart_required) {
            response->reason = "process_restart_required";
            return;
        }
        if (request->expected_generation == 0 ||
            request->expected_generation != observed.generation)
        {
            response->reason = "stale_observation_generation";
            return;
        }
        xarm_api::DriverLifecycleCommand primitive;
        if (!map_supervised_command(request->command, primitive)) {
            response->reason = "unsupported_supervised_command";
            return;
        }
        const auto result =
            supervised_driver_->execute_lifecycle(primitive);
        observed = supervised_driver_->observe(_steady_now_ns());
        response->permitted = result.permitted;
        response->attempted = result.attempted;
        response->return_code = result.return_code;
        response->reason = result.reason;
        response->observed_generation = observed.generation;
        supervised_replay_.remember(
            SupervisedCommandRecord{
                request->request_id,
                request->command,
                request->expected_generation,
                result,
                observed.generation});
        _publish_supervised_state();
    }

    void UFRobotSystemHardware::_set_supervised_command_gate(
        const std::shared_ptr<
        xarm_msgs::srv::SetSupervisedCommandGate::Request> request,
        std::shared_ptr<
        xarm_msgs::srv::SetSupervisedCommandGate::Response> response)
    {
        std::lock_guard<std::mutex> lock(supervised_nrt_mutex_);
        if (supervised_driver_ == nullptr) {
            response->reason = "supervised_driver_unavailable";
            return;
        }
        auto observed = supervised_driver_->observe(_steady_now_ns());
        response->observed_generation = observed.generation;
        if (!request->open) {
            static_cast<void>(
                supervised_driver_->set_command_gate(false, 0));
            response->accepted = true;
            response->reason = "command_gate_closed";
            _publish_supervised_state();
            return;
        }
        if (!valid_supervised_request_id(request->request_id)) {
            response->reason = "invalid_request_id";
            return;
        }
        if (!supervised_hardware_active_.load(
                std::memory_order_acquire))
        {
            response->reason = "hardware_component_inactive";
            return;
        }
        if (observed.process_restart_required) {
            response->reason = "process_restart_required";
            return;
        }
        if (request->expected_generation == 0 ||
            request->expected_generation != observed.generation)
        {
            response->reason = "stale_observation_generation";
            return;
        }
        if (request->lease_duration_ns < MIN_GATE_LEASE_NS ||
            request->lease_duration_ns > supervised_max_gate_lease_ns_)
        {
            response->reason = "invalid_command_gate_lease";
            return;
        }
        const std::int64_t now_ns = _steady_now_ns();
        if (now_ns >
            std::numeric_limits<std::int64_t>::max() -
            request->lease_duration_ns)
        {
            response->reason = "invalid_command_gate_lease";
            return;
        }
        const std::int64_t valid_until_ns =
            now_ns + request->lease_duration_ns;
        response->accepted =
            supervised_driver_->set_command_gate(
                true, valid_until_ns);
        response->reason = response->accepted ?
            "command_gate_open" : "command_gate_refused";
        response->valid_until_steady_time_ns =
            response->accepted ? valid_until_ns : 0;
        observed = supervised_driver_->observe(_steady_now_ns());
        response->observed_generation = observed.generation;
        _publish_supervised_state();
    }

    std::int64_t UFRobotSystemHardware::_steady_now_ns(void)
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    std::string UFRobotSystemHardware::_new_supervised_session_id(
        const std::string& owner_id)
    {
        const auto sequence =
            SESSION_COUNTER.fetch_add(1, std::memory_order_acq_rel) + 1;
        std::ostringstream value;
        value << owner_id << ':' << static_cast<long long>(getpid()) <<
            ':' << _steady_now_ns() << ':' << sequence;
        return value.str();
    }

    void UFRobotSystemHardware::_deactivate_controller(void) {
        if(reactivate_controller_later_)
            return;
        // RCLCPP_INFO(LOGGER, "DEACTIVATE CONTROLLER!! ");
        int ret = _call_request(client_list_controller_, req_list_controller_, res_list_controller_);
        bool valid_operation = false;
        if (ret == 0 && res_list_controller_->controller.size() > 0) {
            req_switch_controller_->activate_controllers.resize(0);
            req_switch_controller_->deactivate_controllers.resize(res_list_controller_->controller.size());
            for (uint i = 0; i < res_list_controller_->controller.size(); i++) {
                // RCLCPP_ERROR(LOGGER, "STATE: %s", res_list_controller_->controller[i].state.c_str());
                if(res_list_controller_->controller[i].state == std::string("active")){
                // for situation of initial launch with emg stop pressed, launch file will activate controller and it takes a while
                    valid_operation = true;
                }
                // req_switch_controller_->activate_controllers[i] = res_list_controller_->controller[i].name;
                req_switch_controller_->deactivate_controllers[i] = res_list_controller_->controller[i].name;
            }
            req_switch_controller_->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
            req_switch_controller_->timeout = rclcpp::Duration::from_seconds(2.0);
            if(valid_operation)
            {
                _call_request(client_switch_controller_, req_switch_controller_, res_switch_controller_);
                reactivate_controller_later_ = true; // Not setting this indicator until activated controller disabled!
            }
        }
    }

    void UFRobotSystemHardware::_activate_controller(void) {
        // RCLCPP_INFO(LOGGER, "ACTIVATE CONTROLLER!! ");
        int ret = _call_request(client_list_controller_, req_list_controller_, res_list_controller_);
        if (ret == 0 && res_list_controller_->controller.size() > 0) {
            req_switch_controller_->deactivate_controllers.resize(0);
            req_switch_controller_->activate_controllers.resize(res_list_controller_->controller.size());
            for (uint i = 0; i < res_list_controller_->controller.size(); i++) {
                req_switch_controller_->activate_controllers[i] = res_list_controller_->controller[i].name;
                // req_switch_controller_->deactivate_controllers[i] = res_list_controller_->controller[i].name;
            }
            req_switch_controller_->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
            req_switch_controller_->timeout = rclcpp::Duration::from_seconds(2.0);
            _call_request(client_switch_controller_, req_switch_controller_, res_switch_controller_);
        }
        update_goal_state_pub_->publish(update_goal_state_msg_);
    }

    bool UFRobotSystemHardware::_check_cmds_is_change(float *prev, float *cur, double threshold)
	{
		for (int i = 0; i < 7; i++) {
            if (std::abs(cur[i] - prev[i]) > threshold) return true;
        }
        return false;
	}

    bool UFRobotSystemHardware::_xarm_is_ready_read(void)
    {
        static int last_err = xarm_driver_.curr_err;
		int curr_err = xarm_driver_.curr_err;
        if (curr_err != 0) {
            if (last_err != curr_err) {
                RCLCPP_ERROR(LOGGER, "[%s] UFACTORY Error detected! Code C%d -> [ %s ] ", robot_ip_.c_str(), curr_err, xarm_driver_.controller_error_interpreter(curr_err).c_str());
            }
        }
        last_err = curr_err;
        return last_err == 0;
    }

    bool UFRobotSystemHardware::_xarm_is_ready_write(void)
    {
        static bool last_not_ready = false;
        static int last_state = xarm_driver_.curr_state;
        static int last_mode = xarm_driver_.curr_mode;
        int curr_mode = xarm_driver_.curr_mode;
		int curr_state = xarm_driver_.curr_state;

        if (!_xarm_is_ready_read()) {
            last_not_ready = true;
            return false;
        }

        if (curr_state > 2) {
            if (last_state != curr_state) {
                last_state = curr_state;
                RCLCPP_WARN(LOGGER, "[%s] Robot State detected! State: %d", robot_ip_.c_str(), curr_state);
            }
            last_not_ready = true;
            return false;
        }
        last_state = curr_state;

        if (!(velocity_control_ ? curr_mode == XARM_MODE::VELO_JOINT : curr_mode == XARM_MODE::SERVO)) {
            if (last_mode != curr_mode) {
                last_mode = curr_mode;
                RCLCPP_WARN(LOGGER, "[%s] Robot Mode detected! Mode: %d", robot_ip_.c_str(), curr_mode);
            }
            last_not_ready = true;
            return false;
        }
        last_mode = curr_mode;

        if (last_not_ready) {
            RCLCPP_INFO(LOGGER, "[%s] Robot is Ready", robot_ip_.c_str());
        }
        last_not_ready = false;
        return true;
    }

    bool UFRobotSystemHardware::_need_reset()
    {
        bool is_not_ready = !_xarm_is_ready_write();
        bool write_succeed = write_code_ == 0;
        if (!write_succeed) {
            // int ret = xarm_driver_.arm->set_state(XARM_STATE::STOP);
            // RCLCPP_ERROR(LOGGER, "[%s] Write() failed, failed_ret=%d !, Setting Robot State to STOP... (ret: %d)", robot_ip_.c_str(), write_code_, ret);
            RCLCPP_ERROR(LOGGER, "[%s] Write() failed, failed_ret=%d !", robot_ip_.c_str(), write_code_);
            if (write_code_ == SERVICE_IS_PERSISTENT_BUT_INVALID || write_code_ == SERVICE_CALL_FAILED) {
                RCLCPP_ERROR(LOGGER, "[%s] Service is invaild, ros shutdown", robot_ip_.c_str());
                rclcpp::shutdown();
                exit(1);
            }
            else if (write_code_ == ROBOT_IS_DISCONNECTED) {
                RCLCPP_ERROR(LOGGER, "[%s] Robot is disconnected, ros shutdown", robot_ip_.c_str());
                rclcpp::shutdown();
                exit(1);
            }
            write_code_ = 0;
        }
        return is_not_ready || !write_succeed || read_code_ != 0 || !read_ready_;
    }
}
