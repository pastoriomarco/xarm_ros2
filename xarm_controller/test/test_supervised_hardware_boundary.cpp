// Copyright 2026 Flexin Group SRL
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Flexin Group SRL nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <string>
#include <utility>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "xarm_controller/hardware/uf_robot_system_hardware.h"

namespace uf_robot_hardware
{
namespace
{

hardware_interface::InterfaceInfo interface(const std::string & name)
{
  hardware_interface::InterfaceInfo result;
  result.name = name;
  result.size = 1;
  result.enable_limits = false;
  return result;
}

hardware_interface::HardwareComponentInterfaceParams parameters()
{
  hardware_interface::HardwareInfo info;
  info.name = "lite6_system";
  info.type = "system";
  info.hardware_plugin_name =
    "uf_robot_hardware/UFRobotSystemHardware";
  info.hardware_parameters = {
    {"robot_ip", "192.0.2.10"},
    {"transport_owner_id", "lite6-test:sole-owner"},
    {"expected_robot_device_type", "9"},
    {"access_mode", "supervised_lifecycle"},
    {"report_type", "rich"},
    {"dof", "6"},
    {"robot_type", "lite"},
    {"prefix", ""},
    {"hw_ns", "ufactory"},
    {"velocity_control", "false"},
    {"add_gripper", "false"},
    {"add_bio_gripper", "false"},
    {"controller_manager_activity_topic", "/controller_manager/activity"},
    {"trajectory_controller_name", "lite6_traj_controller"},
    {"shutdown_stationary_tolerance_rad", "0.001"},
    {"shutdown_stationary_dwell_ms", "1000"},
  };
  for (int index = 1; index <= 6; ++index) {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint" + std::to_string(index);
    joint.type = "joint";
    joint.command_interfaces = {
      interface(hardware_interface::HW_IF_POSITION),
      interface(hardware_interface::HW_IF_VELOCITY)};
    joint.state_interfaces = {
      interface(hardware_interface::HW_IF_POSITION),
      interface(hardware_interface::HW_IF_VELOCITY)};
    info.joints.push_back(std::move(joint));
  }
  hardware_interface::HardwareComponentInterfaceParams result;
  result.hardware_info = std::move(info);
  return result;
}

class SupervisedHardwareBoundaryTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(
  SupervisedHardwareBoundaryTest,
  InitializationCreatesRosBoundaryWithoutOpeningTransport)
{
  UFRobotSystemHardware hardware;
  EXPECT_EQ(
    hardware.on_init(parameters()),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS);
}

TEST_F(
  SupervisedHardwareBoundaryTest,
  RepeatedImmediateDestructionCannotLoseExecutorCancellation)
{
  for (int iteration = 0; iteration < 16; ++iteration) {
    SCOPED_TRACE(iteration);
    UFRobotSystemHardware hardware;
    ASSERT_EQ(
      hardware.on_init(parameters()),
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS);
  }
}

}  // namespace
}  // namespace uf_robot_hardware
