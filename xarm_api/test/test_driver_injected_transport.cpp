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

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "xarm_api/driver_lifecycle.h"
#include "xarm_api/xarm_driver.h"

namespace
{
class FakeInjectedLifecycleTransport
  : public xarm_api::DriverLifecycleTransport
{
public:
  int connect() override
  {
    ++connect_calls;
    events.push_back("connect");
    return connect_result;
  }

  int read_robot_identity(xarm_api::DriverRobotIdentity & output) override
  {
    ++identity_read_calls;
    events.push_back("read_robot_identity");
    output = identity;
    return identity_read_result;
  }

  int read_error_warning(
    std::array<int, xarm_api::kDriverErrorWarningWords> & output) override
  {
    ++error_warning_read_calls;
    events.push_back("read_error_warning");
    output = error_warning;
    return error_warning_result;
  }

  int read_servo_debug(
    std::array<int, xarm_api::kDriverServoDebugWords> & output) override
  {
    ++servo_debug_read_calls;
    events.push_back("read_servo_debug");
    output = servo_debug;
    return servo_debug_result;
  }

  int clear_error() override
  {
    ++clear_error_calls;
    events.push_back("clear_error");
    return clear_error_result;
  }

  int clear_warning() override
  {
    ++clear_warning_calls;
    events.push_back("clear_warning");
    return clear_warning_result;
  }

  int set_motion_enabled(bool enabled, int servo_id) override
  {
    ++set_motion_enabled_calls;
    last_motion_enabled = enabled;
    last_servo_id = servo_id;
    events.push_back(enabled ? "enable_motion" : "disable_motion");
    return set_motion_enabled_result;
  }

  int set_mode(int mode) override
  {
    ++set_mode_calls;
    last_mode = mode;
    events.push_back("set_mode");
    return set_mode_result;
  }

  int set_state(int state) override
  {
    ++set_state_calls;
    last_state = state;
    events.push_back("set_state");
    return set_state_result;
  }

  int set_pose_mode() override
  {
    ++set_pose_mode_calls;
    events.push_back("set_pose_mode");
    return set_pose_mode_result;
  }

  void release_callbacks() override
  {
    ++release_callbacks_calls;
    events.push_back("release_callbacks");
  }

  void disconnect() override
  {
    ++disconnect_calls;
    events.push_back("disconnect");
  }

  int connect_result = 0;
  int identity_read_result = 0;
  int error_warning_result = 0;
  int servo_debug_result = 0;
  int clear_error_result = 0;
  int clear_warning_result = 0;
  int set_motion_enabled_result = 0;
  int set_mode_result = 0;
  int set_state_result = 0;
  int set_pose_mode_result = 0;
  int connect_calls = 0;
  int identity_read_calls = 0;
  int error_warning_read_calls = 0;
  int servo_debug_read_calls = 0;
  int clear_error_calls = 0;
  int clear_warning_calls = 0;
  int set_motion_enabled_calls = 0;
  int set_mode_calls = 0;
  int set_state_calls = 0;
  int set_pose_mode_calls = 0;
  int release_callbacks_calls = 0;
  int disconnect_calls = 0;
  bool last_motion_enabled = false;
  int last_servo_id = -1;
  int last_mode = -1;
  int last_state = -1;
  std::vector<std::string> events;
  xarm_api::DriverRobotIdentity identity{6, 9, "EXPECTED000001"};
  std::array<int, xarm_api::kDriverErrorWarningWords> error_warning{{0, 0}};
  std::array<int, xarm_api::kDriverServoDebugWords> servo_debug{{0}};
};

class DriverInjectedTransportTest : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  static rclcpp::Node::SharedPtr make_node(
    const std::string & name,
    bool read_only,
    bool add_gripper = false,
    const std::string & access_mode = "")
  {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.append_parameter_override("read_only", read_only);
    if (!access_mode.empty()) {
      options.append_parameter_override("access_mode", access_mode);
    }
    options.append_parameter_override("dof", 6);
    options.append_parameter_override(
      "expected_robot_sn", std::string("EXPECTED000001"));
    options.append_parameter_override("expected_robot_device_type", 9);
    options.append_parameter_override("add_gripper", add_gripper);
    return rclcpp::Node::make_shared(name, options);
  }

  static bool has_service(
    const rclcpp::Node::SharedPtr & node,
    const std::string & suffix)
  {
    for (const auto & entry : node->get_service_names_and_types()) {
      if (
        entry.first.size() >= suffix.size() &&
        entry.first.compare(
          entry.first.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        return true;
      }
    }
    return false;
  }
};
}  // namespace

TEST_F(DriverInjectedTransportTest, ReadOnlyDriverUsesFakeWithoutNativeSdk)
{
  auto transport = std::make_shared<FakeInjectedLifecycleTransport>();
  auto node = make_node("read_only_injected_driver", true, true);
  std::string unused_address = "not-contacted.invalid";

  xarm_api::XArmDriver driver;
  driver.init_with_injected_lifecycle_transport(
    node, unused_address, transport);

  EXPECT_EQ(nullptr, driver.arm);
  EXPECT_TRUE(driver.is_connected());
  EXPECT_EQ(1, transport->connect_calls);
  EXPECT_EQ(1, transport->identity_read_calls);
  EXPECT_EQ(1, transport->error_warning_read_calls);
  EXPECT_EQ(1, transport->servo_debug_read_calls);
  EXPECT_EQ(0, transport->clear_error_calls);
  EXPECT_FALSE(has_service(node, "/xarm/clean_error"));
  EXPECT_FALSE(has_service(node, "/xarm/motion_enable"));
  const auto denied = driver.execute_supervised_lifecycle_command(
    xarm_api::DriverLifecycleCommand{
    xarm_api::DriverLifecycleCommandKind::kClearError, 0, 8});
  EXPECT_FALSE(denied.permitted);
  EXPECT_FALSE(denied.attempted);

  driver.shutdown();
  driver.shutdown();

  EXPECT_FALSE(driver.is_connected());
  EXPECT_EQ(0, transport->set_pose_mode_calls);
  EXPECT_EQ(1, transport->release_callbacks_calls);
  EXPECT_EQ(1, transport->disconnect_calls);
  EXPECT_EQ(
    (std::vector<std::string>{
    "connect", "read_robot_identity", "read_error_warning",
    "read_servo_debug", "release_callbacks", "disconnect"}),
    transport->events);
}

TEST_F(
  DriverInjectedTransportTest,
  SupervisedDriverRequiresExplicitInternalCommandsAndHasNoLegacySurface)
{
  auto transport = std::make_shared<FakeInjectedLifecycleTransport>();
  transport->servo_debug[0] = 1;
  transport->servo_debug[1] = 40;
  auto node = make_node(
    "supervised_injected_driver", false, true, "supervised_lifecycle");
  std::string unused_address = "not-contacted.invalid";

  xarm_api::XArmDriver driver;
  driver.init_with_injected_lifecycle_transport(
    node, unused_address, transport, true);

  EXPECT_EQ(nullptr, driver.arm);
  EXPECT_TRUE(driver.is_connected());
  EXPECT_EQ(0, transport->clear_error_calls);
  EXPECT_FALSE(has_service(node, "/xarm/clean_error"));
  EXPECT_FALSE(has_service(node, "/xarm/motion_enable"));

  const std::vector<xarm_api::DriverLifecycleCommand> commands{
    {
      xarm_api::DriverLifecycleCommandKind::kClearError, 0, 8},
    {
      xarm_api::DriverLifecycleCommandKind::kClearWarning, 0, 8},
    {
      xarm_api::DriverLifecycleCommandKind::kSetMotionEnabled, 1, 8},
    {
      xarm_api::DriverLifecycleCommandKind::kSetMode, 1, 8},
    {
      xarm_api::DriverLifecycleCommandKind::kSetState, 0, 8},
  };
  for (const auto & command : commands) {
    const auto result =
      driver.execute_supervised_lifecycle_command(command);
    EXPECT_TRUE(result.permitted);
    EXPECT_TRUE(result.attempted);
    EXPECT_EQ(0, result.return_code);
  }

  EXPECT_EQ(1, transport->clear_error_calls);
  EXPECT_EQ(1, transport->clear_warning_calls);
  EXPECT_EQ(1, transport->set_motion_enabled_calls);
  EXPECT_TRUE(transport->last_motion_enabled);
  EXPECT_EQ(8, transport->last_servo_id);
  EXPECT_EQ(1, transport->set_mode_calls);
  EXPECT_EQ(1, transport->last_mode);
  EXPECT_EQ(1, transport->set_state_calls);
  EXPECT_EQ(0, transport->last_state);

  driver.shutdown();

  EXPECT_EQ(0, transport->set_pose_mode_calls);
  EXPECT_EQ(1, transport->release_callbacks_calls);
  EXPECT_EQ(1, transport->disconnect_calls);
  EXPECT_EQ(
    (std::vector<std::string>{
    "connect", "read_robot_identity", "read_error_warning",
    "read_servo_debug", "clear_error", "clear_warning",
    "enable_motion", "set_mode", "set_state",
    "release_callbacks", "disconnect"}),
    transport->events);
}

TEST_F(DriverInjectedTransportTest, InvalidAccessModeFailsBeforeTransport)
{
  auto transport = std::make_shared<FakeInjectedLifecycleTransport>();
  auto node = make_node(
    "invalid_access_mode_driver", false, false, "control");
  std::string unused_address = "not-contacted.invalid";

  xarm_api::XArmDriver driver;
  driver.init_with_injected_lifecycle_transport(
    node, unused_address, transport);

  EXPECT_FALSE(driver.is_connected());
  EXPECT_EQ(nullptr, driver.arm);
  EXPECT_EQ(0, transport->connect_calls);
  EXPECT_EQ(0, transport->release_callbacks_calls);
  EXPECT_EQ(0, transport->disconnect_calls);

  const auto denied = driver.execute_supervised_lifecycle_command(
    xarm_api::DriverLifecycleCommand{
    xarm_api::DriverLifecycleCommandKind::kClearError, 0, 8});
  EXPECT_FALSE(denied.permitted);
  EXPECT_FALSE(denied.attempted);
  EXPECT_STREQ("driver_not_available", denied.reason);
}

TEST_F(DriverInjectedTransportTest, IdentityMismatchFailsBeforeInspection)
{
  auto transport = std::make_shared<FakeInjectedLifecycleTransport>();
  transport->identity.serial = "DIFFERENT00001";
  auto node = make_node("mismatched_injected_driver", true);
  std::string unused_address = "not-contacted.invalid";

  xarm_api::XArmDriver driver;
  driver.init_with_injected_lifecycle_transport(
    node, unused_address, transport);

  EXPECT_FALSE(driver.is_connected());
  EXPECT_EQ(0, transport->error_warning_read_calls);
  EXPECT_EQ(0, transport->servo_debug_read_calls);
  EXPECT_EQ(0, transport->clear_error_calls);
  EXPECT_EQ(1, transport->release_callbacks_calls);
  EXPECT_EQ(1, transport->disconnect_calls);

  driver.shutdown();

  EXPECT_EQ(1, transport->release_callbacks_calls);
  EXPECT_EQ(1, transport->disconnect_calls);
  EXPECT_EQ(
    (std::vector<std::string>{
    "connect", "read_robot_identity", "release_callbacks", "disconnect"}),
    transport->events);
}

TEST_F(DriverInjectedTransportTest, ControlPolicyWritesOnlyToInjectedFake)
{
  auto transport = std::make_shared<FakeInjectedLifecycleTransport>();
  transport->servo_debug[0] = 1;
  transport->servo_debug[1] = 40;
  auto node = make_node("control_injected_driver", false);
  std::string unused_address = "not-contacted.invalid";

  xarm_api::XArmDriver driver;
  driver.init_with_injected_lifecycle_transport(
    node, unused_address, transport, true);

  EXPECT_EQ(nullptr, driver.arm);
  EXPECT_TRUE(driver.is_connected());
  EXPECT_EQ(1, transport->clear_error_calls);
  EXPECT_FALSE(has_service(node, "/xarm/clean_error"));
  EXPECT_FALSE(has_service(node, "/xarm/motion_enable"));
  const auto denied = driver.execute_supervised_lifecycle_command(
    xarm_api::DriverLifecycleCommand{
    xarm_api::DriverLifecycleCommandKind::kClearError, 0, 8});
  EXPECT_FALSE(denied.permitted);
  EXPECT_FALSE(denied.attempted);

  driver.shutdown();

  EXPECT_FALSE(driver.is_connected());
  EXPECT_EQ(1, transport->set_pose_mode_calls);
  EXPECT_EQ(1, transport->release_callbacks_calls);
  EXPECT_EQ(1, transport->disconnect_calls);
  EXPECT_EQ(
    (std::vector<std::string>{
    "connect", "read_robot_identity", "read_error_warning",
    "read_servo_debug", "clear_error", "release_callbacks",
    "set_pose_mode", "disconnect"}),
    transport->events);
}

TEST_F(DriverInjectedTransportTest, FailedConnectionCleanupIsNotRepeated)
{
  auto transport = std::make_shared<FakeInjectedLifecycleTransport>();
  transport->connect_result = -2;
  auto node = make_node("failed_injected_driver", true);
  std::string unused_address = "not-contacted.invalid";

  xarm_api::XArmDriver driver;
  driver.init_with_injected_lifecycle_transport(
    node, unused_address, transport);

  EXPECT_FALSE(driver.is_connected());
  EXPECT_EQ(0, transport->identity_read_calls);
  EXPECT_EQ(0, transport->error_warning_read_calls);
  EXPECT_EQ(1, transport->release_callbacks_calls);
  EXPECT_EQ(1, transport->disconnect_calls);

  driver.shutdown();

  EXPECT_EQ(1, transport->release_callbacks_calls);
  EXPECT_EQ(1, transport->disconnect_calls);
  EXPECT_EQ(
    (std::vector<std::string>{
    "connect", "release_callbacks", "disconnect"}),
    transport->events);
}
