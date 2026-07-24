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
#include <string>
#include <vector>

#include "xarm_api/driver_lifecycle.h"

namespace
{
class FakeLifecycleTransport : public xarm_api::DriverLifecycleTransport
{
public:
  int connect() override
  {
    ++connect_calls;
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
    return clear_error_result;
  }

  int clear_warning() override
  {
    ++clear_warning_calls;
    return clear_warning_result;
  }

  int set_motion_enabled(bool enabled, int servo_id) override
  {
    ++set_motion_enabled_calls;
    last_motion_enabled = enabled;
    last_servo_id = servo_id;
    return set_motion_enabled_result;
  }

  int set_mode(int mode) override
  {
    ++set_mode_calls;
    last_mode = mode;
    return set_mode_result;
  }

  int set_state(int state) override
  {
    ++set_state_calls;
    last_state = state;
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
  xarm_api::DriverRobotIdentity identity{6, 9};
  std::array<int, xarm_api::kDriverErrorWarningWords> error_warning{{0, 0}};
  std::array<int, xarm_api::kDriverServoDebugWords> servo_debug{{0}};
};
}  // namespace

TEST(DriverLifecycle, ReadOnlyObservesFaultsWithoutWriting)
{
  FakeLifecycleTransport transport;
  transport.error_warning = {{17, 3}};
  transport.servo_debug[0] = 1;
  transport.servo_debug[1] = 40;
  transport.servo_debug[4] = 1;
  transport.servo_debug[5] = 23;

  const xarm_api::DriverStartupObservation observation =
    xarm_api::observe_driver_startup(
    transport, xarm_api::DriverAccessPolicy(true), 6);

  EXPECT_TRUE(observation.connected());
  EXPECT_TRUE(observation.accepted());
  EXPECT_EQ(17, observation.error_warning[0]);
  EXPECT_EQ(6U, observation.inspected_joint_count);
  EXPECT_EQ(1, transport.connect_calls);
  EXPECT_EQ(1, transport.error_warning_read_calls);
  EXPECT_EQ(1, transport.servo_debug_read_calls);
  EXPECT_EQ(0, transport.clear_error_calls);

  xarm_api::close_driver_transport(
    transport, xarm_api::DriverAccessPolicy(true), observation.connected());

  EXPECT_EQ(0, transport.set_pose_mode_calls);
  EXPECT_EQ(1, transport.release_callbacks_calls);
  EXPECT_EQ(1, transport.disconnect_calls);
  EXPECT_EQ(
    (std::vector<std::string>{
    "read_error_warning", "read_servo_debug",
    "release_callbacks", "disconnect"}),
    transport.events);
}

TEST(DriverLifecycle, ControlModePreservesAutomaticFaultClearingAndPoseShutdown)
{
  FakeLifecycleTransport transport;
  transport.servo_debug[0] = 1;
  transport.servo_debug[1] = 40;
  transport.servo_debug[4] = 1;
  transport.servo_debug[5] = 23;

  const xarm_api::DriverStartupObservation observation =
    xarm_api::observe_driver_startup(
    transport, xarm_api::DriverAccessPolicy(false), 6);

  EXPECT_TRUE(observation.connected());
  EXPECT_TRUE(observation.accepted());
  EXPECT_TRUE(observation.automatic_fault_clear_attempted[0]);
  EXPECT_TRUE(observation.automatic_fault_clear_attempted[2]);
  EXPECT_EQ(2, transport.clear_error_calls);

  xarm_api::close_driver_transport(
    transport, xarm_api::DriverAccessPolicy(false), observation.connected());

  EXPECT_EQ(1, transport.set_pose_mode_calls);
  EXPECT_EQ(1, transport.release_callbacks_calls);
  EXPECT_EQ(1, transport.disconnect_calls);
  EXPECT_EQ(
    (std::vector<std::string>{
    "read_error_warning", "read_servo_debug",
    "release_callbacks", "set_pose_mode", "disconnect"}),
    transport.events);
}

TEST(DriverLifecycle, SupervisedModeWritesOnlyAfterExplicitPrimitive)
{
  FakeLifecycleTransport transport;
  transport.servo_debug[0] = 1;
  transport.servo_debug[1] = 40;
  const xarm_api::DriverAccessPolicy policy(
    xarm_api::DriverAccessMode::kSupervisedLifecycle);

  const xarm_api::DriverStartupObservation observation =
    xarm_api::observe_driver_startup(transport, policy, 6);

  EXPECT_TRUE(observation.connected());
  EXPECT_EQ(0, transport.clear_error_calls);

  const auto enabled = xarm_api::execute_supervised_lifecycle_command(
    transport, policy, observation.connected(),
    xarm_api::DriverLifecycleCommand{
    xarm_api::DriverLifecycleCommandKind::kSetMotionEnabled, 1, 8});
  EXPECT_TRUE(enabled.permitted);
  EXPECT_TRUE(enabled.attempted);
  EXPECT_EQ(0, enabled.return_code);
  EXPECT_EQ(1, transport.set_motion_enabled_calls);
  EXPECT_TRUE(transport.last_motion_enabled);
  EXPECT_EQ(8, transport.last_servo_id);

  const auto invalid = xarm_api::execute_supervised_lifecycle_command(
    transport, policy, observation.connected(),
    xarm_api::DriverLifecycleCommand{
    xarm_api::DriverLifecycleCommandKind::kSetMotionEnabled, 2, 8});
  EXPECT_TRUE(invalid.permitted);
  EXPECT_FALSE(invalid.attempted);
  EXPECT_STREQ("invalid_motion_enable_argument", invalid.reason);
  EXPECT_EQ(1, transport.set_motion_enabled_calls);

  xarm_api::close_driver_transport(
    transport, policy, observation.connected());
  EXPECT_EQ(0, transport.set_pose_mode_calls);
}

TEST(DriverLifecycle, NonSupervisedModesCannotUseInternalPrimitivePort)
{
  for (const xarm_api::DriverAccessMode mode : {
    xarm_api::DriverAccessMode::kLegacyControl,
    xarm_api::DriverAccessMode::kReadOnly})
  {
    FakeLifecycleTransport transport;
    const auto result = xarm_api::execute_supervised_lifecycle_command(
      transport, xarm_api::DriverAccessPolicy(mode), true,
      xarm_api::DriverLifecycleCommand{
      xarm_api::DriverLifecycleCommandKind::kClearError, 0, 8});
    EXPECT_FALSE(result.permitted);
    EXPECT_FALSE(result.attempted);
    EXPECT_STREQ("access_mode_not_supervised", result.reason);
    EXPECT_EQ(0, transport.clear_error_calls);
  }
}

TEST(DriverLifecycle, FailedConnectionStopsBeforeInspectionOrCommands)
{
  FakeLifecycleTransport transport;
  transport.connect_result = -2;
  transport.servo_debug[0] = 1;
  transport.servo_debug[1] = 40;

  const xarm_api::DriverStartupObservation observation =
    xarm_api::observe_driver_startup(
    transport, xarm_api::DriverAccessPolicy(false), 6);

  EXPECT_FALSE(observation.connected());
  EXPECT_FALSE(observation.accepted());
  EXPECT_EQ(-2, observation.connect_result);
  EXPECT_EQ(1, transport.connect_calls);
  EXPECT_EQ(0, transport.error_warning_read_calls);
  EXPECT_EQ(0, transport.servo_debug_read_calls);
  EXPECT_EQ(0, transport.clear_error_calls);
  EXPECT_TRUE(observation.failed_connection_cleanup_performed);
  EXPECT_EQ(0, transport.set_pose_mode_calls);
  EXPECT_EQ(1, transport.release_callbacks_calls);
  EXPECT_EQ(1, transport.disconnect_calls);
  EXPECT_EQ(
    (std::vector<std::string>{"release_callbacks", "disconnect"}),
    transport.events);
}

TEST(DriverLifecycle, FailedServoInspectionCannotTriggerFaultClearing)
{
  FakeLifecycleTransport transport;
  transport.servo_debug_result = -7;
  transport.servo_debug[0] = 1;
  transport.servo_debug[1] = 40;

  const xarm_api::DriverStartupObservation observation =
    xarm_api::observe_driver_startup(
    transport, xarm_api::DriverAccessPolicy(false), 6);

  EXPECT_TRUE(observation.connected());
  EXPECT_TRUE(observation.accepted());
  EXPECT_EQ(-7, observation.servo_debug_result);
  EXPECT_EQ(0, transport.clear_error_calls);
}

TEST(DriverLifecycle, MatchingIdentityPrecedesInspectionAndCommands)
{
  FakeLifecycleTransport transport;
  transport.servo_debug[0] = 1;
  transport.servo_debug[1] = 40;
  const xarm_api::DriverIdentityExpectation expected{6, 9};

  const xarm_api::DriverStartupObservation observation =
    xarm_api::observe_driver_startup(
    transport, xarm_api::DriverAccessPolicy(false), 6, expected);

  EXPECT_TRUE(observation.connected());
  EXPECT_TRUE(observation.accepted());
  EXPECT_TRUE(observation.identity_check_required);
  EXPECT_EQ(0, observation.identity_read_result);
  EXPECT_TRUE(observation.identity_matched);
  EXPECT_EQ(1, transport.identity_read_calls);
  EXPECT_EQ(1, transport.error_warning_read_calls);
  EXPECT_EQ(1, transport.servo_debug_read_calls);
  EXPECT_EQ(1, transport.clear_error_calls);
  EXPECT_EQ(
    (std::vector<std::string>{
    "read_robot_identity", "read_error_warning", "read_servo_debug"}),
    transport.events);
}

TEST(DriverLifecycle, AxisOrDeviceMismatchClosesBeforeInspectionOrCommands)
{
  const std::array<xarm_api::DriverRobotIdentity, 2> mismatches{{
    {7, 9},
    {6, 12},
  }};
  const xarm_api::DriverIdentityExpectation expected{6, 9};

  for (const auto & mismatch : mismatches) {
    FakeLifecycleTransport transport;
    transport.identity = mismatch;

    const xarm_api::DriverStartupObservation observation =
      xarm_api::observe_driver_startup(
      transport, xarm_api::DriverAccessPolicy(true), 6, expected);

    EXPECT_TRUE(observation.connected());
    EXPECT_FALSE(observation.accepted());
    EXPECT_TRUE(observation.failed_identity_cleanup_performed);
    EXPECT_EQ(0, transport.error_warning_read_calls);
    EXPECT_EQ(0, transport.servo_debug_read_calls);
    EXPECT_EQ(1, transport.release_callbacks_calls);
    EXPECT_EQ(1, transport.disconnect_calls);
  }
}

TEST(DriverLifecycle, IdentityReadFailureClosesBeforeInspectionOrCommands)
{
  FakeLifecycleTransport transport;
  transport.identity_read_result = -9;
  const xarm_api::DriverIdentityExpectation expected{6, 9};

  const xarm_api::DriverStartupObservation observation =
    xarm_api::observe_driver_startup(
    transport, xarm_api::DriverAccessPolicy(true), 6, expected);

  EXPECT_TRUE(observation.connected());
  EXPECT_FALSE(observation.accepted());
  EXPECT_EQ(-9, observation.identity_read_result);
  EXPECT_TRUE(observation.failed_identity_cleanup_performed);
  EXPECT_EQ(0, transport.error_warning_read_calls);
  EXPECT_EQ(0, transport.servo_debug_read_calls);
  EXPECT_EQ(0, transport.clear_error_calls);
  EXPECT_EQ(1, transport.release_callbacks_calls);
  EXPECT_EQ(1, transport.disconnect_calls);
}
