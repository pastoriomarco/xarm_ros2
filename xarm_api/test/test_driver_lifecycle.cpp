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

  int read_error_warning(
    std::array<int, xarm_api::kDriverErrorWarningWords> & output) override
  {
    ++error_warning_read_calls;
    output = error_warning;
    return error_warning_result;
  }

  int read_servo_debug(
    std::array<int, xarm_api::kDriverServoDebugWords> & output) override
  {
    ++servo_debug_read_calls;
    output = servo_debug;
    return servo_debug_result;
  }

  int clear_error() override
  {
    ++clear_error_calls;
    return clear_error_result;
  }

  int set_pose_mode() override
  {
    ++set_pose_mode_calls;
    return set_pose_mode_result;
  }

  void disconnect() override
  {
    ++disconnect_calls;
  }

  int connect_result = 0;
  int error_warning_result = 0;
  int servo_debug_result = 0;
  int clear_error_result = 0;
  int set_pose_mode_result = 0;
  int connect_calls = 0;
  int error_warning_read_calls = 0;
  int servo_debug_read_calls = 0;
  int clear_error_calls = 0;
  int set_pose_mode_calls = 0;
  int disconnect_calls = 0;
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
  EXPECT_EQ(17, observation.error_warning[0]);
  EXPECT_EQ(6U, observation.inspected_joint_count);
  EXPECT_EQ(1, transport.connect_calls);
  EXPECT_EQ(1, transport.error_warning_read_calls);
  EXPECT_EQ(1, transport.servo_debug_read_calls);
  EXPECT_EQ(0, transport.clear_error_calls);

  xarm_api::close_driver_transport(
    transport, xarm_api::DriverAccessPolicy(true), observation.connected());

  EXPECT_EQ(0, transport.set_pose_mode_calls);
  EXPECT_EQ(1, transport.disconnect_calls);
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
  EXPECT_TRUE(observation.automatic_fault_clear_attempted[0]);
  EXPECT_TRUE(observation.automatic_fault_clear_attempted[2]);
  EXPECT_EQ(2, transport.clear_error_calls);

  xarm_api::close_driver_transport(
    transport, xarm_api::DriverAccessPolicy(false), observation.connected());

  EXPECT_EQ(1, transport.set_pose_mode_calls);
  EXPECT_EQ(1, transport.disconnect_calls);
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
  EXPECT_EQ(-2, observation.connect_result);
  EXPECT_EQ(1, transport.connect_calls);
  EXPECT_EQ(0, transport.error_warning_read_calls);
  EXPECT_EQ(0, transport.servo_debug_read_calls);
  EXPECT_EQ(0, transport.clear_error_calls);
  EXPECT_TRUE(observation.failed_connection_cleanup_performed);
  EXPECT_EQ(0, transport.set_pose_mode_calls);
  EXPECT_EQ(1, transport.disconnect_calls);
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
  EXPECT_EQ(-7, observation.servo_debug_result);
  EXPECT_EQ(0, transport.clear_error_calls);
}
