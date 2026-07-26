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

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "xarm_api/supervised_driver_session.h"

namespace
{
struct FakeTransportState
{
  int connect_result = 0;
  int identity_read_result = 0;
  int error_warning_result = 0;
  int servo_debug_result = 0;
  int joint_read_result = 0;
  int joint_write_result = 0;
  int clear_error_result = 0;
  int clear_warning_result = 0;
  int set_motion_enabled_result = 0;
  int set_mode_result = 0;
  int set_state_result = 0;
  int shutdown_controller_result = 0;
  xarm_api::DriverRobotIdentity identity{6, 9};
  std::array<int, xarm_api::kDriverErrorWarningWords> error_warning{{0, 0}};
  std::array<int, xarm_api::kDriverServoDebugWords> servo_debug{{0}};
  std::array<float, xarm_api::kSupervisedDriverMaximumJoints>
  joint_positions{{0.0F}};
  std::array<float, xarm_api::kSupervisedDriverMaximumJoints>
  joint_velocities{{0.0F}};
  std::array<float, xarm_api::kSupervisedDriverMaximumJoints>
  last_joint_command{{0.0F}};
  int connect_calls = 0;
  int identity_read_calls = 0;
  int error_warning_read_calls = 0;
  int servo_debug_read_calls = 0;
  int joint_read_calls = 0;
  int joint_write_calls = 0;
  int clear_error_calls = 0;
  int clear_warning_calls = 0;
  int set_motion_enabled_calls = 0;
  int set_mode_calls = 0;
  int set_state_calls = 0;
  int shutdown_controller_calls = 0;
  int release_callbacks_calls = 0;
  int disconnect_calls = 0;
  bool block_joint_read = false;
  bool joint_read_entered = false;
  bool release_joint_read = false;
  bool connected = false;
  bool last_motion_enabled = false;
  int last_servo_id = -1;
  int last_mode = -1;
  int last_state = -1;
  std::vector<std::string> events;
  std::mutex joint_read_mutex;
  std::condition_variable joint_read_condition;
};

class FakeSupervisedTransport final
  : public xarm_api::SupervisedDriverTransport
{
public:
  explicit FakeSupervisedTransport(
    std::shared_ptr<FakeTransportState> state)
  : state_(std::move(state))
  {
  }

  int connect() override
  {
    ++state_->connect_calls;
    state_->events.push_back("connect");
    state_->connected = state_->connect_result == 0;
    if (state_->connected && connection_callback_) {
      connection_callback_(true, true);
    }
    return state_->connect_result;
  }

  int read_robot_identity(
    xarm_api::DriverRobotIdentity & identity) override
  {
    ++state_->identity_read_calls;
    state_->events.push_back("read_robot_identity");
    identity = state_->identity;
    return state_->identity_read_result;
  }

  int read_error_warning(
    std::array<int, xarm_api::kDriverErrorWarningWords> &
    error_warning) override
  {
    ++state_->error_warning_read_calls;
    state_->events.push_back("read_error_warning");
    error_warning = state_->error_warning;
    return state_->error_warning_result;
  }

  int read_servo_debug(
    std::array<int, xarm_api::kDriverServoDebugWords> &
    servo_debug) override
  {
    ++state_->servo_debug_read_calls;
    state_->events.push_back("read_servo_debug");
    servo_debug = state_->servo_debug;
    return state_->servo_debug_result;
  }

  int clear_error() override
  {
    ++state_->clear_error_calls;
    state_->events.push_back("clear_error");
    return state_->clear_error_result;
  }

  int clear_warning() override
  {
    ++state_->clear_warning_calls;
    state_->events.push_back("clear_warning");
    return state_->clear_warning_result;
  }

  int set_motion_enabled(bool enabled, int servo_id) override
  {
    ++state_->set_motion_enabled_calls;
    state_->last_motion_enabled = enabled;
    state_->last_servo_id = servo_id;
    state_->events.push_back(
      enabled ? "enable_motion" : "disable_motion");
    return state_->set_motion_enabled_result;
  }

  int set_mode(int mode) override
  {
    ++state_->set_mode_calls;
    state_->last_mode = mode;
    state_->events.push_back("set_mode");
    return state_->set_mode_result;
  }

  int set_state(int state) override
  {
    ++state_->set_state_calls;
    state_->last_state = state;
    state_->events.push_back("set_state");
    return state_->set_state_result;
  }

  int set_pose_mode() override
  {
    state_->events.push_back("set_pose_mode");
    return 0;
  }

  void release_callbacks() override
  {
    ++state_->release_callbacks_calls;
    state_->events.push_back("release_callbacks");
    report_callback_ = {};
    connection_callback_ = {};
  }

  void disconnect() override
  {
    ++state_->disconnect_calls;
    state_->events.push_back("disconnect");
    state_->connected = false;
  }

  void set_report_callback(ReportCallback callback) override
  {
    report_callback_ = std::move(callback);
  }

  void set_connection_callback(ConnectionCallback callback) override
  {
    connection_callback_ = std::move(callback);
  }

  bool connected() const override
  {
    return state_->connected;
  }

  int read_joint_state(
    std::array<float, xarm_api::kSupervisedDriverMaximumJoints> &
    positions,
    std::array<float, xarm_api::kSupervisedDriverMaximumJoints> &
    velocities,
    std::size_t) override
  {
    ++state_->joint_read_calls;
    state_->events.push_back("read_joint_state");
    {
      std::unique_lock<std::mutex> lock(state_->joint_read_mutex);
      if (state_->block_joint_read) {
        state_->joint_read_entered = true;
        state_->joint_read_condition.notify_all();
        state_->joint_read_condition.wait(
          lock, [this]() {return state_->release_joint_read;});
      }
    }
    positions = state_->joint_positions;
    velocities = state_->joint_velocities;
    return state_->joint_read_result;
  }

  int write_joint_position_command(
    const std::array<float, xarm_api::kSupervisedDriverMaximumJoints> &
    positions,
    std::size_t) override
  {
    ++state_->joint_write_calls;
    state_->events.push_back("write_joint_position");
    state_->last_joint_command = positions;
    return state_->joint_write_result;
  }

  int shutdown_controller() override
  {
    ++state_->shutdown_controller_calls;
    state_->events.push_back("shutdown_controller");
    return state_->shutdown_controller_result;
  }

  void emit_report(const xarm_api::SupervisedDriverReport & report)
  {
    if (report_callback_) {
      report_callback_(report);
    }
  }

  void emit_connection(bool connected, bool report_connected)
  {
    state_->connected = connected;
    if (connection_callback_) {
      connection_callback_(connected, report_connected);
    }
  }

private:
  std::shared_ptr<FakeTransportState> state_;
  ReportCallback report_callback_;
  ConnectionCallback connection_callback_;
};

xarm_api::SupervisedDriverSessionConfig make_config()
{
  xarm_api::SupervisedDriverSessionConfig config;
  config.robot_address = "fake.transport";
  config.joint_count = 6;
  config.identity_expectation = {6, 9};
  config.observation_lease_ns = 10000000000LL;
  config.joint_state_lease_ns = 10000000000LL;
  config.io_period_ns = 1000000LL;
  config.transport_loss_timeout_ns = 20000000000LL;
  return config;
}

std::int64_t future_deadline()
{
  return std::numeric_limits<std::int64_t>::max();
}

struct SessionRig
{
  std::shared_ptr<FakeTransportState> state;
  FakeSupervisedTransport * transport = nullptr;
  std::unique_ptr<xarm_api::SupervisedDriverSession> session;
};

SessionRig make_rig(
  xarm_api::SupervisedDriverSessionConfig config = make_config())
{
  SessionRig rig;
  rig.state = std::make_shared<FakeTransportState>();
  std::unique_ptr<FakeSupervisedTransport> transport(
    new FakeSupervisedTransport(rig.state));
  rig.transport = transport.get();
  rig.session.reset(new xarm_api::SupervisedDriverSession(
      std::move(config), std::move(transport), false));
  return rig;
}

void emit_powered_pose_samples(SessionRig & rig)
{
  xarm_api::SupervisedDriverReport report;
  report.state = 2;
  report.mode = 1;
  report.brake_mask = 63;
  report.servo_enable_mask = 63;
  report.error_code = 0;
  report.warning_code = 0;
  for (std::size_t index = 0; index < 6; ++index) {
    report.joint_positions[index] = rig.state->joint_positions[index];
  }
  for (std::size_t sample = 0; sample < 3; ++sample) {
    rig.transport->emit_report(report);
    rig.session->service_io_once();
  }
}

void initialize_powered_pose(SessionRig & rig)
{
  emit_powered_pose_samples(rig);
  ASSERT_TRUE(rig.session->observe(0).position_valid);
}
}  // namespace

TEST(SupervisedDriverSession, StartupObservesWithoutSendingCommands)
{
  auto rig = make_rig();

  EXPECT_TRUE(rig.session->ready());
  const auto observation = rig.session->observe(0);
  EXPECT_TRUE(observation.connected);
  EXPECT_TRUE(observation.report_connected);
  EXPECT_TRUE(observation.identity_verified);
  EXPECT_FALSE(observation.process_restart_required);
  EXPECT_FALSE(observation.report_received);
  EXPECT_FALSE(observation.position_valid);
  EXPECT_EQ(6, observation.identity.axis);
  EXPECT_EQ(9, observation.identity.device_type);
  EXPECT_EQ(0U, observation.report_sample_count);
  EXPECT_EQ(0U, observation.joint_read_attempt_count);
  EXPECT_EQ(0U, observation.joint_read_success_count);
  EXPECT_EQ(0U, observation.joint_write_attempt_count);
  EXPECT_EQ(0U, observation.lifecycle_command_attempt_count);
  EXPECT_EQ(1, rig.state->connect_calls);
  EXPECT_EQ(1, rig.state->identity_read_calls);
  EXPECT_EQ(1, rig.state->error_warning_read_calls);
  EXPECT_EQ(1, rig.state->servo_debug_read_calls);
  EXPECT_EQ(0, rig.state->clear_error_calls);
  EXPECT_EQ(0, rig.state->clear_warning_calls);
  EXPECT_EQ(0, rig.state->set_motion_enabled_calls);
  EXPECT_EQ(0, rig.state->set_mode_calls);
  EXPECT_EQ(0, rig.state->set_state_calls);
  EXPECT_EQ(0, rig.state->joint_write_calls);

  rig.session->close();
  rig.session->close();
  EXPECT_EQ(1, rig.state->release_callbacks_calls);
  EXPECT_EQ(1, rig.state->disconnect_calls);
  EXPECT_EQ(0, std::count(
      rig.state->events.begin(), rig.state->events.end(),
      "set_pose_mode"));
  const auto release_position = std::find(
    rig.state->events.begin(), rig.state->events.end(), "release_callbacks");
  const auto disconnect_position = std::find(
    rig.state->events.begin(), rig.state->events.end(), "disconnect");
  ASSERT_NE(rig.state->events.end(), release_position);
  ASSERT_NE(rig.state->events.end(), disconnect_position);
  EXPECT_LT(release_position, disconnect_position);
}

TEST(SupervisedDriverSession, ReportAndJointPollPopulateIndependentCaches)
{
  auto rig = make_rig();
  xarm_api::SupervisedDriverReport report;
  report.state = 4;
  report.mode = 0;
  report.command_count = 23;
  report.brake_mask = 0;
  report.servo_enable_mask = 0;
  report.error_code = 2;
  report.warning_code = 0;
  rig.transport->emit_report(report);
  for (std::size_t index = 0; index < 6; ++index) {
    rig.state->joint_positions[index] =
      static_cast<float>(index) + 0.25F;
    rig.state->joint_velocities[index] =
      static_cast<float>(index) * 0.1F;
  }

  rig.session->service_io_once();

  const auto observation = rig.session->observe(0);
  EXPECT_TRUE(observation.report_received);
  EXPECT_FALSE(observation.position_valid);
  EXPECT_GT(observation.source_timestamp_ns, 0);
  EXPECT_GT(observation.fresh_until_ns, observation.source_timestamp_ns);
  EXPECT_EQ(4, observation.report.state);
  EXPECT_EQ(0, observation.report.mode);
  EXPECT_EQ(2, observation.report.error_code);
  EXPECT_EQ(1U, observation.report_sample_count);
  EXPECT_EQ(1U, observation.joint_read_attempt_count);
  EXPECT_EQ(1U, observation.joint_read_success_count);
  EXPECT_EQ(1, rig.state->joint_read_calls);

  xarm_api::SupervisedDriverJointState joints;
  EXPECT_FALSE(rig.session->read_joint_state(joints));
}

TEST(SupervisedDriverSession, ReadCountersExcludeAnInFlightSdkRead)
{
  auto rig = make_rig();
  {
    std::lock_guard<std::mutex> lock(rig.state->joint_read_mutex);
    rig.state->block_joint_read = true;
  }
  std::thread service_thread(
    [&rig]() {rig.session->service_io_once();});
  bool entered = false;
  {
    std::unique_lock<std::mutex> lock(rig.state->joint_read_mutex);
    entered = rig.state->joint_read_condition.wait_for(
      lock, std::chrono::seconds(1),
      [&rig]() {return rig.state->joint_read_entered;});
  }
  if (!entered) {
    {
      std::lock_guard<std::mutex> lock(rig.state->joint_read_mutex);
      rig.state->release_joint_read = true;
    }
    rig.state->joint_read_condition.notify_all();
    service_thread.join();
    FAIL() << "fake SDK read did not start";
  }

  auto observation = rig.session->observe(0);
  EXPECT_EQ(0U, observation.joint_read_attempt_count);
  EXPECT_EQ(0U, observation.joint_read_success_count);
  {
    std::lock_guard<std::mutex> lock(rig.state->joint_read_mutex);
    rig.state->release_joint_read = true;
  }
  rig.state->joint_read_condition.notify_all();
  service_thread.join();

  observation = rig.session->observe(0);
  EXPECT_EQ(1U, observation.joint_read_attempt_count);
  EXPECT_EQ(1U, observation.joint_read_success_count);
}

TEST(SupervisedDriverSession, PoseInitializesOnlyAfterEnabledSourcesAgree)
{
  auto rig = make_rig();
  xarm_api::SupervisedDriverReport report;
  report.state = 4;
  report.mode = 0;
  report.brake_mask = 63;
  report.servo_enable_mask = 63;
  report.error_code = 0;
  report.warning_code = 0;
  for (std::size_t index = 0; index < 6; ++index) {
    const float position = static_cast<float>(index) + 0.25F;
    rig.state->joint_positions[index] = position;
    rig.state->joint_velocities[index] =
      static_cast<float>(index) * 0.1F;
    report.joint_positions[index] = position;
  }
  rig.transport->emit_report(report);

  rig.session->service_io_once();
  rig.transport->emit_report(report);
  rig.session->service_io_once();
  EXPECT_FALSE(rig.session->observe(0).position_valid);
  xarm_api::SupervisedDriverJointState joints;
  EXPECT_FALSE(rig.session->read_joint_state(joints));

  rig.transport->emit_report(report);
  rig.session->service_io_once();
  EXPECT_TRUE(rig.session->observe(0).position_valid);
  ASSERT_TRUE(rig.session->read_joint_state(joints));
  EXPECT_EQ(6U, joints.joint_count);
  EXPECT_GT(joints.generation, 0U);
  for (std::size_t index = 0; index < 6; ++index) {
    EXPECT_DOUBLE_EQ(
      static_cast<double>(rig.state->joint_positions[index]),
      joints.positions[index]);
    EXPECT_DOUBLE_EQ(
      static_cast<double>(rig.state->joint_velocities[index]),
      joints.velocities[index]);
  }
}

TEST(SupervisedDriverSession, DisabledOrDivergentPlaceholdersNeverInitialize)
{
  auto rig = make_rig();
  xarm_api::SupervisedDriverReport report;
  report.state = 4;
  report.mode = 0;
  report.brake_mask = 0;
  report.servo_enable_mask = 0;
  report.error_code = 0;
  for (int sample = 0; sample < 5; ++sample) {
    rig.transport->emit_report(report);
    rig.session->service_io_once();
  }
  EXPECT_FALSE(rig.session->observe(0).position_valid);

  report.brake_mask = 63;
  report.servo_enable_mask = 63;
  report.joint_positions[0] = 0.1;
  for (int sample = 0; sample < 5; ++sample) {
    rig.transport->emit_report(report);
    rig.session->service_io_once();
  }
  EXPECT_FALSE(rig.session->observe(0).position_valid);
}

TEST(SupervisedDriverSession, PoweredAgreeingExactZeroPoseIsValid)
{
  auto rig = make_rig();

  initialize_powered_pose(rig);

  xarm_api::SupervisedDriverJointState joints;
  ASSERT_TRUE(rig.session->read_joint_state(joints));
  for (std::size_t index = 0; index < 6; ++index) {
    EXPECT_DOUBLE_EQ(0.0, joints.positions[index]);
  }
}

TEST(SupervisedDriverSession, LosingPoweredStateInvalidatesPoseAndCommandGate)
{
  auto rig = make_rig();
  initialize_powered_pose(rig);
  ASSERT_TRUE(rig.session->set_command_gate(true, future_deadline()));

  xarm_api::SupervisedDriverReport report;
  report.state = 4;
  report.mode = 0;
  report.brake_mask = 0;
  report.servo_enable_mask = 0;
  report.error_code = 2;
  report.warning_code = 0;
  rig.transport->emit_report(report);

  std::array<double, xarm_api::kSupervisedDriverMaximumJoints>
  command{{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0}};
  xarm_api::SupervisedDriverJointState joints;
  EXPECT_FALSE(rig.session->observe(0).position_valid);
  EXPECT_FALSE(rig.session->read_joint_state(joints));
  EXPECT_FALSE(rig.session->submit_joint_position_command(command, 6));
}

TEST(SupervisedDriverSession, ReportTransportLossRequiresFreshProcess)
{
  auto rig = make_rig();
  initialize_powered_pose(rig);
  ASSERT_TRUE(rig.session->set_command_gate(true, future_deadline()));

  rig.transport->emit_connection(true, false);

  std::array<double, xarm_api::kSupervisedDriverMaximumJoints>
  command{{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0}};
  const auto observation = rig.session->observe(0);
  EXPECT_TRUE(observation.connected);
  EXPECT_FALSE(observation.report_connected);
  EXPECT_TRUE(observation.process_restart_required);
  EXPECT_FALSE(observation.report_received);
  EXPECT_FALSE(observation.position_valid);
  EXPECT_FALSE(rig.session->submit_joint_position_command(command, 6));

  rig.transport->emit_connection(true, true);
  EXPECT_FALSE(rig.session->set_command_gate(true, future_deadline()));
  emit_powered_pose_samples(rig);
  EXPECT_FALSE(rig.session->observe(0).position_valid);
  EXPECT_FALSE(rig.session->set_command_gate(true, future_deadline()));
  const auto lifecycle_result = rig.session->execute_lifecycle(
    xarm_api::DriverLifecycleCommand{
    xarm_api::DriverLifecycleCommandKind::kClearError, 0, 0});
  EXPECT_FALSE(lifecycle_result.permitted);
  EXPECT_FALSE(lifecycle_result.attempted);
  EXPECT_STREQ("process_restart_required", lifecycle_result.reason);
  EXPECT_EQ(0, rig.state->clear_error_calls);
  EXPECT_FALSE(rig.session->ready());
}

TEST(
  SupervisedDriverSession,
  ProlongedSdkStalenessRequiresFreshProcessWithoutDisconnectCallback)
{
  auto config = make_config();
  config.observation_lease_ns = 100000000LL;
  config.joint_state_lease_ns = 100000000LL;
  config.transport_loss_timeout_ns = 250000000LL;
  auto rig = make_rig(std::move(config));
  initialize_powered_pose(rig);
  ASSERT_TRUE(rig.session->set_command_gate(true, future_deadline()));

  rig.state->joint_read_result = -1;
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  rig.session->service_io_once();

  const auto observation = rig.session->observe(0);
  EXPECT_TRUE(rig.state->connected);
  EXPECT_FALSE(observation.connected);
  EXPECT_FALSE(observation.report_connected);
  EXPECT_FALSE(observation.identity_verified);
  EXPECT_TRUE(observation.process_restart_required);
  EXPECT_FALSE(observation.report_received);
  EXPECT_FALSE(observation.position_valid);
  EXPECT_FALSE(observation.command_gate_open);
  EXPECT_EQ(-1, observation.last_joint_read_return_code);
  EXPECT_FALSE(rig.session->ready());

  rig.state->joint_read_result = 0;
  rig.transport->emit_connection(true, true);
  emit_powered_pose_samples(rig);
  EXPECT_TRUE(rig.state->connected);
  EXPECT_FALSE(rig.session->ready());
  EXPECT_FALSE(rig.session->observe(0).position_valid);
  EXPECT_EQ(4, rig.state->joint_read_calls);
}

TEST(
  SupervisedDriverSession,
  TransientJointReadFailureFencesWithoutTerminatingSession)
{
  auto rig = make_rig();
  initialize_powered_pose(rig);
  ASSERT_TRUE(rig.session->set_command_gate(true, future_deadline()));

  rig.state->joint_read_result = -1;
  rig.session->service_io_once();

  auto observation = rig.session->observe(0);
  EXPECT_TRUE(observation.connected);
  EXPECT_TRUE(observation.report_connected);
  EXPECT_FALSE(observation.process_restart_required);
  EXPECT_FALSE(observation.position_valid);
  EXPECT_FALSE(observation.command_gate_open);
  EXPECT_TRUE(rig.session->ready());

  rig.state->joint_read_result = 0;
  emit_powered_pose_samples(rig);
  observation = rig.session->observe(0);
  EXPECT_FALSE(observation.process_restart_required);
  EXPECT_TRUE(observation.position_valid);
  EXPECT_TRUE(rig.session->ready());
}

TEST(
  SupervisedDriverSession,
  ReportStalenessPreservesControlTransportDiagnostic)
{
  auto config = make_config();
  config.observation_lease_ns = 100000000LL;
  config.joint_state_lease_ns = 100000000LL;
  config.transport_loss_timeout_ns = 250000000LL;
  auto rig = make_rig(std::move(config));
  initialize_powered_pose(rig);

  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  rig.session->service_io_once();

  const auto observation = rig.session->observe(0);
  EXPECT_TRUE(observation.connected);
  EXPECT_FALSE(observation.report_connected);
  EXPECT_TRUE(observation.process_restart_required);
  EXPECT_FALSE(observation.identity_verified);
  EXPECT_FALSE(observation.report_received);
  EXPECT_FALSE(observation.position_valid);
}

TEST(SupervisedDriverSession, LifecycleExecutesOneExactPrimitiveAndFencesMotion)
{
  auto rig = make_rig();
  initialize_powered_pose(rig);
  ASSERT_TRUE(rig.session->set_command_gate(true, future_deadline()));
  std::array<double, xarm_api::kSupervisedDriverMaximumJoints>
  command{{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0}};
  ASSERT_TRUE(rig.session->submit_joint_position_command(command, 6));

  const auto result = rig.session->execute_lifecycle(
    xarm_api::DriverLifecycleCommand{
    xarm_api::DriverLifecycleCommandKind::kSetMotionEnabled, 1, 8});

  EXPECT_TRUE(result.permitted);
  EXPECT_TRUE(result.attempted);
  EXPECT_EQ(0, result.return_code);
  EXPECT_EQ(1, rig.state->set_motion_enabled_calls);
  EXPECT_EQ(
    1U,
    rig.session->observe(0).lifecycle_command_attempt_count);
  EXPECT_TRUE(rig.state->last_motion_enabled);
  EXPECT_EQ(8, rig.state->last_servo_id);
  EXPECT_FALSE(rig.session->submit_joint_position_command(command, 6));
  rig.session->service_io_once();
  EXPECT_EQ(0, rig.state->joint_write_calls);
}

TEST(SupervisedDriverSession, CommandGateSendsOnlyLatestCoherentCommand)
{
  auto rig = make_rig();
  initialize_powered_pose(rig);
  std::array<double, xarm_api::kSupervisedDriverMaximumJoints>
  first{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0}};
  std::array<double, xarm_api::kSupervisedDriverMaximumJoints>
  second{{2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 0.0}};

  EXPECT_FALSE(rig.session->submit_joint_position_command(first, 6));
  ASSERT_TRUE(rig.session->set_command_gate(true, future_deadline()));
  ASSERT_TRUE(rig.session->submit_joint_position_command(first, 6));
  ASSERT_TRUE(rig.session->submit_joint_position_command(second, 6));

  rig.session->service_io_once();

  ASSERT_EQ(1, rig.state->joint_write_calls);
  EXPECT_EQ(1U, rig.session->observe(0).joint_write_attempt_count);
  for (std::size_t index = 0; index < 6; ++index) {
    EXPECT_FLOAT_EQ(2.0F, rig.state->last_joint_command[index]);
  }
  rig.session->service_io_once();
  EXPECT_EQ(1, rig.state->joint_write_calls);

  EXPECT_FALSE(rig.session->set_command_gate(false, 0));
  EXPECT_FALSE(rig.session->submit_joint_position_command(first, 6));
  ASSERT_TRUE(rig.session->set_command_gate(true, future_deadline()));
  rig.session->service_io_once();
  EXPECT_EQ(1, rig.state->joint_write_calls);
}

TEST(SupervisedDriverSession, InvalidOrExpiredCommandsNeverReachSdk)
{
  auto rig = make_rig();
  initialize_powered_pose(rig);
  std::array<double, xarm_api::kSupervisedDriverMaximumJoints>
  command{{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0}};

  EXPECT_FALSE(rig.session->set_command_gate(true, 1));
  EXPECT_FALSE(rig.session->submit_joint_position_command(command, 6));
  ASSERT_TRUE(rig.session->set_command_gate(true, future_deadline()));
  EXPECT_FALSE(rig.session->submit_joint_position_command(command, 5));
  command[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(rig.session->submit_joint_position_command(command, 6));
  rig.session->service_io_once();
  EXPECT_EQ(0, rig.state->joint_write_calls);
}

TEST(SupervisedDriverSession, CommandGateRequiresCommandCapableServoState)
{
  auto rig = make_rig();
  initialize_powered_pose(rig);
  xarm_api::SupervisedDriverReport report;
  report.state = 4;
  report.mode = 1;
  report.brake_mask = 63;
  report.servo_enable_mask = 63;
  report.error_code = 0;
  report.warning_code = 0;
  for (std::size_t index = 0; index < 6; ++index) {
    report.joint_positions[index] = rig.state->joint_positions[index];
  }

  rig.transport->emit_report(report);
  EXPECT_TRUE(rig.session->observe(0).position_valid);
  EXPECT_FALSE(rig.session->set_command_gate(true, future_deadline()));

  report.state = 0;
  report.mode = 1;
  rig.transport->emit_report(report);
  EXPECT_TRUE(rig.session->observe(0).position_valid);
  EXPECT_FALSE(rig.session->set_command_gate(true, future_deadline()));

  report.state = 2;
  rig.transport->emit_report(report);
  EXPECT_TRUE(rig.session->set_command_gate(true, future_deadline()));

  report.state = 1;
  rig.transport->emit_report(report);
  EXPECT_TRUE(rig.session->set_command_gate(true, future_deadline()));

  report.state = 3;
  rig.transport->emit_report(report);
  std::array<double, xarm_api::kSupervisedDriverMaximumJoints>
  command{{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0}};
  EXPECT_FALSE(rig.session->submit_joint_position_command(command, 6));
}

TEST(
  SupervisedDriverSession,
  ExplicitCloseWaitsForInflightSdkIoAfterAsynchronousFaultClosure)
{
  auto rig = make_rig();
  initialize_powered_pose(rig);
  ASSERT_TRUE(rig.session->set_command_gate(true, future_deadline()));
  {
    std::lock_guard<std::mutex> lock(rig.state->joint_read_mutex);
    rig.state->block_joint_read = true;
    rig.state->joint_read_entered = false;
    rig.state->release_joint_read = false;
  }
  std::thread service_thread(
    [&rig]() {rig.session->service_io_once();});
  bool read_entered = false;
  {
    std::unique_lock<std::mutex> lock(rig.state->joint_read_mutex);
    read_entered = rig.state->joint_read_condition.wait_for(
      lock, std::chrono::seconds(1),
      [&rig]() {return rig.state->joint_read_entered;});
  }
  if (!read_entered) {
    {
      std::lock_guard<std::mutex> lock(rig.state->joint_read_mutex);
      rig.state->release_joint_read = true;
    }
    rig.state->joint_read_condition.notify_all();
    service_thread.join();
    FAIL() << "fake SDK read did not start";
  }

  xarm_api::SupervisedDriverReport unsafe_report;
  unsafe_report.state = 3;
  unsafe_report.mode = 1;
  unsafe_report.brake_mask = 63;
  unsafe_report.servo_enable_mask = 63;
  unsafe_report.error_code = 0;
  unsafe_report.warning_code = 0;
  for (std::size_t index = 0; index < 6; ++index) {
    unsafe_report.joint_positions[index] =
      rig.state->joint_positions[index];
  }
  rig.transport->emit_report(unsafe_report);
  ASSERT_FALSE(rig.session->observe(0).command_gate_open);

  std::promise<void> close_completed;
  auto close_completion = close_completed.get_future();
  std::thread close_thread(
    [&rig, &close_completed]() {
      static_cast<void>(rig.session->set_command_gate(false, 0));
      close_completed.set_value();
    });
  EXPECT_EQ(
    std::future_status::timeout,
    close_completion.wait_for(std::chrono::milliseconds(50)));

  {
    std::lock_guard<std::mutex> lock(rig.state->joint_read_mutex);
    rig.state->release_joint_read = true;
  }
  rig.state->joint_read_condition.notify_all();
  service_thread.join();
  EXPECT_EQ(
    std::future_status::ready,
    close_completion.wait_for(std::chrono::seconds(1)));
  close_thread.join();
}

TEST(SupervisedDriverSession, StaleRichReportCannotOpenCommandGate)
{
  auto config = make_config();
  config.observation_lease_ns = 1;
  auto rig = make_rig(std::move(config));
  initialize_powered_pose(rig);

  EXPECT_FALSE(rig.session->set_command_gate(true, future_deadline()));
  std::array<double, xarm_api::kSupervisedDriverMaximumJoints>
  command{{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0}};
  EXPECT_FALSE(rig.session->submit_joint_position_command(command, 6));
  rig.session->service_io_once();
  EXPECT_EQ(0, rig.state->joint_write_calls);
}

TEST(SupervisedDriverSession, WriteFailureClosesGate)
{
  auto rig = make_rig();
  initialize_powered_pose(rig);
  rig.state->joint_write_result = -7;
  std::array<double, xarm_api::kSupervisedDriverMaximumJoints>
  command{{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0}};
  ASSERT_TRUE(rig.session->set_command_gate(true, future_deadline()));
  ASSERT_TRUE(rig.session->submit_joint_position_command(command, 6));

  rig.session->service_io_once();

  EXPECT_EQ(1, rig.state->joint_write_calls);
  EXPECT_FALSE(rig.session->submit_joint_position_command(command, 6));
  EXPECT_EQ(-7, rig.session->observe(0).last_joint_write_return_code);
}

TEST(SupervisedDriverSession, ConnectionLossClosesGateWithoutStaleReplay)
{
  auto rig = make_rig();
  initialize_powered_pose(rig);
  std::array<double, xarm_api::kSupervisedDriverMaximumJoints>
  command{{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0}};
  ASSERT_TRUE(rig.session->set_command_gate(true, future_deadline()));
  ASSERT_TRUE(rig.session->submit_joint_position_command(command, 6));

  rig.transport->emit_connection(false, false);
  rig.transport->emit_connection(true, true);
  rig.session->service_io_once();

  EXPECT_EQ(0, rig.state->joint_write_calls);
  EXPECT_FALSE(rig.session->submit_joint_position_command(command, 6));
  EXPECT_EQ(1, rig.state->identity_read_calls);
  EXPECT_FALSE(rig.session->observe(0).identity_verified);
  EXPECT_TRUE(rig.session->observe(0).process_restart_required);
  EXPECT_FALSE(rig.session->ready());
}

TEST(SupervisedDriverSession, ReconnectNeverReusesFaultedSdkSession)
{
  auto rig = make_rig();
  rig.transport->emit_connection(false, false);
  rig.transport->emit_connection(true, true);

  rig.session->service_io_once();

  EXPECT_FALSE(rig.session->ready());
  EXPECT_FALSE(rig.session->observe(0).identity_verified);
  EXPECT_TRUE(rig.session->observe(0).process_restart_required);
  EXPECT_TRUE(rig.state->connected);
  EXPECT_EQ(1, rig.state->identity_read_calls);
  EXPECT_EQ(0, rig.state->disconnect_calls);
  rig.transport->emit_connection(true, true);
  rig.session->service_io_once();
  EXPECT_EQ(1, rig.state->identity_read_calls);
}

TEST(SupervisedDriverSession, IdentityMismatchFailsClosedAndCleansTransport)
{
  auto state = std::make_shared<FakeTransportState>();
  state->identity = {7, 9};
  std::unique_ptr<FakeSupervisedTransport> transport(
    new FakeSupervisedTransport(state));

  EXPECT_THROW(
    xarm_api::SupervisedDriverSession(
      make_config(), std::move(transport), false),
    std::runtime_error);

  EXPECT_EQ(1, state->connect_calls);
  EXPECT_EQ(1, state->identity_read_calls);
  EXPECT_EQ(0, state->error_warning_read_calls);
  EXPECT_EQ(0, state->servo_debug_read_calls);
  EXPECT_EQ(0, state->clear_error_calls);
  EXPECT_EQ(0, state->joint_write_calls);
  EXPECT_EQ(1, state->release_callbacks_calls);
  EXPECT_EQ(1, state->disconnect_calls);
}

TEST(SupervisedDriverSession, FailedConnectCleansTransportExactlyOnce)
{
  auto state = std::make_shared<FakeTransportState>();
  state->connect_result = -1;
  std::unique_ptr<FakeSupervisedTransport> transport(
    new FakeSupervisedTransport(state));

  EXPECT_THROW(
    xarm_api::SupervisedDriverSession(
      make_config(), std::move(transport), false),
    std::runtime_error);

  EXPECT_EQ(1, state->connect_calls);
  EXPECT_EQ(0, state->identity_read_calls);
  EXPECT_EQ(1, state->release_callbacks_calls);
  EXPECT_EQ(1, state->disconnect_calls);
  EXPECT_EQ(0, state->shutdown_controller_calls);
}

TEST(
  SupervisedDriverSession,
  ControllerShutdownAttemptsExactlyOnceAndFencesSession)
{
  auto config = make_config();
  config.shutdown_stationary_dwell_ns = 1;
  auto rig = make_rig(std::move(config));
  initialize_powered_pose(rig);

  xarm_api::SupervisedDriverReport disabled_report;
  disabled_report.state = 4;
  disabled_report.mode = 1;
  disabled_report.brake_mask = 0;
  disabled_report.servo_enable_mask = 0;
  disabled_report.error_code = 2;
  disabled_report.warning_code = 0;
  for (std::size_t index = 0; index < 6; ++index) {
    disabled_report.joint_positions[index] =
      rig.state->joint_positions[index];
  }
  rig.transport->emit_report(disabled_report);
  rig.session->service_io_once();
  rig.session->service_io_once();
  const auto before = rig.session->observe(0);
  ASSERT_TRUE(before.position_ever_initialized);
  ASSERT_TRUE(before.stationary);

  const auto result = rig.session->shutdown_controller();
  EXPECT_TRUE(result.permitted);
  EXPECT_TRUE(result.attempted);
  EXPECT_EQ(0, result.return_code);
  EXPECT_STREQ(result.reason, "controller_shutdown_vendor_accepted");
  EXPECT_TRUE(result.process_restart_required);
  EXPECT_EQ(1, rig.state->shutdown_controller_calls);
  EXPECT_EQ(
    1U,
    rig.session->observe(0).shutdown_controller_attempt_count);
  EXPECT_TRUE(rig.session->observe(0).process_restart_required);

  const auto repeated = rig.session->shutdown_controller();
  EXPECT_FALSE(repeated.permitted);
  EXPECT_FALSE(repeated.attempted);
  EXPECT_EQ(1, rig.state->shutdown_controller_calls);
}

TEST(SupervisedDriverSession, CloseFencesLifecycleWaitingOnSdk)
{
  auto rig = make_rig();
  {
    std::lock_guard<std::mutex> lock(rig.state->joint_read_mutex);
    rig.state->block_joint_read = true;
  }
  std::thread service_thread(
    [&rig]() {rig.session->service_io_once();});
  bool read_entered = false;
  {
    std::unique_lock<std::mutex> lock(rig.state->joint_read_mutex);
    read_entered = rig.state->joint_read_condition.wait_for(
      lock, std::chrono::seconds(1),
      [&rig]() {return rig.state->joint_read_entered;});
  }
  if (!read_entered) {
    {
      std::lock_guard<std::mutex> lock(rig.state->joint_read_mutex);
      rig.state->release_joint_read = true;
    }
    rig.state->joint_read_condition.notify_all();
    service_thread.join();
    FAIL() << "fake SDK read did not start";
  }

  std::atomic<bool> lifecycle_started{false};
  xarm_api::DriverLifecycleCommandResult lifecycle_result;
  std::thread lifecycle_thread(
    [&rig, &lifecycle_started, &lifecycle_result]()
    {
      lifecycle_started.store(true, std::memory_order_release);
      lifecycle_result = rig.session->execute_lifecycle(
        xarm_api::DriverLifecycleCommand{
      xarm_api::DriverLifecycleCommandKind::kSetState, 0, 0});
    });
  while (!lifecycle_started.load(std::memory_order_acquire)) {
    std::this_thread::yield();
  }

  std::thread close_thread([&rig]() {rig.session->close();});
  const auto close_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (rig.session->ready() &&
    std::chrono::steady_clock::now() < close_deadline)
  {
    std::this_thread::yield();
  }
  const bool close_started = !rig.session->ready();
  {
    std::lock_guard<std::mutex> lock(rig.state->joint_read_mutex);
    rig.state->release_joint_read = true;
  }
  rig.state->joint_read_condition.notify_all();

  service_thread.join();
  lifecycle_thread.join();
  close_thread.join();
  ASSERT_TRUE(close_started);
  EXPECT_FALSE(lifecycle_result.permitted);
  EXPECT_FALSE(lifecycle_result.attempted);
  EXPECT_STREQ(lifecycle_result.reason, "session_closing");
  EXPECT_EQ(rig.state->set_state_calls, 0);
  EXPECT_EQ(
    rig.session->observe(0).lifecycle_command_attempt_count,
    0U);
}
