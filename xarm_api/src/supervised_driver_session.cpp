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

#include "xarm_api/supervised_driver_session.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

#include "xarm/wrapper/xarm_api.h"

namespace
{
std::int64_t saturating_add(
  std::int64_t value, std::int64_t positive_delta)
{
  if (value > std::numeric_limits<std::int64_t>::max() - positive_delta) {
    return std::numeric_limits<std::int64_t>::max();
  }
  return value + positive_delta;
}

bool command_path_state(int state)
{
  // UFACTORY documents feedback state 1 as MOVING and state 2 as READY.
  // State 0 is the standby state requested through set_state(0), not the
  // command-ready feedback reached after that request. Keep the gate closed
  // during that transition and for every stopped/interlocked state.
  return state == 1 || state == 2;
}

class XArmApiSupervisedTransport final
  : public xarm_api::SupervisedDriverTransport
{
public:
  explicit XArmApiSupervisedTransport(
    const xarm_api::SupervisedDriverSessionConfig & config)
  : arm_(new XArmAPI(
        config.robot_address,
        true,   // is_radian
        true,   // do_not_open
        true,   // check_tcp_limit
        true,   // check_joint_limit
        true,   // check_cmdnum_limit
        false,  // check_robot_sn
        true,   // check_is_ready
        true,   // check_is_pause
        0,      // max_callback_thread_count
        512,    // max_cmdnum
        static_cast<int>(config.joint_count),
        false,  // debug
        config.report_type))
  {
    arm_->set_baud_checkset_enable(true);
    arm_->set_checkset_default_baud(1, 2000000);
  }

  ~XArmApiSupervisedTransport() override
  {
    release_callbacks();
    disconnect();
  }

  int connect() override
  {
    closed_.store(false, std::memory_order_release);
    const int result = arm_->connect();
    callback_bridge_->connected.store(
      result == 0 && arm_->is_connected(), std::memory_order_release);
    return result;
  }

  int read_robot_identity(xarm_api::DriverRobotIdentity & identity) override
  {
    if (!arm_->is_connected()) {
      return -1;
    }
    identity.axis = arm_->axis;
    identity.device_type = arm_->device_type;
    return 0;
  }

  int read_error_warning(
    std::array<int, xarm_api::kDriverErrorWarningWords> &
    error_warning) override
  {
    return arm_->get_err_warn_code(error_warning.data());
  }

  int read_servo_debug(
    std::array<int, xarm_api::kDriverServoDebugWords> &
    servo_debug) override
  {
    return arm_->core == nullptr ?
           -1 : arm_->core->servo_get_dbmsg(servo_debug.data());
  }

  int clear_error() override
  {
    return arm_->clean_error();
  }

  int clear_warning() override
  {
    return arm_->clean_warn();
  }

  int set_motion_enabled(bool enabled, int servo_id) override
  {
    return arm_->motion_enable(enabled, servo_id);
  }

  int set_mode(int mode) override
  {
    return arm_->set_mode(mode);
  }

  int set_state(int state) override
  {
    return arm_->set_state(state);
  }

  int set_pose_mode() override
  {
    return arm_->set_mode(XARM_MODE::POSE);
  }

  void release_callbacks() override
  {
    if (callbacks_released_) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(callback_bridge_->mutex);
      callback_bridge_->enabled = false;
      callback_bridge_->report_callback = {};
      callback_bridge_->connection_callback = {};
    }
    arm_->release_connect_changed_callback(true);
    arm_->release_report_data_callback(true);
    callbacks_released_ = true;
  }

  void disconnect() override
  {
    callback_bridge_->connected.store(false, std::memory_order_release);
    if (closed_.exchange(true, std::memory_order_acq_rel)) {
      return;
    }
    arm_->disconnect();
  }

  void set_report_callback(ReportCallback callback) override
  {
    const auto bridge = callback_bridge_;
    {
      std::lock_guard<std::mutex> lock(bridge->mutex);
      bridge->enabled = true;
      bridge->report_callback = std::move(callback);
    }
    callbacks_released_ = false;
    arm_->register_report_data_callback(
      [bridge](XArmReportData * report)
      {
        if (report == nullptr) {
          return;
        }
        xarm_api::SupervisedDriverReport mapped;
        mapped.state = report->state;
        mapped.mode = report->mode;
        mapped.command_count = report->cmdnum;
        mapped.brake_mask = report->mt_brake;
        mapped.servo_enable_mask = report->mt_able;
        mapped.error_code = report->err;
        mapped.warning_code = report->war;
        for (std::size_t index = 0;
        index < xarm_api::kSupervisedDriverMaximumJoints; ++index)
        {
          mapped.joint_positions[index] = report->angle[index];
        }
        std::lock_guard<std::mutex> lock(bridge->mutex);
        if (bridge->enabled && bridge->report_callback) {
          // Keep the bridge locked through invocation. release_callbacks()
          // then forms a teardown barrier for a callback already in flight,
          // instead of merely preventing the next callback from starting.
          bridge->report_callback(mapped);
        }
      });
  }

  void set_connection_callback(ConnectionCallback callback) override
  {
    const auto bridge = callback_bridge_;
    {
      std::lock_guard<std::mutex> lock(bridge->mutex);
      bridge->enabled = true;
      bridge->connection_callback = std::move(callback);
    }
    callbacks_released_ = false;
    arm_->register_connect_changed_callback(
      [bridge](bool connected_value, bool report_connected)
      {
        bridge->connected.store(
          connected_value, std::memory_order_release);
        std::lock_guard<std::mutex> lock(bridge->mutex);
        if (bridge->enabled && bridge->connection_callback) {
          bridge->connection_callback(
            connected_value, report_connected);
        }
      });
  }

  bool connected() const override
  {
    return callback_bridge_->connected.load(std::memory_order_acquire);
  }

  int read_joint_state(
    std::array<float, xarm_api::kSupervisedDriverMaximumJoints> & positions,
    std::array<float, xarm_api::kSupervisedDriverMaximumJoints> & velocities,
    std::size_t) override
  {
    std::array<float, xarm_api::kSupervisedDriverMaximumJoints> effort{{0.0F}};
    return arm_->get_joint_states(
      positions.data(), velocities.data(), effort.data(), 3);
  }

  int write_joint_position_command(
    const std::array<float, xarm_api::kSupervisedDriverMaximumJoints> &
    positions,
    std::size_t) override
  {
    auto command = positions;
    return arm_->set_servo_angle_j(command.data(), 0, 0, 0);
  }

  int shutdown_controller() override
  {
    return arm_->system_control(1);
  }

private:
  struct CallbackBridge
  {
    std::mutex mutex;
    ReportCallback report_callback;
    ConnectionCallback connection_callback;
    bool enabled = false;
    std::atomic<bool> connected{false};
  };

  std::unique_ptr<XArmAPI> arm_;
  std::shared_ptr<CallbackBridge> callback_bridge_ =
    std::make_shared<CallbackBridge>();
  bool callbacks_released_ = false;
  std::atomic<bool> closed_{true};
};

std::unique_ptr<xarm_api::SupervisedDriverTransport> make_transport(
  const xarm_api::SupervisedDriverSessionConfig & config)
{
  return std::unique_ptr<xarm_api::SupervisedDriverTransport>(
    new XArmApiSupervisedTransport(config));
}
}  // namespace

namespace xarm_api
{
SupervisedDriverSession::SupervisedDriverSession(
  SupervisedDriverSessionConfig config)
: SupervisedDriverSession(config, make_transport(config), true)
{
}

SupervisedDriverSession::SupervisedDriverSession(
  SupervisedDriverSessionConfig config,
  std::unique_ptr<SupervisedDriverTransport> transport,
  bool start_io_worker)
: config_(std::move(config)), transport_(std::move(transport))
{
  initialize(start_io_worker);
}

SupervisedDriverSession::~SupervisedDriverSession()
{
  close();
}

bool SupervisedDriverSession::ready() const noexcept
{
  std::lock_guard<std::mutex> lock(observation_mutex_);
  return !close_started_.load() &&
         !process_restart_required_.load(std::memory_order_acquire) &&
         observation_.connected &&
         observation_.identity_verified;
}

SupervisedDriverObservation SupervisedDriverSession::observe(
  std::int64_t) const
{
  std::lock_guard<std::mutex> lock(observation_mutex_);
  SupervisedDriverObservation result = observation_;
  result.joint_write_attempt_count =
    joint_write_attempt_count_.load(std::memory_order_acquire);
  result.lifecycle_command_attempt_count =
    lifecycle_command_attempt_count_.load(std::memory_order_acquire);
  result.shutdown_controller_attempt_count =
    shutdown_controller_attempt_count_.load(std::memory_order_acquire);
  result.command_gate_open =
    command_gate_open_.load(std::memory_order_acquire);
  result.command_gate_valid_until_ns =
    command_gate_valid_until_ns_.load(std::memory_order_acquire);
  if (result.source_timestamp_ns > 0) {
    result.fresh_until_ns = saturating_add(
      result.source_timestamp_ns, config_.observation_lease_ns);
  }
  return result;
}

DriverLifecycleCommandResult SupervisedDriverSession::execute_lifecycle(
  const DriverLifecycleCommand & command)
{
  static_cast<void>(set_command_gate(false, 0));
  if (close_started_.load() ||
    process_restart_required_.load(std::memory_order_acquire))
  {
    DriverLifecycleCommandResult result;
    result.reason = close_started_.load() ?
      "session_closing" : "process_restart_required";
    return result;
  }
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  if (close_started_.load() ||
    process_restart_required_.load(std::memory_order_acquire))
  {
    DriverLifecycleCommandResult result;
    result.reason = close_started_.load() ?
      "session_closing" : "process_restart_required";
    return result;
  }
  lifecycle_command_attempt_count_.fetch_add(
    1, std::memory_order_acq_rel);
  return execute_supervised_lifecycle_command(
    *transport_, access_policy_, transport_->connected(), command);
}

bool SupervisedDriverSession::read_joint_state(
  SupervisedDriverJointState & output) const noexcept
{
  if (!joint_valid_.load(std::memory_order_acquire)) {
    return false;
  }
  const std::int64_t timestamp_ns =
    joint_timestamp_ns_.load(std::memory_order_acquire);
  const std::int64_t now_ns = steady_now_ns();
  if (timestamp_ns <= 0 || timestamp_ns > now_ns ||
    now_ns - timestamp_ns >= config_.joint_state_lease_ns)
  {
    return false;
  }

  for (int attempt = 0; attempt < 3; ++attempt) {
    const std::uint64_t before =
      joint_sequence_.load(std::memory_order_acquire);
    if ((before & 1U) != 0U) {
      continue;
    }
    output.joint_count = config_.joint_count;
    for (std::size_t index = 0; index < config_.joint_count; ++index) {
      output.positions[index] =
        joint_positions_[index].load(std::memory_order_relaxed);
      output.velocities[index] =
        joint_velocities_[index].load(std::memory_order_relaxed);
    }
    output.generation =
      joint_generation_.load(std::memory_order_relaxed);
    output.source_timestamp_ns = timestamp_ns;
    const std::uint64_t after =
      joint_sequence_.load(std::memory_order_acquire);
    if (before == after && (after & 1U) == 0U) {
      return output.generation > 0;
    }
  }
  return false;
}

bool SupervisedDriverSession::submit_joint_position_command(
  const std::array<double, kSupervisedDriverMaximumJoints> & positions,
  std::size_t joint_count) noexcept
{
  if (!command_gate_open_.load(std::memory_order_acquire) ||
    close_started_.load() ||
    joint_count != config_.joint_count)
  {
    return false;
  }
  for (std::size_t index = 0; index < joint_count; ++index) {
    if (!std::isfinite(positions[index])) {
      return false;
    }
  }
  const std::uint64_t gate_version =
    command_gate_version_.load(std::memory_order_acquire);
  const std::int64_t valid_until_ns =
    command_gate_valid_until_ns_.load(std::memory_order_acquire);
  if (!command_gate_open_.load(std::memory_order_acquire) ||
    steady_now_ns() >= valid_until_ns)
  {
    return false;
  }

  command_sequence_.fetch_add(1, std::memory_order_acq_rel);
  for (std::size_t index = 0; index < joint_count; ++index) {
    command_positions_[index].store(
      positions[index], std::memory_order_relaxed);
  }
  command_version_.store(gate_version, std::memory_order_relaxed);
  command_sequence_.fetch_add(1, std::memory_order_release);
  if (!command_gate_open_.load(std::memory_order_acquire) ||
    command_gate_version_.load(std::memory_order_acquire) != gate_version ||
    steady_now_ns() >=
    command_gate_valid_until_ns_.load(std::memory_order_acquire))
  {
    return false;
  }
  worker_condition_.notify_one();
  return true;
}

bool SupervisedDriverSession::set_command_gate(
  bool open, std::int64_t valid_until_ns)
{
  std::unique_lock<std::mutex> control_lock(gate_control_mutex_);
  const std::int64_t now_ns = steady_now_ns();
  const std::int64_t joint_timestamp_ns =
    joint_timestamp_ns_.load(std::memory_order_acquire);
  bool report_observation_ready = false;
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    report_observation_ready =
      observation_.connected &&
      observation_.report_connected &&
      observation_.identity_verified &&
      observation_.report_received &&
      observation_.position_valid &&
      observation_.report.mode == XARM_MODE::SERVO &&
      command_path_state(observation_.report.state) &&
      observation_.source_timestamp_ns > 0 &&
      observation_.source_timestamp_ns <= now_ns &&
      now_ns - observation_.source_timestamp_ns <
      config_.observation_lease_ns;
  }
  if (!open || valid_until_ns <= now_ns ||
    !report_observation_ready ||
    !position_initialized_.load(std::memory_order_acquire) ||
    !joint_valid_.load(std::memory_order_acquire) ||
    joint_timestamp_ns <= 0 || joint_timestamp_ns > now_ns ||
    now_ns - joint_timestamp_ns >= config_.joint_state_lease_ns ||
    process_restart_required_.load(std::memory_order_acquire) ||
    close_started_.load())
  {
    static_cast<void>(close_command_gate_locked());
    control_lock.unlock();
    // An explicit close is a barrier even if another fault path closed the
    // atomic gate first. Otherwise the caller could observe a completed close
    // while an SDK write that started before that fault is still in flight.
    std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
    worker_condition_.notify_one();
    return false;
  }

  if (close_started_.load() || !transport_->connected()) {
    static_cast<void>(close_command_gate_locked());
    control_lock.unlock();
    std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
    worker_condition_.notify_one();
    return false;
  }
  if (!command_gate_open_.load(std::memory_order_acquire)) {
    command_gate_version_.fetch_add(1, std::memory_order_acq_rel);
  }
  command_gate_valid_until_ns_.store(
    valid_until_ns, std::memory_order_release);
  command_gate_open_.store(true, std::memory_order_release);
  worker_condition_.notify_one();
  return true;
}

SupervisedControllerShutdownResult
SupervisedDriverSession::shutdown_controller()
{
  static_cast<void>(set_command_gate(false, 0));
  SupervisedControllerShutdownResult result;
  if (close_started_.load() ||
    process_restart_required_.load(std::memory_order_acquire))
  {
    result.reason = close_started_.load() ?
      "session_closing" : "process_restart_required";
    return result;
  }

  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    if (close_started_.load() ||
      process_restart_required_.load(std::memory_order_acquire))
    {
      result.reason = close_started_.load() ?
        "session_closing" : "process_restart_required";
      return result;
    }
    result.permitted = true;
    if (!transport_->connected()) {
      result.reason = "transport_not_connected";
      return result;
    }
    shutdown_controller_attempt_count_.fetch_add(
      1, std::memory_order_acq_rel);
    result.attempted = true;
    result.return_code = transport_->shutdown_controller();
  }

  // The command can remove the observation channel. Even a non-zero return
  // cannot safely reuse this transport after an attempted controller power
  // operation, so every attempt requires a fresh owner process.
  require_process_restart(true, true);
  result.process_restart_required = true;
  result.reason = result.return_code == 0 ?
    "controller_shutdown_vendor_accepted" :
    "controller_shutdown_result_ambiguous";
  return result;
}

void SupervisedDriverSession::service_io_once()
{
  if (close_started_.load() ||
    process_restart_required_.load(std::memory_order_acquire))
  {
    return;
  }
  if (command_gate_open_.load(std::memory_order_acquire) &&
    steady_now_ns() >=
    command_gate_valid_until_ns_.load(std::memory_order_acquire))
  {
    close_command_gate(false);
  }

  std::array<float, kSupervisedDriverMaximumJoints> positions{{0.0F}};
  std::array<float, kSupervisedDriverMaximumJoints> velocities{{0.0F}};
  int read_return = -1;
  bool joint_read_attempted = false;
  bool transport_claimed_connected = false;
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    transport_claimed_connected = transport_->connected();
    if (transport_claimed_connected) {
      joint_read_attempted = true;
      read_return = transport_->read_joint_state(
        positions, velocities, config_.joint_count);
    }
  }
  if (joint_read_attempted) {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    ++observation_.joint_read_attempt_count;
    if (read_return == 0) {
      ++observation_.joint_read_success_count;
    }
  }
  const std::int64_t health_check_ns = steady_now_ns();
  if (read_return == 0) {
    last_joint_read_success_ns_ = health_check_ns;
  }
  bool finite_state = read_return == 0;
  for (std::size_t index = 0;
    finite_state && index < config_.joint_count; ++index)
  {
    finite_state = std::isfinite(positions[index]) &&
      std::isfinite(velocities[index]);
  }
  if (finite_state) {
    const std::int64_t joint_sample_time_ns = steady_now_ns();
    joint_sequence_.fetch_add(1, std::memory_order_acq_rel);
    for (std::size_t index = 0; index < config_.joint_count; ++index) {
      joint_positions_[index].store(
        positions[index], std::memory_order_relaxed);
      joint_velocities_[index].store(
        velocities[index], std::memory_order_relaxed);
    }
    joint_timestamp_ns_.store(
      joint_sample_time_ns, std::memory_order_relaxed);
    joint_generation_.fetch_add(1, std::memory_order_relaxed);
    joint_sequence_.fetch_add(1, std::memory_order_release);

    bool position_initialized = false;
    bool shutdown_source_agreement = false;
    {
      std::lock_guard<std::mutex> lock(observation_mutex_);
      const std::uint64_t report_generation =
        observation_.report_sample_count;
      const bool new_report =
        report_generation !=
        last_position_initialization_report_generation_;
      const int expected_mask =
        (1 << static_cast<int>(config_.joint_count)) - 1;
      bool sources_agree =
        new_report &&
        observation_.report_received &&
        observation_.report.error_code == 0 &&
        observation_.report.warning_code == 0 &&
        (observation_.report.brake_mask & expected_mask) ==
        expected_mask &&
        (observation_.report.servo_enable_mask & expected_mask) ==
        expected_mask;
      for (std::size_t index = 0;
        sources_agree && index < config_.joint_count; ++index)
      {
        sources_agree =
          std::isfinite(observation_.report.joint_positions[index]) &&
          std::abs(
          observation_.report.joint_positions[index] -
          static_cast<double>(positions[index])) <=
          config_.source_agreement_tolerance_rad;
      }
      shutdown_source_agreement =
        observation_.report_received &&
        observation_.source_timestamp_ns > 0;
      for (std::size_t index = 0;
        shutdown_source_agreement && index < config_.joint_count; ++index)
      {
        shutdown_source_agreement =
          std::isfinite(observation_.report.joint_positions[index]) &&
          std::abs(
          observation_.report.joint_positions[index] -
          static_cast<double>(positions[index])) <=
          config_.source_agreement_tolerance_rad;
      }
      if (sources_agree) {
        last_position_initialization_report_generation_ =
          report_generation;
        if (!position_initialized_.load(std::memory_order_acquire)) {
          const std::size_t match_count =
            position_initialization_match_count_.fetch_add(
            1, std::memory_order_acq_rel) + 1;
          if (match_count >=
            config_.position_initialization_samples)
          {
            position_initialized_.store(true, std::memory_order_release);
            position_ever_initialized_ = true;
            observation_.position_ever_initialized = true;
          }
        }
      } else if (new_report) {
        last_position_initialization_report_generation_ =
          report_generation;
        // Rich reports and direct joint polls are not time-aligned. Their
        // positions qualify the initial powered pose, but normal motion must
        // not revoke an already qualified pose merely because the newer poll
        // has advanced beyond the report. Explicit report faults, drive-mask
        // loss, read failure, and transport loss still clear the latch.
        if (!position_initialized_.load(std::memory_order_acquire)) {
          position_initialization_match_count_.store(
            0, std::memory_order_release);
        }
      }
      position_initialized =
        position_initialized_.load(std::memory_order_acquire);
      joint_valid_.store(position_initialized, std::memory_order_release);
      if (observation_.position_valid != position_initialized ||
        observation_.last_joint_read_return_code != 0)
      {
        observation_.position_valid = position_initialized;
        observation_.last_joint_read_return_code = 0;
        observation_.generation = next_observation_generation_++;
      }
    }
    update_shutdown_stationarity(
      positions, joint_sample_time_ns, shutdown_source_agreement);
    if (!position_initialized) {
      close_command_gate(false);
    }
  } else {
    joint_valid_.store(false, std::memory_order_release);
    set_position_valid(false, read_return);
    clear_shutdown_stationarity();
    close_command_gate(false);
  }
  evaluate_transport_health(
    health_check_ns,
    transport_claimed_connected,
    joint_read_attempted && read_return == 0);
  if (process_restart_required_.load(std::memory_order_acquire)) {
    return;
  }

  if (!command_gate_open_.load(std::memory_order_acquire)) {
    consumed_command_sequence_.store(
      command_sequence_.load(std::memory_order_acquire),
      std::memory_order_release);
    return;
  }
  const std::uint64_t consumed =
    consumed_command_sequence_.load(std::memory_order_acquire);
  const std::uint64_t sequence =
    command_sequence_.load(std::memory_order_acquire);
  if (sequence == consumed || (sequence & 1U) != 0U) {
    return;
  }
  const std::uint64_t command_version =
    command_version_.load(std::memory_order_relaxed);
  std::array<float, kSupervisedDriverMaximumJoints> command{{0.0F}};
  for (std::size_t index = 0; index < config_.joint_count; ++index) {
    command[index] = static_cast<float>(
      command_positions_[index].load(std::memory_order_relaxed));
  }
  if (command_sequence_.load(std::memory_order_acquire) != sequence ||
    !command_gate_open_.load(std::memory_order_acquire) ||
    command_gate_version_.load(std::memory_order_acquire) !=
    command_version ||
    steady_now_ns() >=
    command_gate_valid_until_ns_.load(std::memory_order_acquire))
  {
    return;
  }

  int write_return = -1;
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    if (command_gate_open_.load(std::memory_order_acquire) &&
      command_gate_version_.load(std::memory_order_acquire) ==
      command_version &&
      steady_now_ns() <
      command_gate_valid_until_ns_.load(std::memory_order_acquire) &&
      transport_->connected())
    {
      joint_write_attempt_count_.fetch_add(
        1, std::memory_order_acq_rel);
      write_return = transport_->write_joint_position_command(
        command, config_.joint_count);
    }
  }
  consumed_command_sequence_.store(sequence, std::memory_order_release);
  set_last_joint_write_return(write_return);
  if (write_return != 0) {
    static_cast<void>(set_command_gate(false, 0));
  }
}

void SupervisedDriverSession::close() noexcept
{
  if (close_started_.exchange(true)) {
    return;
  }
  try {
    static_cast<void>(close_command_gate(true));
  } catch (...) {
    command_gate_open_.store(false, std::memory_order_release);
  }
  worker_condition_.notify_all();
  if (io_worker_.joinable()) {
    io_worker_.join();
  }
  try {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    if (!transport_closed_) {
      close_driver_transport(
        *transport_, access_policy_, transport_->connected());
      transport_closed_ = true;
    }
  } catch (...) {
  }
  handle_connection(false, false);
}

std::int64_t SupervisedDriverSession::steady_now_ns() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void SupervisedDriverSession::initialize(bool start_io_worker)
{
  if (transport_ == nullptr) {
    throw std::invalid_argument("supervised xArm transport is missing");
  }
  if (config_.robot_address.empty()) {
    throw std::invalid_argument("supervised xArm address is empty");
  }
  if (config_.joint_count == 0 ||
    config_.joint_count > kSupervisedDriverMaximumJoints)
  {
    throw std::invalid_argument("supervised xArm joint count is invalid");
  }
  if (config_.identity_expectation.axis !=
    static_cast<int>(config_.joint_count) ||
    config_.identity_expectation.device_type < 0)
  {
    throw std::invalid_argument(
            "supervised xArm identity expectation is incomplete");
  }
  if (config_.observation_lease_ns <= 0 ||
    config_.joint_state_lease_ns <= 0 ||
    config_.io_period_ns <= 0 ||
    config_.transport_loss_timeout_ns <= config_.observation_lease_ns ||
    config_.transport_loss_timeout_ns <= config_.joint_state_lease_ns ||
    !std::isfinite(config_.source_agreement_tolerance_rad) ||
    config_.source_agreement_tolerance_rad <= 0.0 ||
    config_.position_initialization_samples == 0 ||
    !std::isfinite(config_.shutdown_stationary_tolerance_rad) ||
    config_.shutdown_stationary_tolerance_rad <= 0.0 ||
    config_.shutdown_stationary_dwell_ns <= 0)
  {
    throw std::invalid_argument("supervised xArm timing is invalid");
  }
  for (std::size_t index = 0;
    index < kSupervisedDriverMaximumJoints; ++index)
  {
    if (!joint_positions_[index].is_lock_free() ||
      !joint_velocities_[index].is_lock_free() ||
      !command_positions_[index].is_lock_free())
    {
      throw std::runtime_error(
              "supervised xArm RT cache requires lock-free atomic<double>");
    }
    joint_positions_[index].store(0.0);
    joint_velocities_[index].store(0.0);
    command_positions_[index].store(0.0);
  }
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    observation_.generation = next_observation_generation_++;
  }
  transport_->set_report_callback(
    [this](const SupervisedDriverReport & report)
    {
      handle_report(report);
    });
  transport_->set_connection_callback(
    [this](bool connected, bool report_connected)
    {
      handle_connection(connected, report_connected);
    });

  const DriverStartupObservation startup = observe_driver_startup(
    *transport_, access_policy_, static_cast<int>(config_.joint_count),
    config_.identity_expectation);
  if (!startup.accepted()) {
    transport_closed_ =
      startup.failed_connection_cleanup_performed ||
      startup.failed_identity_cleanup_performed;
    throw std::runtime_error(
            startup.connected() ?
            "supervised xArm identity verification failed" :
            "supervised xArm connection failed");
  }
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    observation_.connected = true;
    observation_.identity_verified = true;
    observation_.identity = startup.identity;
    if (!observation_.report_received &&
      startup.error_warning_result == 0)
    {
      observation_.report.error_code = startup.error_warning[0];
      observation_.report.warning_code = startup.error_warning[1];
    }
    observation_.generation = next_observation_generation_++;
  }
  startup_completed_.store(true, std::memory_order_release);
  loss_detection_started_ns_ = steady_now_ns();
  last_joint_read_success_ns_ = loss_detection_started_ns_;
  if (start_io_worker) {
    io_worker_ = std::thread([this]() {run_io_worker();});
  }
}

void SupervisedDriverSession::handle_report(
  const SupervisedDriverReport & report)
{
  if (close_started_.load() ||
    process_restart_required_.load(std::memory_order_acquire))
  {
    return;
  }
  const int expected_mask =
    (1 << static_cast<int>(config_.joint_count)) - 1;
  const bool controller_pose_untrusted =
    report.error_code != 0 ||
    report.warning_code != 0 ||
    (report.brake_mask & expected_mask) != expected_mask ||
    (report.servo_enable_mask & expected_mask) != expected_mask;
  const bool command_path_untrusted =
    controller_pose_untrusted ||
    report.mode != XARM_MODE::SERVO ||
    !command_path_state(report.state);
  std::unique_lock<std::mutex> gate_lock(
    gate_control_mutex_, std::defer_lock);
  if (command_path_untrusted) {
    // Serialize the unsafe observation and gate closure with any concurrent
    // open request. This prevents an opener from validating the old report
    // after this callback has already decided the new report is unsafe.
    gate_lock.lock();
  }
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    const bool position_changed =
      controller_pose_untrusted && observation_.position_valid;
    const bool shutdown_context_changed =
      !observation_.report_received ||
      observation_.report.state != report.state ||
      observation_.report.mode != report.mode ||
      observation_.report.brake_mask != report.brake_mask ||
      observation_.report.servo_enable_mask != report.servo_enable_mask ||
      observation_.report.error_code != report.error_code ||
      observation_.report.warning_code != report.warning_code;
    const bool changed =
      !observation_.report_received ||
      observation_.report.state != report.state ||
      observation_.report.mode != report.mode ||
      observation_.report.command_count != report.command_count ||
      observation_.report.brake_mask != report.brake_mask ||
      observation_.report.servo_enable_mask != report.servo_enable_mask ||
      observation_.report.error_code != report.error_code ||
      observation_.report.warning_code != report.warning_code;
    observation_.report_received = true;
    observation_.report = report;
    ++observation_.report_sample_count;
    observation_.source_timestamp_ns = steady_now_ns();
    if (shutdown_context_changed) {
      observation_.stationary = false;
      observation_.stationary_since_ns = 0;
      stationary_reference_valid_ = false;
    }
    if (controller_pose_untrusted) {
      position_initialization_match_count_.store(
        0, std::memory_order_release);
      position_initialized_.store(false, std::memory_order_release);
      joint_valid_.store(false, std::memory_order_release);
      observation_.position_valid = false;
    }
    if (changed || position_changed) {
      observation_.generation = next_observation_generation_++;
    }
  }
  if (command_path_untrusted) {
    static_cast<void>(close_command_gate_locked());
    gate_lock.unlock();
    worker_condition_.notify_one();
  }
}

void SupervisedDriverSession::handle_connection(
  bool connected, bool report_connected)
{
  const bool observation_lost = !connected || !report_connected;
  const bool unexpected_transport_loss =
    observation_lost &&
    startup_completed_.load(std::memory_order_acquire) &&
    !close_started_.load(std::memory_order_acquire);
  if (unexpected_transport_loss) {
    process_restart_required_.store(true, std::memory_order_release);
  }
  std::unique_lock<std::mutex> gate_lock(
    gate_control_mutex_, std::defer_lock);
  if (observation_lost) {
    // Publish loss and close delivery under the same gate lock so a
    // concurrent opener cannot validate the previous connected observation
    // after this loss has been handled.
    gate_lock.lock();
    static_cast<void>(close_command_gate_locked());
    position_initialized_.store(false, std::memory_order_release);
    position_initialization_match_count_.store(
      0, std::memory_order_release);
    joint_valid_.store(false, std::memory_order_release);
  }
  std::lock_guard<std::mutex> lock(observation_mutex_);
  const bool changed =
    observation_.connected != connected ||
    observation_.report_connected != report_connected ||
    observation_.process_restart_required !=
    process_restart_required_.load(std::memory_order_acquire);
  observation_.connected = connected;
  observation_.report_connected = report_connected;
  observation_.process_restart_required =
    process_restart_required_.load(std::memory_order_acquire);
  if (observation_lost) {
    stationary_reference_valid_ = false;
    observation_.position_valid = false;
    observation_.stationary = false;
    observation_.stationary_since_ns = 0;
    observation_.report_received = false;
    observation_.source_timestamp_ns = 0;
    observation_.fresh_until_ns = 0;
  }
  if (!connected || observation_.process_restart_required) {
    observation_.identity_verified = false;
  }
  if (changed) {
    observation_.generation = next_observation_generation_++;
  }
  if (gate_lock.owns_lock()) {
    gate_lock.unlock();
    worker_condition_.notify_one();
  }
}

void SupervisedDriverSession::evaluate_transport_health(
  std::int64_t now_ns,
  bool transport_claimed_connected,
  bool joint_read_succeeded)
{
  if (close_started_.load() ||
    process_restart_required_.load(std::memory_order_acquire) ||
    !startup_completed_.load(std::memory_order_acquire))
  {
    return;
  }

  std::int64_t last_report_ns = loss_detection_started_ns_;
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    if (observation_.report_received &&
      observation_.source_timestamp_ns > 0)
    {
      last_report_ns = observation_.source_timestamp_ns;
    }
  }
  const bool report_lost =
    now_ns >= last_report_ns &&
    now_ns - last_report_ns >= config_.transport_loss_timeout_ns;
  const bool control_lost =
    !transport_claimed_connected ||
    (!joint_read_succeeded &&
    now_ns >= last_joint_read_success_ns_ &&
    now_ns - last_joint_read_success_ns_ >=
    config_.transport_loss_timeout_ns);
  if (control_lost || report_lost) {
    require_process_restart(control_lost, report_lost);
  }
}

void SupervisedDriverSession::require_process_restart(
  bool control_lost, bool report_lost)
{
  process_restart_required_.store(true, std::memory_order_release);
  std::unique_lock<std::mutex> gate_lock(gate_control_mutex_);
  static_cast<void>(close_command_gate_locked());
  position_initialized_.store(false, std::memory_order_release);
  position_initialization_match_count_.store(
    0, std::memory_order_release);
  joint_valid_.store(false, std::memory_order_release);

  std::lock_guard<std::mutex> lock(observation_mutex_);
  stationary_reference_valid_ = false;
  if (control_lost) {
    observation_.connected = false;
  }
  if (report_lost) {
    observation_.report_connected = false;
    observation_.report_received = false;
    observation_.source_timestamp_ns = 0;
    observation_.fresh_until_ns = 0;
  }
  observation_.identity_verified = false;
  observation_.process_restart_required = true;
  observation_.position_valid = false;
  observation_.stationary = false;
  observation_.stationary_since_ns = 0;
  observation_.generation = next_observation_generation_++;
  gate_lock.unlock();
  worker_condition_.notify_one();
}

void SupervisedDriverSession::set_position_valid(
  bool valid, int return_code)
{
  std::lock_guard<std::mutex> lock(observation_mutex_);
  if (observation_.position_valid != valid ||
    observation_.last_joint_read_return_code != return_code)
  {
    observation_.position_valid = valid;
    observation_.last_joint_read_return_code = return_code;
    observation_.generation = next_observation_generation_++;
  }
}

void SupervisedDriverSession::set_last_joint_write_return(int return_code)
{
  std::lock_guard<std::mutex> lock(observation_mutex_);
  if (observation_.last_joint_write_return_code != return_code) {
    observation_.last_joint_write_return_code = return_code;
    observation_.generation = next_observation_generation_++;
  }
}

void SupervisedDriverSession::update_shutdown_stationarity(
  const std::array<float, kSupervisedDriverMaximumJoints> & positions,
  std::int64_t sample_time_ns,
  bool source_agreement)
{
  std::lock_guard<std::mutex> lock(observation_mutex_);
  if (!source_agreement || !position_ever_initialized_) {
    if (observation_.stationary ||
      observation_.stationary_since_ns != 0 ||
      stationary_reference_valid_)
    {
      observation_.stationary = false;
      observation_.stationary_since_ns = 0;
      stationary_reference_valid_ = false;
      observation_.generation = next_observation_generation_++;
    }
    return;
  }

  bool within_reference = stationary_reference_valid_;
  for (std::size_t index = 0;
    within_reference && index < config_.joint_count; ++index)
  {
    within_reference =
      std::abs(
      static_cast<double>(positions[index]) -
      stationary_reference_positions_[index]) <=
      config_.shutdown_stationary_tolerance_rad;
  }
  if (!within_reference) {
    for (std::size_t index = 0; index < config_.joint_count; ++index) {
      stationary_reference_positions_[index] = positions[index];
    }
    stationary_reference_valid_ = true;
    observation_.stationary = false;
    observation_.stationary_since_ns = sample_time_ns;
    observation_.generation = next_observation_generation_++;
    return;
  }

  const bool stationary =
    sample_time_ns >= observation_.stationary_since_ns &&
    sample_time_ns - observation_.stationary_since_ns >=
    config_.shutdown_stationary_dwell_ns;
  if (observation_.stationary != stationary) {
    observation_.stationary = stationary;
    observation_.generation = next_observation_generation_++;
  }
}

void SupervisedDriverSession::clear_shutdown_stationarity()
{
  std::lock_guard<std::mutex> lock(observation_mutex_);
  if (observation_.stationary ||
    observation_.stationary_since_ns != 0 ||
    stationary_reference_valid_)
  {
    observation_.stationary = false;
    observation_.stationary_since_ns = 0;
    stationary_reference_valid_ = false;
    observation_.generation = next_observation_generation_++;
  }
}

bool SupervisedDriverSession::close_command_gate_locked() noexcept
{
  const bool was_open = command_gate_open_.exchange(
    false, std::memory_order_acq_rel);
  command_gate_valid_until_ns_.store(0, std::memory_order_release);
  if (was_open) {
    command_gate_version_.fetch_add(1, std::memory_order_acq_rel);
  }
  consumed_command_sequence_.store(
    command_sequence_.load(std::memory_order_acquire),
    std::memory_order_release);
  return was_open;
}

bool SupervisedDriverSession::close_command_gate(bool synchronize_sdk)
{
  bool was_open = false;
  {
    std::lock_guard<std::mutex> control_lock(gate_control_mutex_);
    was_open = close_command_gate_locked();
  }
  if (synchronize_sdk) {
    std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
  }
  worker_condition_.notify_one();
  return was_open;
}

void SupervisedDriverSession::run_io_worker()
{
  const auto period = std::chrono::nanoseconds(config_.io_period_ns);
  while (!close_started_.load()) {
    service_io_once();
    std::unique_lock<std::mutex> lock(worker_mutex_);
    worker_condition_.wait_for(lock, period);
  }
}
}  // namespace xarm_api
