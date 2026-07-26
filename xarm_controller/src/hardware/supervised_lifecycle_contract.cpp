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

#include "xarm_controller/hardware/supervised_lifecycle_contract.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "xarm_msgs/srv/execute_supervised_lifecycle_command.hpp"

namespace uf_robot_hardware
{
namespace
{
constexpr int kAllServos = 8;
constexpr int kJointServoMode = 1;
constexpr int kStandbyStateRequest = 0;
constexpr int kReadyFeedbackState = 2;
constexpr int kStoppedState = 4;
}  // namespace

SupervisedCommandReplayCache::SupervisedCommandReplayCache(
  std::size_t capacity)
: capacity_(capacity)
{
  if (capacity_ == 0) {
    throw std::invalid_argument(
            "supervised command replay capacity must be positive");
  }
}

SupervisedReplayDisposition SupervisedCommandReplayCache::lookup(
  const std::string & request_id,
  std::uint8_t command,
  std::uint64_t expected_generation,
  SupervisedCommandRecord & record) const
{
  const auto found = std::find_if(
    records_.begin(), records_.end(),
    [&request_id](const SupervisedCommandRecord & candidate) {
      return candidate.request_id == request_id;
    });
  if (found == records_.end()) {
    return SupervisedReplayDisposition::kMiss;
  }
  record = *found;
  return found->command == command &&
         found->expected_generation == expected_generation ?
         SupervisedReplayDisposition::kReplay :
         SupervisedReplayDisposition::kConflict;
}

void SupervisedCommandReplayCache::remember(
  SupervisedCommandRecord record)
{
  if (records_.size() == capacity_) {
    records_.pop_front();
  }
  records_.push_back(std::move(record));
}

void SupervisedCommandReplayCache::clear()
{
  records_.clear();
}

bool valid_supervised_request_id(const std::string & request_id)
{
  if (request_id.empty() || request_id.size() > 128) {
    return false;
  }
  return std::all_of(
    request_id.begin(), request_id.end(),
    [](char value) {
      const unsigned char character =
      static_cast<unsigned char>(value);
      return character >= 0x21U && character <= 0x7eU;
    });
}

SupervisedReadDisposition supervised_read_disposition(
  bool driver_available,
  bool joint_sample_available)
{
  if (driver_available && joint_sample_available) {
    return SupervisedReadDisposition::kPublishSample;
  }
  if (driver_available) {
    // The SDK owner closes its command gate before invalidating the sample.
    // Hold the last state (initially NaN) so a transient poll failure or
    // interlock preserves diagnostics instead of tearing down the sole owner.
    return SupervisedReadDisposition::kHoldLastState;
  }
  return SupervisedReadDisposition::kFault;
}

SupervisedWriteDisposition supervised_write_disposition(
  bool driver_available,
  bool hardware_active,
  bool command_valid,
  bool submission_accepted)
{
  if (!driver_available || !hardware_active || !command_valid) {
    return SupervisedWriteDisposition::kFault;
  }
  // A closed or concurrently expired command gate is a normal fail-safe
  // fence. The hardware owner remains healthy and observable; no SDK write
  // was accepted.
  return submission_accepted ?
         SupervisedWriteDisposition::kDelivered :
         SupervisedWriteDisposition::kFenced;
}

bool supervised_initial_activation_state(int state)
{
  // set_state(0) requests standby. UFACTORY reports state 2 only after the
  // selected mode is READY; initial hardware activation must wait for that
  // observed postcondition. State 1 is valid only after motion is admitted.
  return state == kReadyFeedbackState;
}

const char * supervised_controller_shutdown_rejection(
  const SupervisedControllerShutdownFacts & facts)
{
  if (!facts.transport_connected || !facts.report_connected) {
    return "transport_not_connected";
  }
  if (!facts.identity_verified) {
    return "identity_not_verified";
  }
  if (facts.process_restart_required) {
    return "process_restart_required";
  }
  if (facts.command_gate_open) {
    return "command_gate_open";
  }
  if (facts.hardware_component_active) {
    return "hardware_component_active";
  }
  if (!facts.controller_activity_known) {
    return "controller_activity_unknown";
  }
  if (facts.trajectory_controller_active) {
    return "trajectory_controller_active";
  }
  if (!facts.position_ever_initialized) {
    return "position_never_initialized";
  }
  if (!facts.stationary) {
    return "stationary_dwell_not_satisfied";
  }
  if (facts.state != kStoppedState) {
    return "robot_not_stopped";
  }
  if (facts.mode != kJointServoMode) {
    return "unexpected_control_mode";
  }
  if (facts.brake_mask != 0 || facts.servo_enable_mask != 0) {
    return "drives_not_disabled";
  }
  if (facts.warning_code != 0) {
    return "warning_present";
  }
  if (
    facts.error_code != 0 &&
    facts.error_code != 1 &&
    facts.error_code != 2 &&
    facts.error_code != 3)
  {
    return "ineligible_fault_present";
  }
  return nullptr;
}

bool map_supervised_command(
  std::uint8_t command,
  xarm_api::DriverLifecycleCommand & primitive)
{
  using Request =
    xarm_msgs::srv::ExecuteSupervisedLifecycleCommand::Request;
  switch (command) {
    case Request::CLEAR_ERROR:
      primitive = {
        xarm_api::DriverLifecycleCommandKind::kClearError, 0,
        kAllServos};
      return true;
    case Request::CLEAR_WARNING:
      primitive = {
        xarm_api::DriverLifecycleCommandKind::kClearWarning, 0,
        kAllServos};
      return true;
    case Request::ENABLE_MOTION:
      primitive = {
        xarm_api::DriverLifecycleCommandKind::kSetMotionEnabled, 1,
        kAllServos};
      return true;
    case Request::DISABLE_MOTION:
      primitive = {
        xarm_api::DriverLifecycleCommandKind::kSetMotionEnabled, 0,
        kAllServos};
      return true;
    case Request::SELECT_JOINT_SERVO_MODE:
      primitive = {
        xarm_api::DriverLifecycleCommandKind::kSetMode,
        kJointServoMode, kAllServos};
      return true;
    case Request::SET_READY_STATE:
      primitive = {
        xarm_api::DriverLifecycleCommandKind::kSetState,
        kStandbyStateRequest, kAllServos};
      return true;
    case Request::SET_STOPPED_STATE:
      primitive = {
        xarm_api::DriverLifecycleCommandKind::kSetState,
        kStoppedState, kAllServos};
      return true;
    default:
      return false;
  }
}
}  // namespace uf_robot_hardware
