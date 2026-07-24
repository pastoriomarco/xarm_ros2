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

#ifndef XARM_API__DRIVER_LIFECYCLE_H_
#define XARM_API__DRIVER_LIFECYCLE_H_

#include <array>
#include <cstddef>
#include <string>

#include "xarm_api/driver_access_policy.h"

// Keep the package's cpplint namespace convention despite its legacy
// uncrustify profile requesting the opposite indentation.
// *INDENT-OFF*
namespace xarm_api
{
constexpr std::size_t kDriverErrorWarningWords = 2;
constexpr std::size_t kDriverServoDebugWords = 16;
constexpr std::size_t kDriverMaximumJoints = kDriverServoDebugWords / 2;

struct DriverRobotIdentity
{
  int axis = -1;
  int device_type = -1;
  std::string serial;
};

struct DriverIdentityExpectation
{
  int axis = -1;
  int device_type = -1;
  std::string serial;

  bool required() const
  {
    return axis >= 0 || device_type >= 0 || !serial.empty();
  }

  bool matches(const DriverRobotIdentity & identity) const
  {
    return
      (axis < 0 || identity.axis == axis) &&
      (device_type < 0 || identity.device_type == device_type) &&
      (serial.empty() || identity.serial == serial);
  }
};

class DriverLifecycleTransport
{
public:
  virtual ~DriverLifecycleTransport() = default;

  virtual int connect() = 0;
  virtual int read_robot_identity(DriverRobotIdentity & identity) = 0;
  virtual int read_error_warning(
    std::array<int, kDriverErrorWarningWords> & error_warning) = 0;
  virtual int read_servo_debug(
    std::array<int, kDriverServoDebugWords> & servo_debug) = 0;
  virtual int clear_error() = 0;
  virtual int set_pose_mode() = 0;
  virtual void release_callbacks() = 0;
  virtual void disconnect() = 0;
};

struct DriverStartupObservation
{
  int connect_result = -1;
  bool identity_check_required = false;
  int identity_read_result = -1;
  bool identity_matched = false;
  bool failed_identity_cleanup_performed = false;
  DriverRobotIdentity identity;
  int error_warning_result = -1;
  int servo_debug_result = -1;
  bool failed_connection_cleanup_performed = false;
  std::size_t inspected_joint_count = 0;
  std::array<int, kDriverErrorWarningWords> error_warning{{0, 0}};
  std::array<int, kDriverServoDebugWords> servo_debug{{0}};
  std::array<bool, kDriverMaximumJoints> automatic_fault_clear_attempted{{false}};
  std::array<int, kDriverMaximumJoints> automatic_fault_clear_result{{0}};

  bool connected() const
  {
    return connect_result == 0;
  }

  bool accepted() const
  {
    return connected() &&
           (!identity_check_required ||
           (identity_read_result == 0 && identity_matched));
  }
};

inline DriverStartupObservation observe_driver_startup(
  DriverLifecycleTransport & transport,
  const DriverAccessPolicy & access_policy,
  int requested_joint_count,
  const DriverIdentityExpectation & identity_expectation = {})
{
  DriverStartupObservation observation;
  observation.connect_result = transport.connect();
  if (!observation.connected()) {
    transport.release_callbacks();
    transport.disconnect();
    observation.failed_connection_cleanup_performed = true;
    return observation;
  }

  observation.identity_check_required = identity_expectation.required();
  if (observation.identity_check_required) {
    observation.identity_read_result =
      transport.read_robot_identity(observation.identity);
    observation.identity_matched =
      observation.identity_read_result == 0 &&
      identity_expectation.matches(observation.identity);
    if (!observation.identity_matched) {
      transport.release_callbacks();
      transport.disconnect();
      observation.failed_identity_cleanup_performed = true;
      return observation;
    }
  }

  observation.error_warning_result =
    transport.read_error_warning(observation.error_warning);
  observation.servo_debug_result =
    transport.read_servo_debug(observation.servo_debug);

  if (requested_joint_count > 0) {
    const std::size_t requested = static_cast<std::size_t>(requested_joint_count);
    observation.inspected_joint_count =
      requested < kDriverMaximumJoints ? requested : kDriverMaximumJoints;
  }

  if (
    observation.servo_debug_result != 0 ||
    !access_policy.permits_automatic_fault_clear())
  {
    return observation;
  }

  for (std::size_t index = 0; index < observation.inspected_joint_count; ++index) {
    const std::size_t status_index = index * 2;
    if (observation.servo_debug[status_index] != 1) {
      continue;
    }
    observation.automatic_fault_clear_attempted[index] = true;
    observation.automatic_fault_clear_result[index] = transport.clear_error();
  }

  return observation;
}

inline void close_driver_transport(
  DriverLifecycleTransport & transport,
  const DriverAccessPolicy & access_policy,
  bool connected)
{
  transport.release_callbacks();
  if (connected && access_policy.permits_shutdown_mode_change()) {
    transport.set_pose_mode();
  }
  transport.disconnect();
}
}  // namespace xarm_api
// *INDENT-ON*

#endif  // XARM_API__DRIVER_LIFECYCLE_H_
