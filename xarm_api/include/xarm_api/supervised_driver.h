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

#ifndef XARM_API__SUPERVISED_DRIVER_H_
#define XARM_API__SUPERVISED_DRIVER_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "xarm_api/driver_lifecycle.h"

// Keep the package's cpplint namespace convention despite its legacy
// uncrustify profile requesting the opposite indentation.
// *INDENT-OFF*
namespace xarm_api
{
constexpr std::size_t kSupervisedDriverMaximumJoints = 7;

struct SupervisedDriverReport
{
  int state = -1;
  int mode = -1;
  int command_count = -1;
  int brake_mask = -1;
  int servo_enable_mask = -1;
  int error_code = -1;
  int warning_code = -1;
  std::array<double, kSupervisedDriverMaximumJoints>
  joint_positions{{0.0}};
};

struct SupervisedDriverObservation
{
  std::uint64_t generation = 0;
  std::int64_t source_timestamp_ns = 0;
  std::int64_t fresh_until_ns = 0;
  bool connected = false;
  bool report_connected = false;
  bool identity_verified = false;
  bool process_restart_required = false;
  bool report_received = false;
  bool position_valid = false;
  bool position_ever_initialized = false;
  bool stationary = false;
  std::int64_t stationary_since_ns = 0;
  bool command_gate_open = false;
  std::int64_t command_gate_valid_until_ns = 0;
  std::uint64_t report_sample_count = 0;
  std::uint64_t joint_read_attempt_count = 0;
  std::uint64_t joint_read_success_count = 0;
  std::uint64_t joint_write_attempt_count = 0;
  std::uint64_t lifecycle_command_attempt_count = 0;
  std::uint64_t shutdown_controller_attempt_count = 0;
  DriverRobotIdentity identity;
  SupervisedDriverReport report;
  int last_joint_read_return_code = -1;
  int last_joint_write_return_code = 0;
};

struct SupervisedDriverJointState
{
  std::uint64_t generation = 0;
  std::int64_t source_timestamp_ns = 0;
  std::size_t joint_count = 0;
  std::array<double, kSupervisedDriverMaximumJoints> positions{{0.0}};
  std::array<double, kSupervisedDriverMaximumJoints> velocities{{0.0}};
};

struct SupervisedDriverConfig
{
  std::string robot_address;
  std::string report_type = "rich";
  std::size_t joint_count = 0;
  DriverIdentityExpectation identity_expectation;
  std::int64_t observation_lease_ns = 250000000;
  std::int64_t joint_state_lease_ns = 100000000;
  std::int64_t io_period_ns = 5000000;
  std::int64_t transport_loss_timeout_ns = 2000000000;
  double source_agreement_tolerance_rad = 0.002;
  std::size_t position_initialization_samples = 3;
  double shutdown_stationary_tolerance_rad = 0.001;
  std::int64_t shutdown_stationary_dwell_ns = 1000000000;
};

struct SupervisedControllerShutdownResult
{
  bool permitted = false;
  bool attempted = false;
  int return_code = -1;
  const char * reason = "not_evaluated";
  bool process_restart_required = false;
};

/// Installed xarm_ros2 façade for the private supervised SDK session.
///
/// This class exposes bounded state, primitive, gate, and joint-cache
/// operations to the xarm_ros2 hardware plugin. It does not expose the vendor
/// SDK object, transport, callbacks, or reconnect behavior. Transport loss is
/// terminal for an instance and requires replacement of the containing
/// controller-manager process.
class SupervisedDriver
{
public:
  explicit SupervisedDriver(SupervisedDriverConfig config);
  ~SupervisedDriver();

  SupervisedDriver(const SupervisedDriver &) = delete;
  SupervisedDriver & operator=(const SupervisedDriver &) = delete;

  [[nodiscard]] bool ready() const noexcept;
  [[nodiscard]] SupervisedDriverObservation observe(
    std::int64_t now_ns) const;
  DriverLifecycleCommandResult execute_lifecycle(
    const DriverLifecycleCommand & command);
  [[nodiscard]] bool read_joint_state(
    SupervisedDriverJointState & output) const noexcept;
  [[nodiscard]] bool submit_joint_position_command(
    const std::array<double, kSupervisedDriverMaximumJoints> & positions,
    std::size_t joint_count) noexcept;
  [[nodiscard]] bool set_command_gate(
    bool open, std::int64_t valid_until_ns);
  /// Request the fixed physical-controller shutdown operation.
  ///
  /// The xarm_ros2 owner must enforce the higher-level ROS preconditions
  /// before calling this method. Any attempted call terminally fences this
  /// session and requires a fresh owner process.
  SupervisedControllerShutdownResult shutdown_controller();
  void close() noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace xarm_api

#endif  // XARM_API__SUPERVISED_DRIVER_H_
