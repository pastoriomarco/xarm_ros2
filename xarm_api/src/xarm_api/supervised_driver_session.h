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

#ifndef XARM_API__SUPERVISED_DRIVER_SESSION_H_
#define XARM_API__SUPERVISED_DRIVER_SESSION_H_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "xarm_api/supervised_driver.h"

// Keep the package's cpplint namespace convention despite its legacy
// uncrustify profile requesting the opposite indentation.
// *INDENT-OFF*
namespace xarm_api
{
using SupervisedDriverSessionConfig = SupervisedDriverConfig;

class SupervisedDriverTransport : public DriverLifecycleTransport
{
public:
  using ReportCallback = std::function<void(const SupervisedDriverReport &)>;
  using ConnectionCallback = std::function<void(bool, bool)>;

  ~SupervisedDriverTransport() override = default;

  virtual void set_report_callback(ReportCallback callback) = 0;
  virtual void set_connection_callback(ConnectionCallback callback) = 0;
  virtual bool connected() const = 0;
  virtual int read_joint_state(
    std::array<float, kSupervisedDriverMaximumJoints> & positions,
    std::array<float, kSupervisedDriverMaximumJoints> & velocities,
    std::size_t joint_count) = 0;
  virtual int write_joint_position_command(
    const std::array<float, kSupervisedDriverMaximumJoints> & positions,
    std::size_t joint_count) = 0;
  virtual int shutdown_controller() = 0;
};

/// One command-suppressed SDK transport with an NRT I/O worker.
///
/// Lifecycle primitives are serialized with the same SDK calls used by the
/// worker. Controller read/write users only consume/publish fixed-size atomic
/// snapshots; they never call the SDK or wait on the NRT mutex. Transport loss
/// is terminal for this object: the caller must replace the containing process
/// rather than reconnecting or re-arming the same SDK instance.
///
/// close() only tears down this software/transport session. It never invokes
/// the SDK's physical controller system_control(1) operation.
class SupervisedDriverSession
{
public:
  explicit SupervisedDriverSession(SupervisedDriverSessionConfig config);
  SupervisedDriverSession(
    SupervisedDriverSessionConfig config,
    std::unique_ptr<SupervisedDriverTransport> transport,
    bool start_io_worker);
  ~SupervisedDriverSession();

  SupervisedDriverSession(const SupervisedDriverSession &) = delete;
  SupervisedDriverSession & operator=(
    const SupervisedDriverSession &) = delete;

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
  /// Open command delivery only until the supplied steady-clock deadline.
  ///
  /// Passing false, a non-future deadline, a disconnected/unverified
  /// transport, an unqualified/stale joint source, or a shutting-down session
  /// closes the gate. Closing synchronizes with any SDK write already in
  /// progress before it returns.
  [[nodiscard]] bool set_command_gate(
    bool open, std::int64_t valid_until_ns);
  SupervisedControllerShutdownResult shutdown_controller();

  /// Perform one SDK poll/latest-command cycle on the calling NRT thread.
  ///
  /// Production construction starts the internal worker. This method is also
  /// the deterministic fake-SDK qualification seam.
  void service_io_once();
  void close() noexcept;

private:
  static std::int64_t steady_now_ns() noexcept;
  void initialize(bool start_io_worker);
  void handle_report(const SupervisedDriverReport & report);
  void handle_connection(bool connected, bool report_connected);
  void evaluate_transport_health(
    std::int64_t now_ns,
    bool transport_claimed_connected,
    bool joint_read_succeeded);
  void require_process_restart(bool control_lost, bool report_lost);
  void set_position_valid(bool valid, int return_code);
  void set_last_joint_write_return(int return_code);
  void update_shutdown_stationarity(
    const std::array<float, kSupervisedDriverMaximumJoints> & positions,
    std::int64_t sample_time_ns,
    bool source_agreement);
  void clear_shutdown_stationarity();
  bool close_command_gate_locked() noexcept;
  bool close_command_gate(bool synchronize_sdk);
  void run_io_worker();

  SupervisedDriverSessionConfig config_;
  DriverAccessPolicy access_policy_{DriverAccessMode::kSupervisedLifecycle};
  std::unique_ptr<SupervisedDriverTransport> transport_;

  mutable std::mutex observation_mutex_;
  SupervisedDriverObservation observation_;
  std::uint64_t next_observation_generation_ = 1;

  std::array<std::atomic<double>, kSupervisedDriverMaximumJoints>
  joint_positions_;
  std::array<std::atomic<double>, kSupervisedDriverMaximumJoints>
  joint_velocities_;
  std::atomic<std::uint64_t> joint_sequence_{0};
  std::atomic<std::uint64_t> joint_generation_{0};
  std::atomic<std::int64_t> joint_timestamp_ns_{0};
  std::atomic<bool> joint_valid_{false};
  std::atomic<bool> position_initialized_{false};
  std::atomic<std::size_t> position_initialization_match_count_{0};
  std::uint64_t last_position_initialization_report_generation_ = 0;
  std::int64_t loss_detection_started_ns_ = 0;
  std::int64_t last_joint_read_success_ns_ = 0;
  std::atomic<bool> startup_completed_{false};
  std::atomic<bool> process_restart_required_{false};
  std::atomic<std::uint64_t> joint_write_attempt_count_{0};
  std::atomic<std::uint64_t> lifecycle_command_attempt_count_{0};
  std::atomic<std::uint64_t> shutdown_controller_attempt_count_{0};
  bool position_ever_initialized_ = false;
  bool stationary_reference_valid_ = false;
  std::array<double, kSupervisedDriverMaximumJoints>
  stationary_reference_positions_{{0.0}};

  std::array<std::atomic<double>, kSupervisedDriverMaximumJoints>
  command_positions_;
  std::atomic<std::uint64_t> command_sequence_{0};
  std::atomic<std::uint64_t> consumed_command_sequence_{0};
  std::atomic<std::uint64_t> command_gate_version_{0};
  std::atomic<std::uint64_t> command_version_{0};
  std::atomic<std::int64_t> command_gate_valid_until_ns_{0};
  std::atomic<bool> command_gate_open_{false};

  std::mutex gate_control_mutex_;
  std::mutex sdk_mutex_;
  std::mutex worker_mutex_;
  std::condition_variable worker_condition_;
  std::thread io_worker_;
  std::atomic<bool> close_started_{false};
  bool transport_closed_ = false;
};
}  // namespace xarm_api
// *INDENT-ON*

#endif  // XARM_API__SUPERVISED_DRIVER_SESSION_H_
