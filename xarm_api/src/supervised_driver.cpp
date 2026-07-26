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

#include "xarm_api/supervised_driver.h"

#include <utility>

#include "xarm_api/supervised_driver_session.h"

namespace xarm_api
{
class SupervisedDriver::Impl
{
public:
  explicit Impl(SupervisedDriverConfig config)
  : session(std::move(config))
  {
  }

  SupervisedDriverSession session;
};

SupervisedDriver::SupervisedDriver(SupervisedDriverConfig config)
: impl_(new Impl(std::move(config)))
{
}

SupervisedDriver::~SupervisedDriver() = default;

bool SupervisedDriver::ready() const noexcept
{
  return impl_->session.ready();
}

SupervisedDriverObservation SupervisedDriver::observe(
  std::int64_t now_ns) const
{
  return impl_->session.observe(now_ns);
}

DriverLifecycleCommandResult SupervisedDriver::execute_lifecycle(
  const DriverLifecycleCommand & command)
{
  return impl_->session.execute_lifecycle(command);
}

bool SupervisedDriver::read_joint_state(
  SupervisedDriverJointState & output) const noexcept
{
  return impl_->session.read_joint_state(output);
}

bool SupervisedDriver::submit_joint_position_command(
  const std::array<double, kSupervisedDriverMaximumJoints> & positions,
  std::size_t joint_count) noexcept
{
  return impl_->session.submit_joint_position_command(
    positions, joint_count);
}

bool SupervisedDriver::set_command_gate(
  bool open, std::int64_t valid_until_ns)
{
  return impl_->session.set_command_gate(open, valid_until_ns);
}

SupervisedControllerShutdownResult SupervisedDriver::shutdown_controller()
{
  return impl_->session.shutdown_controller();
}

void SupervisedDriver::close() noexcept
{
  impl_->session.close();
}
}  // namespace xarm_api
