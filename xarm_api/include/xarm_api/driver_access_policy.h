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

#ifndef XARM_API__DRIVER_ACCESS_POLICY_H_
#define XARM_API__DRIVER_ACCESS_POLICY_H_

#include <string>

// Keep the package's cpplint namespace convention despite its legacy
// uncrustify profile requesting the opposite indentation.
// *INDENT-OFF*
namespace xarm_api
{
enum class DriverAccessMode
{
  kLegacyControl,
  kReadOnly,
  kSupervisedLifecycle,
};

inline const char * driver_access_mode_name(DriverAccessMode mode)
{
  switch (mode) {
    case DriverAccessMode::kLegacyControl:
      return "legacy_control";
    case DriverAccessMode::kReadOnly:
      return "read_only";
    case DriverAccessMode::kSupervisedLifecycle:
      return "supervised_lifecycle";
  }
  return "unknown";
}

inline bool parse_driver_access_mode(
  const std::string & value, DriverAccessMode & mode)
{
  if (value == "legacy_control") {
    mode = DriverAccessMode::kLegacyControl;
    return true;
  }
  if (value == "read_only") {
    mode = DriverAccessMode::kReadOnly;
    return true;
  }
  if (value == "supervised_lifecycle") {
    mode = DriverAccessMode::kSupervisedLifecycle;
    return true;
  }
  return false;
}

class DriverAccessPolicy
{
public:
  DriverAccessPolicy()
  : mode_(DriverAccessMode::kLegacyControl)
  {
  }

  explicit DriverAccessPolicy(bool read_only)
  : mode_(
      read_only ? DriverAccessMode::kReadOnly :
      DriverAccessMode::kLegacyControl)
  {
  }

  explicit DriverAccessPolicy(DriverAccessMode mode)
  : mode_(mode)
  {
  }

  DriverAccessMode mode() const
  {
    return mode_;
  }

  bool is_read_only() const
  {
    return mode_ == DriverAccessMode::kReadOnly;
  }

  bool is_supervised_lifecycle() const
  {
    return mode_ == DriverAccessMode::kSupervisedLifecycle;
  }

  bool permits_command_endpoints() const
  {
    return mode_ == DriverAccessMode::kLegacyControl;
  }

  bool permits_automatic_fault_clear() const
  {
    return mode_ == DriverAccessMode::kLegacyControl;
  }

  bool permits_shutdown_mode_change() const
  {
    return mode_ == DriverAccessMode::kLegacyControl;
  }

  bool permits_gripper_actions() const
  {
    return mode_ == DriverAccessMode::kLegacyControl;
  }

  bool permits_supervised_lifecycle_commands() const
  {
    return mode_ == DriverAccessMode::kSupervisedLifecycle;
  }

private:
  DriverAccessMode mode_;
};
}  // namespace xarm_api
// *INDENT-ON*

#endif  // XARM_API__DRIVER_ACCESS_POLICY_H_
