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

#ifndef XARM_CONTROLLER__HARDWARE__SUPERVISED_LIFECYCLE_CONTRACT_H_
#define XARM_CONTROLLER__HARDWARE__SUPERVISED_LIFECYCLE_CONTRACT_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

#include "xarm_api/driver_lifecycle.h"

// Keep the package's cpplint namespace convention despite its legacy
// uncrustify profile requesting the opposite indentation.
// *INDENT-OFF*
namespace uf_robot_hardware
{
struct SupervisedCommandRecord
{
  std::string request_id;
  std::uint8_t command = 0;
  std::uint64_t expected_generation = 0;
  xarm_api::DriverLifecycleCommandResult result;
  std::uint64_t observed_generation = 0;
};

enum class SupervisedReplayDisposition
{
  kMiss,
  kReplay,
  kConflict,
};

enum class SupervisedReadDisposition
{
  kPublishSample,
  kHoldLastState,
  kFault,
};

enum class SupervisedWriteDisposition
{
  kDelivered,
  kFenced,
  kFault,
};

class SupervisedCommandReplayCache
{
public:
  explicit SupervisedCommandReplayCache(std::size_t capacity);

  SupervisedReplayDisposition lookup(
    const std::string & request_id,
    std::uint8_t command,
    std::uint64_t expected_generation,
    SupervisedCommandRecord & record) const;
  void remember(SupervisedCommandRecord record);
  void clear();

private:
  std::size_t capacity_;
  std::deque<SupervisedCommandRecord> records_;
};

bool valid_supervised_request_id(const std::string & request_id);

SupervisedReadDisposition supervised_read_disposition(
  bool driver_available,
  bool joint_sample_available);

SupervisedWriteDisposition supervised_write_disposition(
  bool driver_available,
  bool hardware_active,
  bool command_valid,
  bool submission_accepted);

bool map_supervised_command(
  std::uint8_t command,
  xarm_api::DriverLifecycleCommand & primitive);
}  // namespace uf_robot_hardware

#endif  // XARM_CONTROLLER__HARDWARE__SUPERVISED_LIFECYCLE_CONTRACT_H_
