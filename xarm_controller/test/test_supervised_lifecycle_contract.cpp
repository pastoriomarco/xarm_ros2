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
#include <cstdint>
#include <string>

#include "xarm_controller/hardware/supervised_lifecycle_contract.h"
#include "xarm_msgs/srv/execute_supervised_lifecycle_command.hpp"

namespace
{
using uf_robot_hardware::SupervisedCommandRecord;
using uf_robot_hardware::SupervisedCommandReplayCache;
using uf_robot_hardware::SupervisedReadDisposition;
using uf_robot_hardware::SupervisedReplayDisposition;
using uf_robot_hardware::SupervisedWriteDisposition;

TEST(SupervisedLifecycleContract, HoldsInvalidPositionWhileOwnerCanFence)
{
  EXPECT_EQ(
    uf_robot_hardware::supervised_read_disposition(true, false),
    SupervisedReadDisposition::kHoldLastState);
  EXPECT_EQ(
    uf_robot_hardware::supervised_read_disposition(false, false),
    SupervisedReadDisposition::kFault);
}

TEST(SupervisedLifecycleContract, PublishesValidPosition)
{
  EXPECT_EQ(
    uf_robot_hardware::supervised_read_disposition(true, true),
    SupervisedReadDisposition::kPublishSample);
}

TEST(SupervisedLifecycleContract, HoldsFencedCommandWithoutHardwareFault)
{
  EXPECT_EQ(
    uf_robot_hardware::supervised_write_disposition(
      true, true, true, false),
    SupervisedWriteDisposition::kFenced);
  EXPECT_EQ(
    uf_robot_hardware::supervised_write_disposition(
      true, true, true, true),
    SupervisedWriteDisposition::kDelivered);
}

TEST(SupervisedLifecycleContract, FaultsInvalidWritePreconditions)
{
  EXPECT_EQ(
    uf_robot_hardware::supervised_write_disposition(
      false, true, true, false),
    SupervisedWriteDisposition::kFault);
  EXPECT_EQ(
    uf_robot_hardware::supervised_write_disposition(
      true, false, true, false),
    SupervisedWriteDisposition::kFault);
  EXPECT_EQ(
    uf_robot_hardware::supervised_write_disposition(
      true, true, false, false),
    SupervisedWriteDisposition::kFault);
}

TEST(SupervisedLifecycleContract, MapsOnlySevenNamedPrimitives)
{
  using Request =
    xarm_msgs::srv::ExecuteSupervisedLifecycleCommand::Request;
  const std::array<std::uint8_t, 7> commands{{
    Request::CLEAR_ERROR,
    Request::CLEAR_WARNING,
    Request::ENABLE_MOTION,
    Request::DISABLE_MOTION,
    Request::SELECT_JOINT_SERVO_MODE,
    Request::SET_READY_STATE,
    Request::SET_STOPPED_STATE,
  }};
  for (const auto command : commands) {
    xarm_api::DriverLifecycleCommand primitive;
    EXPECT_TRUE(
      uf_robot_hardware::map_supervised_command(
        command, primitive));
    EXPECT_EQ(primitive.servo_id, 8);
  }

  xarm_api::DriverLifecycleCommand primitive;
  EXPECT_FALSE(uf_robot_hardware::map_supervised_command(0, primitive));
  EXPECT_FALSE(uf_robot_hardware::map_supervised_command(255, primitive));
}

TEST(SupervisedLifecycleContract, UsesFixedValuesInsteadOfRawSdkArguments)
{
  using Request =
    xarm_msgs::srv::ExecuteSupervisedLifecycleCommand::Request;
  xarm_api::DriverLifecycleCommand primitive;

  ASSERT_TRUE(uf_robot_hardware::map_supervised_command(
      Request::ENABLE_MOTION, primitive));
  EXPECT_EQ(
    primitive.kind,
    xarm_api::DriverLifecycleCommandKind::kSetMotionEnabled);
  EXPECT_EQ(primitive.value, 1);

  ASSERT_TRUE(uf_robot_hardware::map_supervised_command(
      Request::DISABLE_MOTION, primitive));
  EXPECT_EQ(primitive.value, 0);
  ASSERT_TRUE(uf_robot_hardware::map_supervised_command(
      Request::SELECT_JOINT_SERVO_MODE, primitive));
  EXPECT_EQ(primitive.value, 1);
  ASSERT_TRUE(uf_robot_hardware::map_supervised_command(
      Request::SET_READY_STATE, primitive));
  EXPECT_EQ(primitive.value, 0);
  ASSERT_TRUE(uf_robot_hardware::map_supervised_command(
      Request::SET_STOPPED_STATE, primitive));
  EXPECT_EQ(primitive.value, 4);
}

TEST(SupervisedLifecycleContract, RequestIdsAreBoundedPrintableTokens)
{
  EXPECT_TRUE(
    uf_robot_hardware::valid_supervised_request_id(
      "session-4:request-9:step-0"));
  EXPECT_FALSE(uf_robot_hardware::valid_supervised_request_id(""));
  EXPECT_FALSE(
    uf_robot_hardware::valid_supervised_request_id(
      std::string(129, 'a')));
  EXPECT_FALSE(
    uf_robot_hardware::valid_supervised_request_id("contains space"));
  EXPECT_FALSE(
    uf_robot_hardware::valid_supervised_request_id(
      std::string("line\nbreak")));
}

TEST(SupervisedLifecycleContract, ExactDuplicateReplaysWithoutReattempt)
{
  SupervisedCommandReplayCache cache(2);
  SupervisedCommandRecord original;
  original.request_id = "request-1";
  original.command = 3;
  original.expected_generation = 17;
  original.result.permitted = true;
  original.result.attempted = true;
  original.result.return_code = 0;
  original.result.reason = "ok";
  original.observed_generation = 18;
  cache.remember(original);

  SupervisedCommandRecord replay;
  EXPECT_EQ(
    cache.lookup("request-1", 3, 17, replay),
    SupervisedReplayDisposition::kReplay);
  EXPECT_TRUE(replay.result.attempted);
  EXPECT_EQ(replay.observed_generation, 18U);
  EXPECT_EQ(
    cache.lookup("request-1", 4, 17, replay),
    SupervisedReplayDisposition::kConflict);
  EXPECT_EQ(
    cache.lookup("request-1", 3, 18, replay),
    SupervisedReplayDisposition::kConflict);
  EXPECT_EQ(
    cache.lookup("request-2", 3, 17, replay),
    SupervisedReplayDisposition::kMiss);
}

TEST(SupervisedLifecycleContract, ReplayHistoryIsBounded)
{
  SupervisedCommandReplayCache cache(2);
  for (std::uint64_t index = 1; index <= 3; ++index) {
    SupervisedCommandRecord record;
    record.request_id = "request-" + std::to_string(index);
    record.command = 3;
    record.expected_generation = index;
    cache.remember(record);
  }
  SupervisedCommandRecord found;
  EXPECT_EQ(
    cache.lookup("request-1", 3, 1, found),
    SupervisedReplayDisposition::kMiss);
  EXPECT_EQ(
    cache.lookup("request-2", 3, 2, found),
    SupervisedReplayDisposition::kReplay);
  cache.clear();
  EXPECT_EQ(
    cache.lookup("request-2", 3, 2, found),
    SupervisedReplayDisposition::kMiss);
}
}  // namespace
