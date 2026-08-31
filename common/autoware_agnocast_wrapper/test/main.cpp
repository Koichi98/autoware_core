// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#ifdef USE_AGNOCAST_ENABLED
#include <agnocast/agnocast.hpp>
#endif

namespace
{

/// @brief Brings up the same contexts a wrapper node's main() brings up.
class WrapperEnvironment : public testing::Environment
{
public:
  WrapperEnvironment(int argc, char ** argv) : argc_(argc), argv_(argv) {}

  void SetUp() override
  {
    rclcpp::init(argc_, argv_);
#ifdef USE_AGNOCAST_ENABLED
    // Only sets up the agnocast context: it opens no device and issues no ioctl, so it is safe
    // even where the kernel module and the heaphook are absent.
    agnocast::init(argc_, argv_);
#endif
  }

  void TearDown() override
  {
#ifdef USE_AGNOCAST_ENABLED
    agnocast::shutdown();
#endif
    rclcpp::shutdown();
  }

private:
  int argc_;
  char ** argv_;
};

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  testing::AddGlobalTestEnvironment(new WrapperEnvironment(argc, argv));
  return RUN_ALL_TESTS();
}
