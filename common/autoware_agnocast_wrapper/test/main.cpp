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

namespace
{

/// @brief Initializes the context the way a mixed-mode executable does: exactly one of the rclcpp
/// and the agnocast context is brought up, and only an AgnocastOnly executable brings up the
/// agnocast one. See templates/node_main_switchable.cpp.in.
class ContextEnvironment : public testing::Environment
{
public:
  ContextEnvironment(int argc, char ** argv) : argc_(argc), argv_(argv) {}

  void SetUp() override { rclcpp::init(argc_, argv_); }
  void TearDown() override { rclcpp::shutdown(); }

private:
  int argc_;
  char ** argv_;
};

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  testing::AddGlobalTestEnvironment(new ContextEnvironment(argc, argv));
  return RUN_ALL_TESTS();
}
