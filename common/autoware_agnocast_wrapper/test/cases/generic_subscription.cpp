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

// Callback delivery is not asserted here: this binary spins no executor, as the other cases in
// this directory do not either. What is pinned instead is that both callback shapes produce a
// subscription a publisher on the topic matches, on whichever backend is active.

#include "autoware/agnocast_wrapper/generic_subscription.hpp"

#include "autoware/agnocast_wrapper/node.hpp"
#include "autoware/agnocast_wrapper/runtime.hpp"
#include "heaphook_probe.hpp"

#include <std_msgs/msg/string.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

namespace
{

using autoware::agnocast_wrapper::Node;
using autoware::agnocast_wrapper::test::agnocast_heaphook_loaded;

constexpr auto discovery_timeout = std::chrono::seconds(10);
constexpr auto poll_interval = std::chrono::milliseconds(10);
constexpr const char * string_type = "std_msgs/msg/String";

class GenericSubscriptionTest : public testing::Test
{
protected:
  void SetUp() override
  {
    if (autoware::agnocast_wrapper::use_agnocast() && !agnocast_heaphook_loaded()) {
      GTEST_SKIP() << "ENABLE_AGNOCAST=1 without the agnocast heaphook: the agnocast backend "
                      "cannot be exercised in this environment.";
    }
  }

  /// Both counts are needed: a same-process subscriber shows up in the intra-process count on the
  /// agnocast backend and in the other one on the ROS 2 backend.
  template <typename PublisherT>
  static bool wait_for_subscriber(const PublisherT & publisher)
  {
    const auto deadline = std::chrono::steady_clock::now() + discovery_timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (
        publisher->get_subscription_count() + publisher->get_intra_process_subscription_count() >
        0) {
        return true;
      }
      std::this_thread::sleep_for(poll_interval);
    }
    return false;
  }
};

TEST_F(GenericSubscriptionTest, arrival_callback_subscription_is_matched_by_a_publisher)
{
  auto node = std::make_shared<Node>("generic_subscription_arrival_matched");
  auto publisher =
    node->create_publisher<std_msgs::msg::String>("/generic_subscription/arrival", rclcpp::QoS{1});

  auto subscription = node->create_generic_subscription(
    "/generic_subscription/arrival", string_type, rclcpp::QoS{1}, [] {});

  EXPECT_TRUE(wait_for_subscriber(publisher));
}

TEST_F(GenericSubscriptionTest, serialized_callback_subscription_is_matched_by_a_publisher)
{
  auto node = std::make_shared<Node>("generic_subscription_serialized_matched");
  auto publisher = node->create_publisher<std_msgs::msg::String>(
    "/generic_subscription/serialized", rclcpp::QoS{1});

  auto subscription = node->create_generic_subscription(
    "/generic_subscription/serialized", string_type, rclcpp::QoS{1},
    [](std::shared_ptr<rclcpp::SerializedMessage>) {});

  EXPECT_TRUE(wait_for_subscriber(publisher));
}

TEST_F(GenericSubscriptionTest, arrival_callback_subscription_reports_topic_name_and_qos)
{
  auto node = std::make_shared<Node>("generic_subscription_arrival_reports");

  auto subscription = node->create_generic_subscription(
    "/absolute/topic", string_type, rclcpp::QoS{7}.transient_local(), [] {});

  EXPECT_STREQ(subscription->get_topic_name(), "/absolute/topic");
  EXPECT_EQ(subscription->get_actual_qos().depth(), 7u);
  EXPECT_EQ(subscription->get_actual_qos().durability(), rclcpp::DurabilityPolicy::TransientLocal);
}

TEST_F(GenericSubscriptionTest, arrival_callback_history_depth_overload_matches_the_qos_overload)
{
  auto node = std::make_shared<Node>("generic_subscription_arrival_depth");

  auto subscription = node->create_generic_subscription("/absolute/topic", string_type, 7u, [] {});

  EXPECT_EQ(subscription->get_actual_qos().depth(), 7u);
}

}  // namespace
