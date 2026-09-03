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

// The expectations below are the ones autoware_utils_rclcpp pins for its own polling subscriber
// (autoware_utils_rclcpp/test/cases/polling_subscriber.cpp). They are written against the
// backend-agnostic interface, so the same expectations cover the ROS 2 and the agnocast backend.
//
// polling_policy::All has no counterpart here (take_data() returns a single message, not a
// vector); the static_assert in polling_subscriber.hpp rejects it at compile time, so the
// All-policy cases are not ported. last_taken_data_timestamp() has no counterpart either: the
// agnocast take path carries no source timestamp, so the wrapper does not expose the accessor.

#include "autoware/agnocast_wrapper/polling_subscriber.hpp"

#include "autoware/agnocast_wrapper/node.hpp"
#include "autoware/agnocast_wrapper/runtime.hpp"

#include <std_msgs/msg/string.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

namespace polling = autoware::agnocast_wrapper::polling;
using autoware::agnocast_wrapper::Node;
using std_msgs::msg::String;

constexpr auto discovery_timeout = std::chrono::seconds(10);
constexpr auto poll_interval = std::chrono::milliseconds(10);

/// agnocast exits the process from inside the subscription constructor when LD_PRELOAD lacks the
/// heaphook (validate_ld_preload() in agnocast_utils.cpp), which would take the whole test binary
/// down instead of failing one case. Probe the same condition so the test can skip instead.
bool agnocast_heaphook_loaded()
{
  const char * ld_preload = std::getenv("LD_PRELOAD");
  return ld_preload != nullptr &&
         std::string(ld_preload).find("libagnocast_heaphook.so") != std::string::npos;
}

class PollingSubscriberTest : public testing::Test
{
protected:
  void SetUp() override
  {
    if (autoware::agnocast_wrapper::use_agnocast() && !agnocast_heaphook_loaded()) {
      GTEST_SKIP() << "ENABLE_AGNOCAST=1 without the agnocast heaphook: the agnocast backend "
                      "cannot be exercised in this environment.";
    }
  }

  /// Publishing before the subscriber is matched drops the message on both backends, so wait for
  /// the match rather than republishing, which would leave extra messages queued behind the take.
  template <typename PublisherT>
  static bool wait_for_subscriber(const PublisherT & publisher)
  {
    const auto deadline = std::chrono::steady_clock::now() + discovery_timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (publisher->get_subscription_count() > 0) {
        return true;
      }
      std::this_thread::sleep_for(poll_interval);
    }
    return false;
  }

  /// Delivery is asynchronous, so poll until the message shows up instead of taking once.
  template <typename SubscriberT>
  static std::shared_ptr<const String> take_until_delivered(const SubscriberT & subscriber)
  {
    const auto deadline = std::chrono::steady_clock::now() + discovery_timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (const auto taken = subscriber->take_data()) {
        return taken;
      }
      std::this_thread::sleep_for(poll_interval);
    }
    return nullptr;
  }
};

TEST_F(PollingSubscriberTest, CheckQosDepthGreaterThanOneThrows)
{
  if (autoware::agnocast_wrapper::use_agnocast()) {
    // TODO(Koichi98): the agnocast backend accepts a deep QoS instead of rejecting it. The check
    // is in agnocast upstream but not in a released version yet; enable this once it is released.
    GTEST_SKIP() << "The agnocast backend does not reject QoS depth > 1 yet.";
  }

  const auto node = std::make_shared<Node>("test_check_qos_throw");

  EXPECT_THROW(
    polling::create_polling_subscriber<String>(node.get(), "/test/latest_deep", rclcpp::QoS{10}),
    std::invalid_argument);

  EXPECT_THROW(
    (polling::create_polling_subscriber<String, polling::polling_policy::Newest>(
      node.get(), "/test/newest_deep", rclcpp::QoS{10})),
    std::invalid_argument);
}

TEST_F(PollingSubscriberTest, CheckQosDepthOneDoesNotThrow)
{
  const auto node = std::make_shared<Node>("test_check_qos_no_throw");

  EXPECT_NO_THROW(
    polling::create_polling_subscriber<String>(node.get(), "/test/latest_shallow", rclcpp::QoS{1}));

  EXPECT_NO_THROW((polling::create_polling_subscriber<String, polling::polling_policy::Newest>(
    node.get(), "/test/newest_shallow", rclcpp::QoS{1})));
}

TEST_F(PollingSubscriberTest, InitialValues)
{
  const auto node = std::make_shared<Node>("test_initial_values");

  const auto latest_sub =
    polling::create_polling_subscriber<String>(node.get(), "/test/initial_latest", 1);
  EXPECT_EQ(latest_sub->take_data(), nullptr);

  const auto newest_sub =
    polling::create_polling_subscriber<String, polling::polling_policy::Newest>(
      node.get(), "/test/initial_newest", 1);
  EXPECT_EQ(newest_sub->take_data(), nullptr);
}

TEST_F(PollingSubscriberTest, PubSub)
{
  const auto pub_node = std::make_shared<Node>("pub_node");
  const auto sub_node = std::make_shared<Node>("sub_node");

  const auto pub = pub_node->create_publisher<String>("/test/text", rclcpp::QoS{1});
  const auto sub = polling::create_polling_subscriber<String>(sub_node.get(), "/test/text", 1);

  ASSERT_TRUE(wait_for_subscriber(pub));

  String pub_msg;
  pub_msg.data = "foo-bar";
  pub->publish(pub_msg);

  const auto sub_msg = take_until_delivered(sub);
  ASSERT_NE(sub_msg, nullptr);
  EXPECT_EQ(sub_msg->data, pub_msg.data);
}

TEST_F(PollingSubscriberTest, LatestRedeliversUntilANewerMessageArrives)
{
  const auto pub_node = std::make_shared<Node>("pub_node_latest");
  const auto sub_node = std::make_shared<Node>("sub_node_latest");

  const auto pub = pub_node->create_publisher<String>("/test/latest_retention", rclcpp::QoS{1});
  const auto sub =
    polling::create_polling_subscriber<String>(sub_node.get(), "/test/latest_retention", 1);

  ASSERT_TRUE(wait_for_subscriber(pub));

  String pub_msg;
  pub_msg.data = "test-message";
  pub->publish(pub_msg);

  const auto msg1 = take_until_delivered(sub);
  ASSERT_NE(msg1, nullptr);

  // Nothing was published in between, so Latest hands back the message it took last.
  const auto msg2 = sub->take_data();
  EXPECT_EQ(msg2, msg1);
}

TEST_F(PollingSubscriberTest, NewestReturnsNullWithoutNewMessage)
{
  const auto pub_node = std::make_shared<Node>("pub_node_newest");
  const auto sub_node = std::make_shared<Node>("sub_node_newest");

  const auto pub = pub_node->create_publisher<String>("/test/newest_clear", rclcpp::QoS{1});
  const auto sub = polling::create_polling_subscriber<String, polling::polling_policy::Newest>(
    sub_node.get(), "/test/newest_clear", 1);

  ASSERT_TRUE(wait_for_subscriber(pub));

  String pub_msg;
  pub_msg.data = "test-message";
  pub->publish(pub_msg);

  const auto msg1 = take_until_delivered(sub);
  ASSERT_NE(msg1, nullptr);

  // Nothing was published in between, so Newest reports that no new message arrived.
  EXPECT_EQ(sub->take_data(), nullptr);
}

}  // namespace
