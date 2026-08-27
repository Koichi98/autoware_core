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

#pragma once

#include "autoware/agnocast_wrapper/node.hpp"

#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <utility>

#ifdef USE_AGNOCAST_ENABLED
#include <agnocast/agnocast.hpp>
#endif

namespace autoware::agnocast_wrapper::polling
{

namespace polling_policy = autoware_utils_rclcpp::polling_policy;

template <typename MessageT, template <typename> class PollingPolicy>
inline constexpr bool polling_keeps_last_data_v =
  !std::is_same_v<PollingPolicy<MessageT>, polling_policy::Newest<MessageT>>;

template <typename MessageT, template <typename> class PollingPolicy>
inline constexpr bool polling_policy_supported_v =
  !std::is_same_v<PollingPolicy<MessageT>, polling_policy::All<MessageT>>;

/// @brief Backend-agnostic polling subscriber. take_data() returns a plain
/// std::shared_ptr<const MessageT> regardless of ENABLE_AGNOCAST.
template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
class PollingSubscriber
{
public:
  static_assert(
    polling_policy_supported_v<MessageT, PollingPolicy>,
    "polling_policy::All is not supported by "
    "autoware::agnocast_wrapper::polling::create_polling_subscriber "
    "(take_data() returns a single message, not a vector). Use polling_policy::Latest or "
    "polling_policy::Newest.");

  using SharedPtr = std::shared_ptr<PollingSubscriber<MessageT, PollingPolicy>>;

  virtual ~PollingSubscriber() = default;

  std::shared_ptr<const MessageT> take_data()
  {
    return take_data_impl(polling_keeps_last_data_v<MessageT, PollingPolicy>);
  }

protected:
  virtual std::shared_ptr<const MessageT> take_data_impl(bool keeps_last_data) = 0;
};

template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
class ROS2PollingSubscriber : public PollingSubscriber<MessageT, PollingPolicy>
{
  typename autoware_utils_rclcpp::InterProcessPollingSubscriber<MessageT, PollingPolicy>::SharedPtr
    subscriber_;

public:
  explicit ROS2PollingSubscriber(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
  : subscriber_(
      autoware_utils_rclcpp::InterProcessPollingSubscriber<
        MessageT, PollingPolicy>::create_subscription(node, topic_name, qos))
  {
  }

  // keeps_last_data is unused: upstream autoware_utils_rclcpp has no take_data(bool);
  // re-delivery is already governed by the PollingPolicy tag (Latest re-delivers, Newest only new).
  std::shared_ptr<const MessageT> take_data_impl(bool keeps_last_data) override
  {
    (void)keeps_last_data;
    return subscriber_->take_data();
  }
};

#ifdef USE_AGNOCAST_ENABLED

template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
class AgnocastPollingSubscriber : public PollingSubscriber<MessageT, PollingPolicy>
{
  typename agnocast::TakeSubscription<MessageT>::SharedPtr subscriber_;

  std::shared_ptr<const MessageT> last_data_;
  std::mutex last_data_mtx_;

public:
  explicit AgnocastPollingSubscriber(
    agnocast::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
  : subscriber_(std::make_shared<agnocast::TakeSubscription<MessageT>>(node, topic_name, qos))
  {
  }

  std::shared_ptr<const MessageT> take_data_impl(bool keeps_last_data) override
  {
    agnocast::ipc_shared_ptr<const MessageT> data = subscriber_->take(false);

    std::shared_ptr<const MessageT> new_data;
    if (data) {
      // Zero-copy: alias the shared-memory message into the returned std::shared_ptr. `holder`
      // keeps the ipc_shared_ptr alive for the returned pointer's lifetime, so lifetime/refcount
      // match the rclcpp heap path. While any copy is alive it pins one agnocast shared-memory
      // entry.
      auto holder = std::make_shared<agnocast::ipc_shared_ptr<const MessageT>>(std::move(data));
      new_data = std::shared_ptr<const MessageT>(holder, holder->get());
    }

    if (!keeps_last_data) {
      return new_data;
    }

    // Declared outside the lock so that dropping the previous message, which may release the
    // kernel-side reference, happens after the mutex is released.
    std::shared_ptr<const MessageT> old_data;
    {
      std::lock_guard<std::mutex> lock(last_data_mtx_);
      if (new_data) {
        old_data = std::move(last_data_);
        last_data_ = std::move(new_data);
      }
      return last_data_;
    }
  }
};

/// @note The returned subscriber references the node's backend by raw pointer, so it must not
/// outlive @p node.
template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
typename PollingSubscriber<MessageT, PollingPolicy>::SharedPtr create_polling_subscriber(
  autoware::agnocast_wrapper::Node * node, const std::string & topic_name,
  const rclcpp::QoS & qos = rclcpp::QoS{1})
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastPollingSubscriber<MessageT, PollingPolicy>>(
      node->get_agnocast_node().get(), topic_name, qos);
  }
  return std::make_shared<ROS2PollingSubscriber<MessageT, PollingPolicy>>(
    node->get_rclcpp_node().get(), topic_name, qos);
}

#else  // USE_AGNOCAST_ENABLED

/// @note The returned subscriber references the node's rclcpp node by raw pointer, so it must not
/// outlive @p node.
template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
typename PollingSubscriber<MessageT, PollingPolicy>::SharedPtr create_polling_subscriber(
  autoware::agnocast_wrapper::Node * node, const std::string & topic_name,
  const rclcpp::QoS & qos = rclcpp::QoS{1})
{
  return std::make_shared<ROS2PollingSubscriber<MessageT, PollingPolicy>>(
    node->get_rclcpp_node().get(), topic_name, qos);
}

#endif  // USE_AGNOCAST_ENABLED

template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
typename PollingSubscriber<MessageT, PollingPolicy>::SharedPtr create_polling_subscriber(
  autoware::agnocast_wrapper::Node * node, const std::string & topic_name, size_t qos_history_depth)
{
  return create_polling_subscriber<MessageT, PollingPolicy>(
    node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
}

}  // namespace autoware::agnocast_wrapper::polling
