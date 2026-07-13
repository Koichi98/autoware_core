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

#include <rclcpp/qos.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/qos.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/version.h>

#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/node/tf2/buffer.hpp>
#include <agnocast/node/tf2/static_transform_broadcaster.hpp>
#include <agnocast/node/tf2/transform_broadcaster.hpp>
#include <agnocast/node/tf2/transform_listener.hpp>

namespace autoware::agnocast_wrapper
{

/// @brief Buffer alias — agnocast::Buffer here, tf2_ros::Buffer in the disabled build.
///        Unlike the listener / broadcaster wrappers, there is no runtime dispatch; the
///        choice is fixed at build time, and agnocast::Buffer omits APIs that would break
///        under an AgnocastOnly executor (e.g. waitForTransform) so misuse fails to compile.
// TODO(Koichi98): agnocast::Buffer currently does not implement waitForTransform, so it has no
// dependency on the agnocast executor — that lets us alias it directly here, which surfaces
// the AgnocastOnly-safety constraint at compile time (the async API simply does not exist
// on the wrapper type). Once waitForTransform is added, this alias must become a
// runtime-dispatched wrapper class.
using Buffer = agnocast::Buffer;

/// @brief Wrapper TransformListener that switches between tf2_ros and agnocast
///        TransformListener implementations at runtime.
///
/// @invariant The backend is selected from use_agnocast() at construction and never
///            changes, so the held impl's identity is stable for the wrapper's lifetime.
///
/// The node-taking constructors require a Method 2 node (autoware::agnocast_wrapper::Node)
/// because an AgnocastOnly executor does not spin a plain tf2_ros::TransformListener; in
/// Agnocast-enabled mode the agnocast backend must own the /tf subscription instead.
/// See the README for a usage example.
class TransformListener
{
public:
  using RclcppImpl = std::unique_ptr<tf2_ros::TransformListener>;
  using AgnocastImpl = std::unique_ptr<agnocast::TransformListener>;

  TransformListener(
    tf2::BufferCore & buffer, Node & node, bool spin_thread = true,
    const rclcpp::QoS & qos = tf2_ros::DynamicListenerQoS(),
    const rclcpp::QoS & static_qos = tf2_ros::StaticListenerQoS())
  : impl_(
      use_agnocast() ? decltype(impl_)(
                         std::in_place_type<AgnocastImpl>,
                         std::make_unique<agnocast::TransformListener>(
                           buffer, *node.get_agnocast_node(), spin_thread, qos, static_qos))
                     : decltype(impl_)(
                         std::in_place_type<RclcppImpl>,
                         std::make_unique<tf2_ros::TransformListener>(
                           buffer, node.get_rclcpp_node().get(), spin_thread, qos, static_qos)))
  {
  }

  /// @brief Options-taking overload. Kept separate (not defaulted parameters) so the
  ///        no-options ctor preserves each backend's own non-trivial option defaults.
  ///        On the rclcpp path only the subset shared with agnocast::SubscriptionOptions
  ///        is forwarded; other rclcpp fields stay at their rclcpp defaults.
  TransformListener(
    tf2::BufferCore & buffer, Node & node, bool spin_thread, const rclcpp::QoS & qos,
    const rclcpp::QoS & static_qos, const AUTOWARE_SUBSCRIPTION_OPTIONS & options,
    const AUTOWARE_SUBSCRIPTION_OPTIONS & static_options)
  : impl_(
      use_agnocast()
        ? decltype(impl_)(
            std::in_place_type<AgnocastImpl>, std::make_unique<agnocast::TransformListener>(
                                                buffer, *node.get_agnocast_node(), spin_thread, qos,
                                                static_qos, options, static_options))
        : [&] {
            rclcpp::SubscriptionOptions ros2_options;
            ros2_options.callback_group = options.callback_group;
            ros2_options.qos_overriding_options = options.qos_overriding_options;
            ros2_options.ignore_local_publications = options.ignore_local_publications;
            rclcpp::SubscriptionOptions ros2_static_options;
            ros2_static_options.callback_group = static_options.callback_group;
            ros2_static_options.qos_overriding_options = static_options.qos_overriding_options;
            ros2_static_options.ignore_local_publications =
              static_options.ignore_local_publications;
            return decltype(impl_)(
              std::in_place_type<RclcppImpl>,
              std::make_unique<tf2_ros::TransformListener>(
                buffer, node.get_rclcpp_node().get(), spin_thread, qos, static_qos, ros2_options,
                ros2_static_options));
          }())
  {
  }

  /// @brief Buffer-only constructor; the underlying listener creates its own node internally.
  explicit TransformListener(tf2::BufferCore & buffer, bool spin_thread = true)
  : impl_(
      use_agnocast() ? decltype(impl_)(
                         std::in_place_type<AgnocastImpl>,
                         std::make_unique<agnocast::TransformListener>(buffer, spin_thread))
                     : decltype(impl_)(
                         std::in_place_type<RclcppImpl>,
                         std::make_unique<tf2_ros::TransformListener>(buffer, spin_thread)))
  {
  }

  // Non-copyable and non-movable: matches the wrapped tf2_ros / agnocast types, which own
  // subscriptions bound to the supplied node at construction.
  TransformListener(const TransformListener &) = delete;
  TransformListener & operator=(const TransformListener &) = delete;
  TransformListener(TransformListener &&) = delete;
  TransformListener & operator=(TransformListener &&) = delete;

private:
  std::variant<RclcppImpl, AgnocastImpl> impl_;
};

/// @brief Wrapper TransformBroadcaster that switches between tf2_ros and agnocast
///        TransformBroadcaster implementations at runtime.
///
/// Same backend-fixed-at-construction guarantee as TransformListener. Requires a
/// Method 2 node so the agnocast backend can own the /tf publisher under an
/// AgnocastOnly executor. sendTransform() dispatches via std::visit to the active impl.
class TransformBroadcaster
{
public:
  using RclcppImpl = std::unique_ptr<tf2_ros::TransformBroadcaster>;
  using AgnocastImpl = std::unique_ptr<agnocast::TransformBroadcaster>;

  explicit TransformBroadcaster(
    Node & node, const rclcpp::QoS & qos = tf2_ros::DynamicBroadcasterQoS())
  : impl_(
      use_agnocast()
        ? decltype(impl_)(
            std::in_place_type<AgnocastImpl>,
            std::make_unique<agnocast::TransformBroadcaster>(*node.get_agnocast_node(), qos))
        : decltype(impl_)(
            std::in_place_type<RclcppImpl>,
            std::make_unique<tf2_ros::TransformBroadcaster>(node.get_rclcpp_node().get(), qos)))
  {
  }

  /// @brief Options-taking overload; same separate-overload rationale as TransformListener.
  ///        Only qos_overriding_options crosses to the rclcpp side (the shared field).
  TransformBroadcaster(
    Node & node, const rclcpp::QoS & qos, const AUTOWARE_PUBLISHER_OPTIONS & options)
  : impl_(
      use_agnocast()
        ? decltype(impl_)(
            std::in_place_type<AgnocastImpl>, std::make_unique<agnocast::TransformBroadcaster>(
                                                *node.get_agnocast_node(), qos, options))
        : [&] {
            rclcpp::PublisherOptions ros2_options;
            ros2_options.qos_overriding_options = options.qos_overriding_options;
            return decltype(impl_)(
              std::in_place_type<RclcppImpl>, std::make_unique<tf2_ros::TransformBroadcaster>(
                                                node.get_rclcpp_node().get(), qos, ros2_options));
          }())
  {
  }

  // Non-copyable and non-movable: matches the wrapped types, which own the /tf publisher
  // bound to the supplied node at construction.
  TransformBroadcaster(const TransformBroadcaster &) = delete;
  TransformBroadcaster & operator=(const TransformBroadcaster &) = delete;
  TransformBroadcaster(TransformBroadcaster &&) = delete;
  TransformBroadcaster & operator=(TransformBroadcaster &&) = delete;

  void sendTransform(const geometry_msgs::msg::TransformStamped & transform)
  {
    std::visit([&](auto & impl) { impl->sendTransform(transform); }, impl_);
  }

  void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
  {
    std::visit([&](auto & impl) { impl->sendTransform(transforms); }, impl_);
  }

private:
  std::variant<RclcppImpl, AgnocastImpl> impl_;
};

/// @brief Wrapper StaticTransformBroadcaster mirroring TransformBroadcaster, but for
///        /tf_static. Same backend-fixed-at-construction guarantee and Method-2-node
///        requirement; the options-taking ctor has a Humble-specific intra-process tweak
///        (see the inline comment on that branch).
class StaticTransformBroadcaster
{
public:
  using RclcppImpl = std::unique_ptr<tf2_ros::StaticTransformBroadcaster>;
  using AgnocastImpl = std::unique_ptr<agnocast::StaticTransformBroadcaster>;

  explicit StaticTransformBroadcaster(
    Node & node, const rclcpp::QoS & qos = tf2_ros::StaticBroadcasterQoS())
  : impl_(
      use_agnocast()
        ? decltype(impl_)(
            std::in_place_type<AgnocastImpl>,
            std::make_unique<agnocast::StaticTransformBroadcaster>(*node.get_agnocast_node(), qos))
        : decltype(impl_)(
            std::in_place_type<RclcppImpl>, std::make_unique<tf2_ros::StaticTransformBroadcaster>(
                                              node.get_rclcpp_node().get(), qos)))
  {
  }

  /// @brief Options-taking overload; same rationale as TransformBroadcaster's. Humble adds
  ///        an intra-process tweak on the rclcpp side (see the #if branch below).
  StaticTransformBroadcaster(
    Node & node, const rclcpp::QoS & qos, const AUTOWARE_PUBLISHER_OPTIONS & options)
  : impl_(
      use_agnocast()
        ? decltype(impl_)(
            std::in_place_type<AgnocastImpl>,
            std::make_unique<agnocast::StaticTransformBroadcaster>(
              *node.get_agnocast_node(), qos, options))
        : [&] {
            rclcpp::PublisherOptions ros2_options;
            ros2_options.qos_overriding_options = options.qos_overriding_options;
#if RCLCPP_VERSION_MAJOR < 28
            // Humble: rclcpp intra-process doesn't support transient_local, so /tf_static needs
            // it off. tf2_ros::StaticTransformBroadcaster did this in its default options
            // pre-Jazzy; Jazzy (rclcpp 28+) removed it.
            ros2_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
#endif
            return decltype(impl_)(
              std::in_place_type<RclcppImpl>, std::make_unique<tf2_ros::StaticTransformBroadcaster>(
                                                node.get_rclcpp_node().get(), qos, ros2_options));
          }())
  {
  }

  // Non-copyable and non-movable: matches the wrapped types, which own the /tf_static
  // publisher bound to the supplied node at construction.
  StaticTransformBroadcaster(const StaticTransformBroadcaster &) = delete;
  StaticTransformBroadcaster & operator=(const StaticTransformBroadcaster &) = delete;
  StaticTransformBroadcaster(StaticTransformBroadcaster &&) = delete;
  StaticTransformBroadcaster & operator=(StaticTransformBroadcaster &&) = delete;

  void sendTransform(const geometry_msgs::msg::TransformStamped & transform)
  {
    std::visit([&](auto & impl) { impl->sendTransform(transform); }, impl_);
  }

  void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
  {
    std::visit([&](auto & impl) { impl->sendTransform(transforms); }, impl_);
  }

private:
  std::variant<RclcppImpl, AgnocastImpl> impl_;
};

}  // namespace autoware::agnocast_wrapper

#else

namespace autoware::agnocast_wrapper
{

// Agnocast-disabled build: thin composition wrappers over the tf2_ros types. Constructor
// signatures match the agnocast-enabled build above.

/// @brief Curated Buffer for the non-Agnocast build.
///
/// Holds a tf2_ros::Buffer by value and forwards only the curated member set (construction,
/// lookupTransform, canTransform), instead of deriving from it. This keeps the public surface
/// identical to agnocast::Buffer used in the Agnocast build — both omit AsyncBufferInterface
/// (waitForTransform / cancel / setCreateTimerInterface), which would deadlock under an
/// AgnocastOnly executor that does not spin the timer the async wait relies on. Deriving from
/// tf2_ros::Buffer would instead leak the full API (including the async members) via an implicit
/// upcast to tf2_ros::Buffer& / tf2::BufferCore&, allowing =0-only code that breaks under =1.
class Buffer
{
public:
  explicit Buffer(
    rclcpp::Clock::SharedPtr clock,
    tf2::Duration cache_time = tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME),
    rclcpp::Node::SharedPtr node = rclcpp::Node::SharedPtr())
  : buffer_(std::move(clock), cache_time, std::move(node))
  {
  }

  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration timeout) const
  {
    return buffer_.lookupTransform(target_frame, source_frame, time, timeout);
  }

  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    const rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(0)) const
  {
    return buffer_.lookupTransform(target_frame, source_frame, time, timeout);
  }

  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame, const tf2::TimePoint & target_time,
    const std::string & source_frame, const tf2::TimePoint & source_time,
    const std::string & fixed_frame, const tf2::Duration timeout) const
  {
    return buffer_.lookupTransform(
      target_frame, target_time, source_frame, source_time, fixed_frame, timeout);
  }

  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame, const rclcpp::Time & target_time,
    const std::string & source_frame, const rclcpp::Time & source_time,
    const std::string & fixed_frame,
    const rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(0)) const
  {
    return buffer_.lookupTransform(
      target_frame, target_time, source_frame, source_time, fixed_frame, timeout);
  }

  bool canTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration timeout, std::string * errstr = nullptr) const
  {
    return buffer_.canTransform(target_frame, source_frame, time, timeout, errstr);
  }

  bool canTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    const rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(0),
    std::string * errstr = nullptr) const
  {
    return buffer_.canTransform(target_frame, source_frame, time, timeout, errstr);
  }

  bool canTransform(
    const std::string & target_frame, const tf2::TimePoint & target_time,
    const std::string & source_frame, const tf2::TimePoint & source_time,
    const std::string & fixed_frame, const tf2::Duration timeout,
    std::string * errstr = nullptr) const
  {
    return buffer_.canTransform(
      target_frame, target_time, source_frame, source_time, fixed_frame, timeout, errstr);
  }

  bool canTransform(
    const std::string & target_frame, const rclcpp::Time & target_time,
    const std::string & source_frame, const rclcpp::Time & source_time,
    const std::string & fixed_frame,
    const rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(0),
    std::string * errstr = nullptr) const
  {
    return buffer_.canTransform(
      target_frame, target_time, source_frame, source_time, fixed_frame, timeout, errstr);
  }

  // Internal API: used by TransformListener to bind the underlying tf2_ros listener to this
  // buffer's tf2::BufferCore. Not intended for downstream use.
  tf2::BufferCore & buffer_core() { return buffer_; }

private:
  tf2_ros::Buffer buffer_;
};

class TransformListener
{
public:
  TransformListener(
    Buffer & buffer, Node & node, bool spin_thread = true,
    const rclcpp::QoS & qos = tf2_ros::DynamicListenerQoS(),
    const rclcpp::QoS & static_qos = tf2_ros::StaticListenerQoS())
  : impl_(buffer.buffer_core(), node.get_rclcpp_node().get(), spin_thread, qos, static_qos)
  {
  }

  TransformListener(
    Buffer & buffer, Node & node, bool spin_thread, const rclcpp::QoS & qos,
    const rclcpp::QoS & static_qos, const AUTOWARE_SUBSCRIPTION_OPTIONS & options,
    const AUTOWARE_SUBSCRIPTION_OPTIONS & static_options)
  : impl_(
      buffer.buffer_core(), node.get_rclcpp_node().get(), spin_thread, qos, static_qos, options,
      static_options)
  {
  }

  explicit TransformListener(Buffer & buffer, bool spin_thread = true)
  : impl_(buffer.buffer_core(), spin_thread)
  {
  }

  TransformListener(const TransformListener &) = delete;
  TransformListener & operator=(const TransformListener &) = delete;
  TransformListener(TransformListener &&) = delete;
  TransformListener & operator=(TransformListener &&) = delete;

private:
  tf2_ros::TransformListener impl_;
};

class TransformBroadcaster
{
public:
  explicit TransformBroadcaster(
    Node & node, const rclcpp::QoS & qos = tf2_ros::DynamicBroadcasterQoS())
  : impl_(node.get_rclcpp_node().get(), qos)
  {
  }

  TransformBroadcaster(
    Node & node, const rclcpp::QoS & qos, const AUTOWARE_PUBLISHER_OPTIONS & options)
  : impl_(node.get_rclcpp_node().get(), qos, options)
  {
  }

  TransformBroadcaster(const TransformBroadcaster &) = delete;
  TransformBroadcaster & operator=(const TransformBroadcaster &) = delete;
  TransformBroadcaster(TransformBroadcaster &&) = delete;
  TransformBroadcaster & operator=(TransformBroadcaster &&) = delete;

  void sendTransform(const geometry_msgs::msg::TransformStamped & transform)
  {
    impl_.sendTransform(transform);
  }

  void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
  {
    impl_.sendTransform(transforms);
  }

private:
  tf2_ros::TransformBroadcaster impl_;
};

class StaticTransformBroadcaster
{
public:
  explicit StaticTransformBroadcaster(
    Node & node, const rclcpp::QoS & qos = tf2_ros::StaticBroadcasterQoS())
  : impl_(node.get_rclcpp_node().get(), qos)
  {
  }

  StaticTransformBroadcaster(
    Node & node, const rclcpp::QoS & qos, const AUTOWARE_PUBLISHER_OPTIONS & options)
  : impl_(node.get_rclcpp_node().get(), qos, options)
  {
  }

  StaticTransformBroadcaster(const StaticTransformBroadcaster &) = delete;
  StaticTransformBroadcaster & operator=(const StaticTransformBroadcaster &) = delete;
  StaticTransformBroadcaster(StaticTransformBroadcaster &&) = delete;
  StaticTransformBroadcaster & operator=(StaticTransformBroadcaster &&) = delete;

  void sendTransform(const geometry_msgs::msg::TransformStamped & transform)
  {
    impl_.sendTransform(transform);
  }

  void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
  {
    impl_.sendTransform(transforms);
  }

private:
  tf2_ros::StaticTransformBroadcaster impl_;
};

}  // namespace autoware::agnocast_wrapper

#endif
