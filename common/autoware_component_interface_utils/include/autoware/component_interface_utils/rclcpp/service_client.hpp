// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_CLIENT_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_CLIENT_HPP_

#include <autoware/component_interface_utils/rclcpp/exceptions.hpp>
#include <autoware/component_interface_utils/rclcpp/interface.hpp>
#include <rclcpp/node.hpp>

#include <chrono>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

namespace autoware::component_interface_utils
{

/// True when the node's create_client() takes rclcpp::QoS. ROS 2 Iron (rclcpp 21) onward does;
/// Humble (rclcpp 16) exposes only the rmw_qos_profile_t overload. Detected rather than gated on
/// the distro so that a node type offering only the QoS overload also works.
template <class SpecT, class NodeT, class = void>
struct has_qos_create_client : std::false_type
{
};
template <class SpecT, class NodeT>
struct has_qos_create_client<
  SpecT, NodeT,
  std::void_t<decltype(std::declval<NodeT &>().template create_client<typename SpecT::Service>(
    std::declval<const std::string &>(), std::declval<const rclcpp::QoS &>(),
    std::declval<rclcpp::CallbackGroup::SharedPtr>()))>> : std::true_type
{
};

/// Create the underlying client handle.
template <class SpecT, class NodeT>
auto create_client_handle(NodeT * node, rclcpp::CallbackGroup::SharedPtr group)
{
  if constexpr (has_qos_create_client<SpecT, NodeT>::value) {
    return node->template create_client<typename SpecT::Service>(
      SpecT::name, rclcpp::ServicesQoS(), group);
  } else {
    return node->template create_client<typename SpecT::Service>(
      SpecT::name, rmw_qos_profile_services_default, group);
  }
}

/// True when the handle's async_send_request() accepts a shared_ptr request without taking
/// ownership, as rclcpp::Client does. A handle that requires an owned (rvalue) request instead
/// needs the request allocated from it and copied in.
template <class HandleT, class RequestPtrT, class = void>
struct accepts_shared_request : std::false_type
{
};
template <class HandleT, class RequestPtrT>
struct accepts_shared_request<
  HandleT, RequestPtrT,
  std::void_t<decltype(std::declval<HandleT &>().async_send_request(
    std::declval<RequestPtrT &>()))>> : std::true_type
{
};

#if AUTOWARE_COMPONENT_INTERFACE_UTILS_RCLCPP_GE_IRON
/// True when the handle supports ROS 2 service introspection (rclcpp's Client / Service do; a
/// handle that does not expose configure_introspection() cannot be traced this way).
template <class HandleT, class = void>
struct has_configure_introspection : std::false_type
{
};
template <class HandleT>
struct has_configure_introspection<
  HandleT, std::void_t<decltype(std::declval<HandleT &>().configure_introspection(
             std::declval<rclcpp::Clock::SharedPtr>(), std::declval<const rclcpp::QoS &>(),
             std::declval<rcl_service_introspection_state_t>()))>> : std::true_type
{
};
#endif

/// The wrapper class of a service client. Service-call tracing is provided by ROS 2
/// service introspection (enabled via NodeInterface::introspection_state), not a
/// custom log topic.
///
/// The handle type is deduced from the node's create_client(), so it is rclcpp::Client<Service>
/// for the default NodeT. The request/response/future aliases are spelled from the spec rather
/// than taken off the handle, so they do not depend on the node type.
template <class SpecT, class NodeT = rclcpp::Node>
class Client
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Client)
  using SpecType = SpecT;
  using NodeType = NodeT;
  using WrapSharedPtr = decltype(create_client_handle<SpecT>(
    std::declval<NodeT *>(), std::declval<rclcpp::CallbackGroup::SharedPtr>()));
  using WrapType = typename WrapSharedPtr::element_type;

  using SharedRequest = std::shared_ptr<typename SpecT::Service::Request>;
  using SharedResponse = std::shared_ptr<typename SpecT::Service::Response>;
  using SharedFuture = std::shared_future<SharedResponse>;

  /// Constructor.
  Client(typename NodeInterface<NodeT>::SharedPtr interface, rclcpp::CallbackGroup::SharedPtr group)
  : interface_(interface)
  {
    client_ = create_client_handle<SpecT>(interface_->node, group);
#if AUTOWARE_COMPONENT_INTERFACE_UTILS_RCLCPP_GE_IRON
    if constexpr (has_configure_introspection<WrapType>::value) {
      if (interface_->introspection_state != RCL_SERVICE_INTROSPECTION_OFF) {
        client_->configure_introspection(
          interface_->node->get_clock(), rclcpp::QoS(1), interface_->introspection_state);
      }
    }
#endif
  }

  /// Send request.
  SharedResponse call(const SharedRequest request, std::optional<double> timeout = std::nullopt)
  {
    if (!client_->service_is_ready()) {
      throw ServiceUnready(SpecT::name);
    }

    auto future = this->async_send_request(request);
    if (timeout) {
      const auto duration = std::chrono::duration<double, std::ratio<1>>(timeout.value());
      if (future.wait_for(duration) != std::future_status::ready) {
        throw ServiceTimeout(SpecT::name);
      }
    }
    auto response = future.get();
    if constexpr (std::is_same_v<decltype(response), SharedResponse>) {
      return response;
    } else {
      // The handle owns the response (and may hand out a const one), so copy it out to keep
      // call()'s return type the same for every node type.
      return std::make_shared<typename SpecT::Service::Response>(*response);
    }
  }

  /// Send request.
  auto async_send_request(SharedRequest request)
  {
    if constexpr (accepts_shared_request<WrapType, SharedRequest>::value) {
      return client_->async_send_request(request).future;
    } else {
      auto req = client_->allocate_output_service_request();
      *req = *request;
      return client_->async_send_request(std::move(req)).future;
    }
  }

  /// Send request with a response callback. The callback is wrapped in a
  /// concrete-signature lambda so callers may pass a generic (auto-parameter)
  /// callback, which rclcpp::Client::async_send_request would otherwise reject.
  template <class CallbackT>
  auto async_send_request(SharedRequest request, CallbackT && callback)
  {
    const auto wrapped = [callback](typename WrapType::SharedFuture future) { callback(future); };
    if constexpr (accepts_shared_request<WrapType, SharedRequest>::value) {
      return client_->async_send_request(request, wrapped).future;
    } else {
      auto req = client_->allocate_output_service_request();
      *req = *request;
      return client_->async_send_request(std::move(req), wrapped).future;
    }
  }

  /// Check if the service is ready.
  bool service_is_ready() const { return client_->service_is_ready(); }

private:
  RCLCPP_DISABLE_COPY(Client)
  WrapSharedPtr client_;
  typename NodeInterface<NodeT>::SharedPtr interface_;
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_CLIENT_HPP_
