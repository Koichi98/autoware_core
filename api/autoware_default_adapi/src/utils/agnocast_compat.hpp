// Copyright 2026 The Autoware Contributors
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

#ifndef UTILS__AGNOCAST_COMPAT_HPP_
#define UTILS__AGNOCAST_COMPAT_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>

namespace autoware::default_adapi::agnocast_compat
{

/// Create a service from an rclcpp-style shared_ptr callback, adapting it to the wrapper's
/// message_ptr service callback. Under ENABLE_AGNOCAST=1 the wrapper requires the message_ptr
/// callback shape; this wraps the shared_ptr callback (copying request/response) so node code can
/// keep its rclcpp-style handlers. Works for ENABLE_AGNOCAST=0 as well.
template <class ServiceT, class NodeT, class Fn>
AUTOWARE_SERVICE_PTR(ServiceT)
create_service(
  NodeT * node, const std::string & name, Fn && fn, const rclcpp::QoS & qos = rclcpp::ServicesQoS(),
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return node->template create_service<ServiceT>(
    name,
    [fn = std::forward<Fn>(fn)](
      AUTOWARE_SERVER_REQUEST_PTR(ServiceT) && req_msg,
      AUTOWARE_SERVER_RESPONSE_PTR(ServiceT) && res_msg) {
      auto request = std::make_shared<typename ServiceT::Request>(*req_msg);
      auto response = std::make_shared<typename ServiceT::Response>();
      fn(request, response);
      *res_msg = *response;
    },
    qos, group);
}

}  // namespace autoware::default_adapi::agnocast_compat

#endif  // UTILS__AGNOCAST_COMPAT_HPP_
