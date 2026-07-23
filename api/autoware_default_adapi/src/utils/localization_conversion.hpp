// Copyright 2024 TIER IV, Inc.
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

#ifndef UTILS__LOCALIZATION_CONVERSION_HPP_
#define UTILS__LOCALIZATION_CONVERSION_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/srv/initialize_localization.hpp>
#include <autoware_localization_msgs/srv/initialize_localization.hpp>

#include <utility>

namespace autoware::default_adapi::localization_conversion
{

using ExternalInitializeRequest =
  autoware_adapi_v1_msgs::srv::InitializeLocalization::Request::SharedPtr;
using InternalInitializeRequest =
  autoware_localization_msgs::srv::InitializeLocalization::Request::SharedPtr;
InternalInitializeRequest convert_request(const ExternalInitializeRequest & external);

using ExternalResponse = autoware_adapi_v1_msgs::msg::ResponseStatus;
using InternalResponse = autoware_common_msgs::msg::ResponseStatus;
ExternalResponse convert_response(const InternalResponse & internal);

template <class ClientT, class RequestT>
ExternalResponse convert_call(ClientT & client, RequestT & req)
{
  // client is an agnocast_wrapper client (AUTOWARE_CLIENT_PTR): allocate the request from the
  // client, copy in the converted fields, then send. Works for both ENABLE_AGNOCAST=0 and =1.
  auto request = client->allocate_output_service_request();
  *request = *convert_request(req);
  auto future = client->async_send_request(std::move(request)).future;
  return convert_response(future.get()->status);
}

}  // namespace autoware::default_adapi::localization_conversion

#endif  // UTILS__LOCALIZATION_CONVERSION_HPP_
