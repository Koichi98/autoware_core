// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_HPP_
#define AUTOWARE__PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_HPP_

// since ASSERT_NO_THROW in gtest masks the exception message, redefine it.
#define ASSERT_NO_THROW_WITH_ERROR_MSG(statement)                                                \
  try {                                                                                          \
    statement;                                                                                   \
    SUCCEED();                                                                                   \
  } catch (const std::exception & e) {                                                           \
    FAIL() << "Expected: " << #statement                                                         \
           << " doesn't throw an exception.\nActual: it throws. Error message: " << e.what()     \
           << std::endl;                                                                         \
  } catch (...) {                                                                                \
    FAIL() << "Expected: " << #statement                                                         \
           << " doesn't throw an exception.\nActual: it throws. Error message is not available." \
           << std::endl;                                                                         \
  }

#include <autoware/component_interface_specs/planning.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <gtest/gtest.h>

#include <ctime>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_test_manager
{
class PlanningInterfaceTestManager
{
public:
  PlanningInterfaceTestManager();

  template <typename InputT, typename NodeT = rclcpp::Node>
  void publishInput(
    const std::shared_ptr<NodeT> target_node, const std::string & topic_name, const InputT & input,
    const int repeat_count = 3) const
  {
    autoware::test_utils::publishToTargetNode(
      test_node_, target_node, topic_name, {}, input, repeat_count);
  }

  template <typename OutputT, typename CallbackT>
  void subscribeOutput(const std::string & topic_name, CallbackT && callback)
  {
    const auto qos = []() {
      if constexpr (std::is_same_v<OutputT, autoware_planning_msgs::msg::Trajectory>) {
        return rclcpp::QoS{1};
      }
      return rclcpp::QoS{10};
    }();

    test_output_subs_.push_back(
      test_node_->create_subscription<OutputT>(topic_name, qos, std::forward<CallbackT>(callback)));
  }

  template <typename OutputT>
  void subscribeOutput(const std::string & topic_name)
  {
    return subscribeOutput<OutputT>(
      topic_name, [this](const typename OutputT::ConstSharedPtr) { received_topic_num_++; });
  }

  template <typename NodeT = rclcpp::Node>
  void testWithNormalTrajectory(std::shared_ptr<NodeT> target_node, const std::string & topic_name)
  {
    publishInput(
      target_node, topic_name,
      autoware::test_utils::generateTrajectory<autoware_planning_msgs::msg::Trajectory>(10, 1.0),
      5);
  }

  template <typename NodeT = rclcpp::Node>
  void testWithAbnormalTrajectory(
    std::shared_ptr<NodeT> target_node, const std::string & topic_name)
  {
    publishInput(target_node, topic_name, autoware_planning_msgs::msg::Trajectory{}, 5);
    publishInput(
      target_node, topic_name,
      autoware::test_utils::generateTrajectory<autoware_planning_msgs::msg::Trajectory>(1, 0.0), 5);
    publishInput(
      target_node, topic_name,
      autoware::test_utils::generateTrajectory<autoware_planning_msgs::msg::Trajectory>(
        10, 0.0, 0.0, 0.0, 0.0, 1),
      5);
  }

  template <typename NodeT = rclcpp::Node>
  void testWithNormalRoute(std::shared_ptr<NodeT> target_node, const std::string & topic_name)
  {
    publishInput(target_node, topic_name, autoware::test_utils::makeNormalRoute(), 5);
  }

  template <typename NodeT = rclcpp::Node>
  void testWithAbnormalRoute(std::shared_ptr<NodeT> target_node, const std::string & topic_name)
  {
    publishInput(target_node, topic_name, autoware_planning_msgs::msg::LaneletRoute{}, 5);
  }

  template <typename NodeT = rclcpp::Node>
  void testWithBehaviorNormalRoute(
    std::shared_ptr<NodeT> target_node, const std::string & topic_name)
  {
    publishInput(target_node, topic_name, autoware::test_utils::makeBehaviorNormalRoute(), 5);
  }

  template <typename NodeT = rclcpp::Node>
  void testWithBehaviorGoalOnLeftSide(
    std::shared_ptr<NodeT> target_node, const std::string & topic_name)
  {
    publishInput(
      target_node, topic_name, autoware::test_utils::makeBehaviorGoalOnLeftSideRoute(), 5);
  }

  template <typename NodeT = rclcpp::Node>
  void testWithNormalPathWithLaneId(
    std::shared_ptr<NodeT> target_node, const std::string & topic_name)
  {
    try {
      const auto path = autoware::test_utils::loadPathWithLaneIdInYaml();
      publishInput(target_node, topic_name, path, 5);
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
    }
  }

  template <typename NodeT = rclcpp::Node>
  void testWithAbnormalPathWithLaneId(
    std::shared_ptr<NodeT> target_node, const std::string & topic_name)
  {
    publishInput(
      target_node, topic_name, autoware_internal_planning_msgs::msg::PathWithLaneId{}, 5);
  }

  template <typename NodeT = rclcpp::Node>
  void testWithNormalPath(std::shared_ptr<NodeT> target_node, const std::string & topic_name)
  {
    try {
      const auto path = autoware::test_utils::loadPathWithLaneIdInYaml();
      publishInput(
        target_node, topic_name,
        autoware::motion_utils::convertToPath<autoware_internal_planning_msgs::msg::PathWithLaneId>(
          path),
        5);
    } catch (const std::exception & e) {
      std::cerr << e.what() << '\n';
    }
  }

  template <typename NodeT = rclcpp::Node>
  void testWithAbnormalPath(std::shared_ptr<NodeT> target_node, const std::string & topic_name)
  {
    publishInput(target_node, topic_name, autoware_planning_msgs::msg::Path{}, 5);
  }

  template <typename NodeT = rclcpp::Node>
  void testWithOffTrackInitialPoses(
    std::shared_ptr<NodeT> target_node, const std::string & topic_name)
  {
    for (const auto & deviation : {0.0, 1.0, 10.0, 100.0}) {
      publishInput(target_node, topic_name, autoware::test_utils::makeInitialPose(deviation), 5);
    }
  }

  template <typename NodeT = rclcpp::Node>
  void testWithOffTrackOdometry(std::shared_ptr<NodeT> target_node, const std::string & topic_name)
  {
    for (const auto & deviation : {0.0, 1.0, 10.0, 100.0}) {
      publishInput(target_node, topic_name, autoware::test_utils::makeOdometry(deviation), 5);
    }
  }

  void resetReceivedTopicNum() { received_topic_num_ = 0; }

  size_t getReceivedTopicNum() const { return received_topic_num_; }

  rclcpp::Node::SharedPtr getTestNode() const { return test_node_; }

private:
  // Subscriber
  std::vector<rclcpp::SubscriptionBase::SharedPtr> test_output_subs_;

  // Node
  rclcpp::Node::SharedPtr test_node_;

  size_t received_topic_num_ = 0;
};  // class PlanningInterfaceTestManager

}  // namespace autoware::planning_test_manager

#endif  // AUTOWARE__PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_HPP_
