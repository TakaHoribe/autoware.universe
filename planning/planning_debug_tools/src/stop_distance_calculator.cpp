// Copyright 2022 Tier IV, Inc.
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

#include <planning_debug_tools/stop_distance_calculator.hpp>
#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <memory>
#include <string>

namespace planning_debug_tools
{

StopDistanceCalculatorNode::StopDistanceCalculatorNode(const rclcpp::NodeOptions & options)
: Node("stop_distance_calculator", options)
{
  using std::placeholders::_1;

  pub_stop_distance_ = create_publisher<Float64Stamped>("~/stop_distance", 1);
  sub_odometry_ = create_subscription<Odometry>(
    "/localization/kinematic_state", 1, std::bind(&StopDistanceCalculatorNode::onEgoKinematics, this, _1));
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", 1,
    [this](const Trajectory::ConstSharedPtr msg) { trajectory_ = msg; });
}

void StopDistanceCalculatorNode::onEgoKinematics(const Odometry::ConstSharedPtr msg)
{
  if (!trajectory_) return;

  const auto stop_dist = motion_utils::calcDistanceToForwardStopPoint(trajectory_->points, msg->pose.pose);

  if (!stop_dist) return;

  Float64Stamped result;
  result.stamp = now();
  result.data = stop_dist.get();
  pub_stop_distance_->publish(result);
}

}  // namespace planning_debug_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planning_debug_tools::StopDistanceCalculatorNode)
