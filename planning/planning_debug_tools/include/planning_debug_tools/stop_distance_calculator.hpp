// Copyright 2021 Tier IV, Inc.
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

#ifndef PLANNING_DEBUG_TOOLS__STOP_DISTANCE_CALCULATOR_HPP_
#define PLANNING_DEBUG_TOOLS__STOP_DISTANCE_CALCULATOR_HPP_

#include "motion_utils/trajectory/trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tier4_debug_msgs/msg/float64_stamped.hpp"

#include <memory>
#include <string>
#include <vector>

namespace planning_debug_tools
{
using autoware_auto_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;
using tier4_debug_msgs::msg::Float64Stamped;

class StopDistanceCalculatorNode : public rclcpp::Node
{
public:
  explicit StopDistanceCalculatorNode(const rclcpp::NodeOptions & options);
  ~StopDistanceCalculatorNode() = default;

private:
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr pub_stop_distance_;

  Trajectory::ConstSharedPtr trajectory_;
  void onEgoKinematics(const Odometry::ConstSharedPtr msg);
};

}  // namespace planning_debug_tools

#endif  // PLANNING_DEBUG_TOOLS__STOP_DISTANCE_CALCULATOR_HPP_
