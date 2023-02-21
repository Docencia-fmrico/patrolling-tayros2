// Copyright 2021 Intelligent Robotics Lab
// Copyright 2023 Tyros2 Group
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

#include "tyros2_patrolling/GetWaypoint.hpp"

#include <iostream>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tyros2_patrolling
{

int GetWaypoint::current_ = 0;

GetWaypoint::GetWaypoint(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  geometry_msgs::msg::PoseStamped wp;
  wp.header.frame_id = "map";
  wp.pose.orientation.w = 1.0;

  // wp1
  wp.pose.position.x = 3.02;
  wp.pose.position.y = -4.78;
  waypoints_.push_back(wp);

  // wp2
  wp.pose.position.x = -1.56;
  wp.pose.position.y = -10.32;
  waypoints_.push_back(wp);

  // wp3
  wp.pose.position.x = -6.44;
  wp.pose.position.y = -12.26;
  waypoints_.push_back(wp);

  // wp4
  wp.pose.position.x = -2.94;
  wp.pose.position.y = 0.66;
  waypoints_.push_back(wp);
}

void GetWaypoint::halt()
{
}

BT::NodeStatus GetWaypoint::tick()
{
  setOutput("waypoint", waypoints_[current_++]);
  current_ = current_ % waypoints_.size();

  return BT::NodeStatus::SUCCESS;
}

}  // namespace tyros2_patrolling

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<tyros2_patrolling::GetWaypoint>("GetWaypoint");
}
