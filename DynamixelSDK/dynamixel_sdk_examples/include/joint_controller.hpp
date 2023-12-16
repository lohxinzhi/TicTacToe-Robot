// Copyright 2021 ROBOTIS CO., LTD.
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

#ifndef JOINT_CONTROLLER_HPP_
#define JOINT_CONTROLLER_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/joints_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_joints_position.hpp"


class JointController : public rclcpp::Node
{
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  using JointsPosition = dynamixel_sdk_custom_interfaces::msg::JointsPosition;
  using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;
  using GetJointsPosition = dynamixel_sdk_custom_interfaces::srv::GetJointsPosition;

  JointController();
  virtual ~JointController();

private:
  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Subscription<JointsPosition>::SharedPtr joints_position_subscriber_;  
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
  rclcpp::Service<GetJointsPosition>::SharedPtr get_joints_position_server_;


  int present_position;
  
};

#endif  // READ_WRITE_NODE_HPP_
