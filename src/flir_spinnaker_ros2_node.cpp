// -*-c++-*--------------------------------------------------------------------
// Copyright 2020 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include <flir_spinnaker_ros2/flir_spinnaker_ros2.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
    rclcpp::Node::make_shared("flir_spinnaker_ros2");
  const auto fs2 =
    std::make_shared<flir_spinnaker_ros2::FlirSpinnakerROS2>(node);

  if (fs2->start()) {
    RCLCPP_INFO(node->get_logger(), "flir_spinnaker_ros2_node started up!");
    // actually run the node
    rclcpp::spin(node);  // should not return
    rclcpp::shutdown();
  } else {
    RCLCPP_ERROR(node->get_logger(), "flir_spinnaker_ros2_node start failed!");
  }
  return 0;
}
