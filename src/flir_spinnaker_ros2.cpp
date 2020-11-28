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

#include <image_transport/image_transport.hpp>

namespace flir_spinnaker_ros2
{
FlirSpinnakerROS2::FlirSpinnakerROS2(const std::shared_ptr<rclcpp::Node> & node)
: node_(node) {}

void FlirSpinnakerROS2::readParameters()
{
  const std::vector<std::string> cameraGroups =
    node_->declare_parameter<std::vector<std::string>>("camera_groups", std::vector<std::string>());
  for (const auto & cg : cameraGroups) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "camera group: " << cg);
    const std::vector<std::string> cameras = node_->declare_parameter<std::vector<std::string>>(
      cg + "." + "cameras", std::vector<std::string>());
    for (const auto & cam : cameras) {
      RCLCPP_INFO_STREAM(node_->get_logger(), " camera " << cam << ":");
    }
  }
}

void FlirSpinnakerROS2::start()
{
  readParameters();
  const rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
  pub_ = image_transport::create_camera_publisher(node_.get(), "camera/image", custom_qos);
  infoManager_ = std::make_shared<camera_info_manager::CameraInfoManager>(node_.get());
  driver_ = std::make_shared<flir_spinnaker_common::Driver>();
  RCLCPP_INFO_STREAM(
    node_->get_logger(), "using spinnaker library version: " + driver_->getLibraryVersion());
}
}  // namespace flir_spinnaker_ros2
