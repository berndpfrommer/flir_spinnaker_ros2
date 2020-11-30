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

#ifndef FLIR_SPINNAKER_ROS2__FLIR_SPINNAKER_ROS2_H_
#define FLIR_SPINNAKER_ROS2__FLIR_SPINNAKER_ROS2_H_

#include <flir_spinnaker_common/driver.h>
#include <flir_spinnaker_common/image.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace flir_spinnaker_ros2
{
class FlirSpinnakerROS2
{
public:
  typedef flir_spinnaker_common::ImageConstPtr ImageConstPtr;
  explicit FlirSpinnakerROS2(const std::shared_ptr<rclcpp::Node> & node);
  ~FlirSpinnakerROS2();

  bool start();
  bool stop();

private:
  void publishImage(const ImageConstPtr & image);
  void readParameters();
  void printCameraInfo();
  // ----- variables --
  std::shared_ptr<rclcpp::Node> node_;
  image_transport::CameraPublisher pub_;
  std::string name_;
  std::string serial_;
  std::string cameraInfoURL_;
  std::string frameId_;
  std::shared_ptr<flir_spinnaker_common::Driver> driver_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager_;
  sensor_msgs::msg::Image::SharedPtr imageMsg_;
  sensor_msgs::msg::CameraInfo::SharedPtr cameraInfoMsg_;
  bool cameraRunning_{false};
};
}  // namespace flir_spinnaker_ros2
#endif  // FLIR_SPINNAKER_ROS2__FLIR_SPINNAKER_ROS2_H_
