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
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/fill_image.hpp>

namespace flir_spinnaker_ros2
{
FlirSpinnakerROS2::FlirSpinnakerROS2(const std::shared_ptr<rclcpp::Node> & node)
: node_(node) {}


FlirSpinnakerROS2::~FlirSpinnakerROS2()
{
  stop();
}

bool FlirSpinnakerROS2::stop()
{
  if (cameraRunning_ && driver_) {
    return driver_->stopCamera();
  }
  return false;
}

void FlirSpinnakerROS2::readParameters()
{
  name_ = node_->declare_parameter<std::string>("name", "missing_camera_name");
  serial_ = node_->declare_parameter<std::string>("serial_number", "missing_serial_number");
  cameraInfoURL_ = node_->declare_parameter<std::string>("camerainfo_url", "");
  frameId_ = node_->declare_parameter<std::string>("frame_id", name_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "camera name: " << name_ << " serial: " << serial_);
}

void FlirSpinnakerROS2::publishImage(const ImageConstPtr & im)
{
  imageMsg_->header.stamp = rclcpp::Time(im->time_);
  cameraInfoMsg_->header.stamp = rclcpp::Time(im->time_);

  const std::string encoding = sensor_msgs::image_encodings::BAYER_RGGB8;  //
  //const std::string encoding = sensor_msgs::image_encodings::BAYER_GBRG8; // checkered
  //const std::string encoding = sensor_msgs::image_encodings::BAYER_BGGR8; // smooth but col reversed??
  //const std::string encoding = sensor_msgs::image_encodings::BAYER_GRBG8;  // checkered

  // will make deep copy. Do we need to?
  bool ret =
    sensor_msgs::fillImage(*imageMsg_, encoding, im->height_, im->width_, im->stride_, im->data_);
  if (!ret) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "camera name: " << name_ << " serial: " << serial_);
  }
  pub_.publish(imageMsg_, cameraInfoMsg_);
}

void FlirSpinnakerROS2::printCameraInfo()
{
  if (!driver_) {
    RCLCPP_INFO_STREAM(
      node_->get_logger(), "camera has pixel format: " << driver_->getPixelFormat());
  }
}
bool FlirSpinnakerROS2::start()
{
  readParameters();
  infoManager_ =
    std::make_shared<camera_info_manager::CameraInfoManager>(node_.get(), name_, cameraInfoURL_);
  cameraInfoMsg_ = std::make_shared<sensor_msgs::msg::CameraInfo>(infoManager_->getCameraInfo());
  imageMsg_ = std::make_shared<sensor_msgs::msg::Image>();
  imageMsg_->header.frame_id = frameId_;
  cameraInfoMsg_->header.frame_id = frameId_;

  const rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
  pub_ = image_transport::create_camera_publisher(node_.get(), name_ + "/image", custom_qos);
  driver_ = std::make_shared<flir_spinnaker_common::Driver>();
  RCLCPP_INFO_STREAM(
    node_->get_logger(), "using spinnaker library version: " + driver_->getLibraryVersion());
  const auto camList = driver_->getSerialNumbers();
  if (camList.empty()) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "no cameras found!");
  }
  for (const auto cam : camList) {
    if (cam == serial_) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "found matching camera serial number: " << cam);
      flir_spinnaker_common::Driver::Callback cb =
        std::bind(&FlirSpinnakerROS2::publishImage, this, std::placeholders::_1);
      cameraRunning_ = driver_->startCamera(cam, cb);
      if (!cameraRunning_) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "failed to start cam " << cam);
      } else {
        printCameraInfo();
      }
    } else {
      RCLCPP_INFO_STREAM(node_->get_logger(), "skipping camera with serial number: " << cam);
    }
  }
  if (!cameraRunning_) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "no camera found with serial number:" << serial_);
    for (const auto cam : camList) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "found cam: " << cam);
    }
  }
  return cameraRunning_;
}
}  // namespace flir_spinnaker_ros2
