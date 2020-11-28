/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <flir_spinnaker_common/driver.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace flir_spinnaker_ros2 {
class FlirSpinnakerROS2 {
 public:
  FlirSpinnakerROS2(const std::shared_ptr<rclcpp::Node>& node);
  void start();

 private:
  void readParameters();
  // ----- variables --
  std::shared_ptr<rclcpp::Node> node_;
  image_transport::CameraPublisher pub_;
  std::shared_ptr<flir_spinnaker_common::Driver> driver_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager_;
};
}  // namespace flir_spinnaker_ros2
