/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <flir_spinnaker_ros2/flir_spinnaker_ros2.h>

#include <image_transport/image_transport.hpp>

namespace flir_spinnaker_ros2 {
FlirSpinnakerROS2::FlirSpinnakerROS2(const std::shared_ptr<rclcpp::Node>& node)
    : node_(node) {}

void FlirSpinnakerROS2::start() {
  const rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
  pub_ = image_transport::create_camera_publisher(node_.get(), "camera/image",
                                                  custom_qos);
  infoManager_ =
      std::make_shared<camera_info_manager::CameraInfoManager>(node_.get());
  driver_ = std::make_shared<flir_spinnaker_common::Driver>();
  RCLCPP_INFO_STREAM(node_->get_logger(), "using spinnaker library version: " +
                                              driver_->getLibraryVersion());
}
}  // namespace flir_spinnaker_ros2
