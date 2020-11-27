/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <flir_spinnaker_ros2/flir_spinnaker_ros2.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("flir_spinnaker_ros2");
  const auto fs2 =
      std::make_shared<flir_spinnaker_ros2::FlirSpinnakerROS2>(node);

  fs2->start();

  RCLCPP_INFO(node->get_logger(), "flir_spinnaker_ros2_node started up!");
  // actually run the node
  rclcpp::spin(node);  // should not return
  rclcpp::shutdown();
  return 0;
}
