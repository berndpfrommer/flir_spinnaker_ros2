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

#include <functional>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace flir_spinnaker_ros2
{
FlirSpinnakerROS2::FlirSpinnakerROS2(const std::shared_ptr<rclcpp::Node> & node) : node_(node)
{
  statusTimer_ = rclcpp::create_timer(
    node, node->get_clock(), rclcpp::Duration(5, 0),
    std::bind(&FlirSpinnakerROS2::printStatus, this));
}

FlirSpinnakerROS2::~FlirSpinnakerROS2() { stop(); }

bool FlirSpinnakerROS2::stop()
{
  stopCamera();
  if (driver_) {
    driver_->deInitCamera();
  }
  if (!statusTimer_->is_canceled()) {
    statusTimer_->cancel();
  }
  keepRunning_ = false;
  if (thread_) {
    thread_->join();
  }
  return (true);
}

bool FlirSpinnakerROS2::stopCamera()
{
  if (cameraRunning_ && driver_) {
    return driver_->stopCamera();
  }
  return false;
}

void FlirSpinnakerROS2::printStatus()
{
  if (driver_) {
    RCLCPP_INFO_STREAM(
      node_->get_logger(), serial_ << " incoming frame rate: " << driver_->getFrameRate());
  } else {
    RCLCPP_WARN_STREAM(node_->get_logger(), serial_ << " is not online!");
  }
}

void FlirSpinnakerROS2::readParameters()
{
  name_ = node_->declare_parameter<std::string>("name", "missing_camera_name");
  serial_ = node_->declare_parameter<std::string>("serial_number", "missing_serial_number");
  cameraInfoURL_ = node_->declare_parameter<std::string>("camerainfo_url", "");
  frameId_ = node_->declare_parameter<std::string>("frame_id", name_);
  dumpNodeMap_ = node_->declare_parameter<bool>("dump_node_map", false);
  RCLCPP_INFO_STREAM(node_->get_logger(), "camera name: " << name_ << " serial: " << serial_);
  callbackHandle_ = node_->add_on_set_parameters_callback(
    std::bind(&FlirSpinnakerROS2::parameterChanged, this, std::placeholders::_1));
}

void FlirSpinnakerROS2::createCameraParameters()
{
  rcl_interfaces::msg::ParameterDescriptor frameRateDesc;
  frameRateDesc.name = "frame_frate";
  frameRateDesc.type = 3;  // double
  frameRateDesc.description = "frame rate of camera";
  frameRate_ = node_->declare_parameter<double>("frame_rate", 10.0, frameRateDesc);
}

rcl_interfaces::msg::SetParametersResult FlirSpinnakerROS2::parameterChanged(
  const std::vector<rclcpp::Parameter> & params)
{
  for (const auto & p : params) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "parameter changed: " << p.get_name());
    if (
      p.get_name() == "frame_rate" && driver_ &&
      (p.get_type() == rclcpp::PARAMETER_DOUBLE || p.get_type() == rclcpp::PARAMETER_INTEGER)) {
      double v = p.get_type() == rclcpp::PARAMETER_DOUBLE ? p.as_double() : double(p.as_int());
      RCLCPP_INFO_STREAM(node_->get_logger(), "setting frame rate to: " << v);
      std::string msg = driver_->setFrameRate(v, &frameRate_);
      if (msg != "OK") {
        RCLCPP_WARN_STREAM(node_->get_logger(), "frame rate set failed: " << msg);
      }
      if (std::abs(v - frameRate_) > 0.2) {
        RCLCPP_WARN_STREAM(
          node_->get_logger(), "frame rate does not match desired: " << frameRate_);
      }
    }
  }
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  res.reason = "not implemented yet!";
  return (res);
}

void FlirSpinnakerROS2::publishImage(const ImageConstPtr & im)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (imageQueue_.size() < 2) {
    imageQueue_.push_back(im);
    cv_.notify_all();
  }
}

void FlirSpinnakerROS2::run()
{
  while (keepRunning_) {
    {
      ImageConstPtr img;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        const std::chrono::microseconds timeout((int64_t)(10000000LL));
        while (imageQueue_.empty()) {
          cv_.wait_for(lock, timeout);
        }
        if (!imageQueue_.empty()) {
          img = imageQueue_.back();
          imageQueue_.pop_back();
        }
      }
      if (img) {
        doPublish(img);
      }
    }
  }
}

void FlirSpinnakerROS2::doPublish(const ImageConstPtr & im)
{
  imageMsg_->header.stamp = rclcpp::Time(im->time_);
  cameraInfoMsg_->header.stamp = rclcpp::Time(im->time_);

  const std::string encoding = sensor_msgs::image_encodings::BAYER_RGGB8;  // looks good
  //const std::string encoding = sensor_msgs::image_encodings::BAYER_GBRG8; // checkered
  //const std::string encoding = sensor_msgs::image_encodings::BAYER_BGGR8; // smooth but col reversed??
  //const std::string encoding = sensor_msgs::image_encodings::BAYER_GRBG8;  // checkered

  // will make deep copy. Do we need to?
  bool ret =
    sensor_msgs::fillImage(*imageMsg_, encoding, im->height_, im->width_, im->stride_, im->data_);
  if (!ret) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "fill image failed!");
  }
  //const auto t0 = node_->now();
  pub_.publish(imageMsg_, cameraInfoMsg_);
  //const auto t1 = node_->now();
  //std::cout << "dt: " << (t1 - t0).nanoseconds() * 1e-9 << std::endl;
}

void FlirSpinnakerROS2::printCameraInfo()
{
  if (cameraRunning_) {
    RCLCPP_INFO_STREAM(
      node_->get_logger(), "camera has pixel format: " << driver_->getPixelFormat());
  }
}

void FlirSpinnakerROS2::startCamera()
{
  if (!cameraRunning_) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "starting camera!");
    flir_spinnaker_common::Driver::Callback cb =
      std::bind(&FlirSpinnakerROS2::publishImage, this, std::placeholders::_1);
    cameraRunning_ = driver_->startCamera(cb);
    if (!cameraRunning_) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "failed to start camera!");
    } else {
      printCameraInfo();
    }
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

  //const rmw_qos_profile_t qos = rmw_qos_profile_default;
  rmw_qos_profile_t qos_profile;
  qos_profile = rmw_qos_profile_default;
  qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  //qos_profile.depth = 1;  // keep at most one image
  qos_profile.depth = 1;  // keep at most one image

  qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
  //qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

  //qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  //qos_profile.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
  qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;  // sender does not have to store

  qos_profile.deadline.sec = 5;  // max expected time between messages beeing published
  qos_profile.deadline.nsec = 0;

  qos_profile.lifespan.sec = 1;  // how long until messages are considered expired
  qos_profile.lifespan.nsec = 0;

  qos_profile.liveliness_lease_duration.sec = 10;  // how long until client is considered dead
  qos_profile.liveliness_lease_duration.nsec = 0;

  pub_ = image_transport::create_camera_publisher(node_.get(), name_ + "/image_raw", qos_profile);
  driver_ = std::make_shared<flir_spinnaker_common::Driver>();
  RCLCPP_INFO_STREAM(
    node_->get_logger(), "using spinnaker library version: " + driver_->getLibraryVersion());
  const auto camList = driver_->getSerialNumbers();
  if (camList.empty()) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "no cameras found!");
  }
  if (std::find(camList.begin(), camList.end(), serial_) == camList.end()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "no camera found with serial number:" << serial_);
    for (const auto cam : camList) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "found cameras: " << cam);
    }
    return (false);
  }
  keepRunning_ = true;
  thread_ = std::make_shared<std::thread>(&FlirSpinnakerROS2::run, this);

  if (driver_->initCamera(serial_)) {
    if (dumpNodeMap_) {
      std::string nm = driver_->getNodeMapAsString();
      std::cout << nm;
    }
    startCamera();  // TODO: once ROS2 supports subscriber status callbacks, this can go!
    createCameraParameters();
  } else {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "init camera failed for cam: " << serial_);
  }
  return (true);
}
}  // namespace flir_spinnaker_ros2
