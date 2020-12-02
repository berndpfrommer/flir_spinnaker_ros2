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

#include <fstream>
#include <functional>
#include <image_transport/image_transport.hpp>
#include <iomanip>
#include <iostream>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "logging.h"

namespace flir_spinnaker_ros2
{
/* These are the types declared in rclcpp::msg::ParameterType
  uint8 PARAMETER_BOOL=1
  uint8 PARAMETER_INTEGER=2
  uint8 PARAMETER_DOUBLE=3
  uint8 PARAMETER_STRING=4
  uint8 PARAMETER_BYTE_ARRAY=5
  uint8 PARAMETER_BOOL_ARRAY=6
  uint8 PARAMETER_INTEGER_ARRAY=7
  uint8 PARAMETER_DOUBLE_ARRAY=8
  uint8 PARAMETER_STRING_ARRAY=9
*/
static rcl_interfaces::msg::ParameterDescriptor make_desc(
  const std::string name, int type)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.name = name;
  desc.type = type;
  desc.description = name;
  return (desc);
}

FlirSpinnakerROS2::NodeInfo::NodeInfo(
  const std::string & n, const std::string & nodeType)
: name(n)
{
  if (nodeType == "float") {
    type = FLOAT;
    descriptor = make_desc(n, 3);
  } else if (nodeType == "int") {
    type = INT;
    descriptor = make_desc(n, 2);
  } else if (nodeType == "bool") {
    type = BOOL;
    descriptor = make_desc(n, 1);
  } else if (nodeType == "enum") {
    type = ENUM;
    descriptor = make_desc(n, 4);
  }
}

FlirSpinnakerROS2::FlirSpinnakerROS2(const std::shared_ptr<rclcpp::Node> & node)
: node_(node)
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
      node_->get_logger(),
      serial_ << " incoming frame rate: " << driver_->getFrameRate());
  } else {
    RCLCPP_WARN_STREAM(node_->get_logger(), serial_ << " is not online!");
  }
}

void FlirSpinnakerROS2::readParameters()
{
  name_ = node_->declare_parameter<std::string>("name", "missing_camera_name");
  serial_ = node_->declare_parameter<std::string>(
    "serial_number", "missing_serial_number");
  cameraInfoURL_ = node_->declare_parameter<std::string>("camerainfo_url", "");
  frameId_ = node_->declare_parameter<std::string>("frame_id", name_);
  dumpNodeMap_ = node_->declare_parameter<bool>("dump_node_map", false);
  parameterFile_ =
    node_->declare_parameter<std::string>("parameter_file", "parameters.cfg");
  LOG_INFO("camera name: " << name_ << " serial: " << serial_);
  callbackHandle_ = node_->add_on_set_parameters_callback(std::bind(
    &FlirSpinnakerROS2::parameterChanged, this, std::placeholders::_1));
}

bool FlirSpinnakerROS2::readParameterFile()
{
  std::ifstream f(parameterFile_);
  if (!f.is_open()) {
    LOG_ERROR("cannot read parameter definition file: " << parameterFile_);
    return (false);
  }
  std::string l;
  while (getline(f, l)) {
    std::istringstream iss(l);
    std::string s;
    std::vector<std::string> tokens;
    while (iss >> std::quoted(s)) {
      tokens.push_back(s);
    }
    if (!tokens.empty()) {
      if (tokens[0] == "#") {
        continue;
      }
      if (tokens.size() != 3) {
        LOG_WARN("skipping bad camera param line: " << l);
        continue;
      }
      parameterMap_.insert({tokens[0], NodeInfo(tokens[2], tokens[1])});
    }
  }
  f.close();
#if 0
  parameterMap_.insert(
    {"gain_auto", NodeInfo("AnalogControl/GainAuto", NodeInfo::ENUM)});

  parameterMap_.insert(
    {"exposure_auto",
     NodeInfo(
       "AcquisitionControl/ExposureAuto", NodeInfo::ENUM, "Continuous")});
  parameterMap_.insert(
    {"frame_rate_auto", NodeInfo(
                          "AcquisitionControl/AcquisitionFrameRateAuto",
                          NodeInfo::ENUM, "Continuous")});
  parameterMap_.insert(
    {"frame_rate",
     NodeInfo("AcquisitionControl/AcquisitionFrameRate", NodeInfo::FLOAT, "")});
  parameterMap_.insert(
    {"exposure_time",
     NodeInfo("AcquisitionControl/ExposureTime", NodeInfo::FLOAT, "10000.0")});
#endif
  return (true);
}

void FlirSpinnakerROS2::createCameraParameters()
{
  for (const auto & m : parameterMap_) {
    const auto ni = m.second;
    try {
#if 1
      LOG_INFO(
        "creating param " << m.first << " desc: " << (int)ni.descriptor.type);
#endif
      node_->declare_parameter(
        m.first, rclcpp::ParameterValue(), ni.descriptor, false);
    } catch (rclcpp::exceptions::InvalidParameterTypeException & e) {
      LOG_WARN("overwriting bad param with default: " + std::string(e.what()));
      node_->declare_parameter(
        m.first, rclcpp::ParameterValue(), ni.descriptor, true);
    }
  }
}

static bool check_double(const rclcpp::Parameter & p, const char * name)
{
  return (
    p.get_name() == name && (p.get_type() == rclcpp::PARAMETER_DOUBLE ||
                             p.get_type() == rclcpp::PARAMETER_INTEGER));
}

static bool check_bool(const rclcpp::Parameter & p, const char * name)
{
  return (
    p.get_name() == name && (p.get_type() == rclcpp::PARAMETER_INTEGER ||
                             p.get_type() == rclcpp::PARAMETER_BOOL));
}

static double get_double_int_param(const rclcpp::Parameter & p)
{
  return (
    p.get_type() == rclcpp::PARAMETER_DOUBLE ? p.as_double()
                                             : double(p.as_int()));
}

static bool get_bool_int_param(const rclcpp::Parameter & p)
{
  return (
    p.get_type() == rclcpp::PARAMETER_BOOL ? p.as_bool() : (bool)(p.as_int()));
}

bool FlirSpinnakerROS2::setEnum(
  const std::string & nodeName, const std::string & v)
{
  RCLCPP_INFO_STREAM(
    node_->get_logger(), "setting " << nodeName << " to: " << v);
  std::string retV;  // what actually was set
  std::string msg = driver_->setEnum(nodeName, v, &retV);
  bool status(true);
  if (msg != "OK") {
    RCLCPP_WARN_STREAM(
      node_->get_logger(), "setting " << nodeName << " failed: " << msg);
    status = false;
  }
  if (v != retV) {
    RCLCPP_WARN_STREAM(
      node_->get_logger(),
      nodeName << " set to: " << retV << " instead of: " << v);
    status = false;
  }
  return (status);
}

bool FlirSpinnakerROS2::setDouble(const std::string & nodeName, double v)
{
  LOG_INFO("setting " << nodeName << " to: " << v);
  double retV;  // what actually was set
  std::string msg = driver_->setDouble(nodeName, v, &retV);
  bool status(true);
  if (msg != "OK") {
    LOG_WARN("setting " << nodeName << " failed: " << msg);
    status = false;
  }
  if (std::abs(v - retV) * retV > 0.05) {
    LOG_WARN(nodeName << " set to: " << retV << " instead of: " << v);
    status = false;
  }
  return (status);
}

bool FlirSpinnakerROS2::setBool(const std::string & nodeName, bool v)
{
  LOG_INFO("setting " << nodeName << " to: " << v);
  bool retV;  // what actually was set
  std::string msg = driver_->setBool(nodeName, v, &retV);
  bool status(true);
  if (msg != "OK") {
    LOG_WARN("setting " << nodeName << " failed: " << msg);
    status = false;
  }
  if (v != retV) {
    LOG_WARN(nodeName << " set to: " << retV << " instead of: " << v);
    status = false;
  }
  return (status);
}

rcl_interfaces::msg::SetParametersResult FlirSpinnakerROS2::parameterChanged(
  const std::vector<rclcpp::Parameter> & params)
{
  for (const auto & p : params) {
    const auto it = parameterMap_.find(p.get_name());
    if (it == parameterMap_.end()) {
      continue;  // ignore unknown param
    }
    if (!driver_) {
      LOG_WARN("got parameter update while driver is not ready!");
      continue;
    }
    const NodeInfo & ni = it->second;
    LOG_INFO("param changed, type " << ni.name << ": " << (int)ni.type);
    if (p.get_type() == rclcpp::PARAMETER_NOT_SET) {
      //LOG_INFO("not setting unset param: " << ni.name);
      continue;
    }

    switch (ni.type) {
      case NodeInfo::ENUM: {
        std::string s = p.value_to_string();
        // remove quotes
        s.erase(remove(s.begin(), s.end(), '\"'), s.end());
        setEnum(ni.name, s);
        break;
      }
      case NodeInfo::FLOAT:
        if (
          p.get_type() == rclcpp::PARAMETER_DOUBLE ||
          p.get_type() == rclcpp::PARAMETER_INTEGER) {
          setDouble(
            ni.name, p.get_type() == rclcpp::PARAMETER_DOUBLE
                       ? p.as_double()
                       : double(p.as_int()));
        } else {
          LOG_WARN(
            "bad non-float parameter: " << p.get_name()
                                        << " type: " << p.get_type());
        }
        break;
      case NodeInfo::BOOL:
        if (
          p.get_type() == rclcpp::PARAMETER_BOOL ||
          p.get_type() == rclcpp::PARAMETER_INTEGER) {
          setBool(
            ni.name, p.get_type() == rclcpp::PARAMETER_BOOL ? p.as_bool()
                                                            : bool(p.as_int()));
        } else {
          LOG_WARN(
            "bad non-bool parameter: " << p.get_name()
                                       << " type: " << p.get_type());
        }
        break;
      default:
        LOG_WARN("invalid node type in map: " << ni.type);
    }
  }
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  res.reason = "all good!";
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

  const std::string encoding =
    sensor_msgs::image_encodings::BAYER_RGGB8;  // looks good
  //const std::string encoding = sensor_msgs::image_encodings::BAYER_GBRG8; // checkered
  //const std::string encoding = sensor_msgs::image_encodings::BAYER_BGGR8; // smooth but col reversed??
  //const std::string encoding = sensor_msgs::image_encodings::BAYER_GRBG8;  // checkered

  // will make deep copy. Do we need to?
  bool ret = sensor_msgs::fillImage(
    *imageMsg_, encoding, im->height_, im->width_, im->stride_, im->data_);
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
      node_->get_logger(),
      "camera has pixel format: " << driver_->getPixelFormat());
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
  if (!readParameterFile()) {
    return (false);
  }
  infoManager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    node_.get(), name_, cameraInfoURL_);
  cameraInfoMsg_ = std::make_shared<sensor_msgs::msg::CameraInfo>(
    infoManager_->getCameraInfo());
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
  qos_profile.durability =
    RMW_QOS_POLICY_DURABILITY_VOLATILE;  // sender does not have to store

  qos_profile.deadline.sec =
    5;  // max expected time between messages beeing published
  qos_profile.deadline.nsec = 0;

  qos_profile.lifespan.sec =
    1;  // how long until messages are considered expired
  qos_profile.lifespan.nsec = 0;

  qos_profile.liveliness_lease_duration.sec =
    10;  // how long until client is considered dead
  qos_profile.liveliness_lease_duration.nsec = 0;

  pub_ = image_transport::create_camera_publisher(
    node_.get(), name_ + "/image_raw", qos_profile);
  driver_ = std::make_shared<flir_spinnaker_common::Driver>();
  RCLCPP_INFO_STREAM(
    node_->get_logger(),
    "using spinnaker library version: " + driver_->getLibraryVersion());
  const auto camList = driver_->getSerialNumbers();
  if (camList.empty()) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "no cameras found!");
  }
  if (std::find(camList.begin(), camList.end(), serial_) == camList.end()) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(), "no camera found with serial number:" << serial_);
    for (const auto cam : camList) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "found cameras: " << cam);
    }
    return (false);
  }
  keepRunning_ = true;
  thread_ = std::make_shared<std::thread>(&FlirSpinnakerROS2::run, this);

  if (driver_->initCamera(serial_)) {
    if (dumpNodeMap_) {
      LOG_INFO("getting nodemap!");
      std::string nm = driver_->getNodeMapAsString();
      std::cout << nm;
    }
    startCamera();  // TODO: once ROS2 supports subscriber status callbacks, this can go!
    createCameraParameters();
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(), "init camera failed for cam: " << serial_);
  }
  return (true);
}
}  // namespace flir_spinnaker_ros2
