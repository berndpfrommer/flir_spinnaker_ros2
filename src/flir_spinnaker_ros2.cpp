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
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "logging.h"

namespace flir_spinnaker_ros2
{
static rcl_interfaces::msg::ParameterDescriptor make_desc(
  const std::string name, int type)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.name = name;
  desc.type = type;
  desc.description = name;
  return (desc);
}

static std::pair<bool, double> get_double_int_param(const rclcpp::Parameter & p)
{
  std::pair<bool, double> bd(false, 0);
  if (p.get_type() == rclcpp::PARAMETER_DOUBLE) {
    bd.second = p.as_double();
    bd.first = true;
  }
  if (p.get_type() == rclcpp::PARAMETER_INTEGER) {
    bd.second = (double)p.as_int();
    bd.first = true;
  }
  return (bd);
}

static std::pair<bool, bool> get_bool_int_param(const rclcpp::Parameter & p)
{
  std::pair<bool, bool> bb(false, false);
  if (p.get_type() == rclcpp::PARAMETER_BOOL) {
    bb.second = p.as_bool();
    bb.first = true;
  }
  if (p.get_type() == rclcpp::PARAMETER_INTEGER) {
    bb.second = (bool)p.as_int();
    bb.first = true;
  }
  return (bb);
}

FlirSpinnakerROS2::NodeInfo::NodeInfo(
  const std::string & n, const std::string & nodeType)
: name(n)
{
  if (nodeType == "float") {
    type = FLOAT;
    descriptor = make_desc(n, rclcpp::ParameterType::PARAMETER_DOUBLE);
  } else if (nodeType == "int") {
    type = INT;
    descriptor = make_desc(n, rclcpp::ParameterType::PARAMETER_INTEGER);
  } else if (nodeType == "bool") {
    type = BOOL;
    descriptor = make_desc(n, rclcpp::ParameterType::PARAMETER_BOOL);
  } else if (nodeType == "enum") {
    type = ENUM;
    descriptor = make_desc(n, rclcpp::ParameterType::PARAMETER_STRING);
  }
}

FlirSpinnakerROS2::FlirSpinnakerROS2(const rclcpp::NodeOptions & options)
: Node("cam_sync", options)
{
  statusTimer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration(5, 0),
    std::bind(&FlirSpinnakerROS2::printStatus, this));
  bool status = start();
  if (!status) {
    LOG_ERROR("startup failed!");
    throw std::runtime_error("startup of FlirSpinnakerROS2 node failed!");
  }
}

FlirSpinnakerROS2::~FlirSpinnakerROS2()
{
  stop();
  driver_.reset();  // invoke destructor
}

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
    cameraRunning_ = false;
    return driver_->stopCamera();
  }
  return false;
}

void FlirSpinnakerROS2::printStatus()
{
  if (driver_) {
    LOG_INFO(
      "incoming frame rate: " << driver_->getReceiveFrameRate() << " Hz");
  } else {
    LOG_WARN("camera " << serial_ << " is not online!");
  }
}

void FlirSpinnakerROS2::readParameters()
{
  name_ = this->declare_parameter<std::string>("name", "missing_camera_name");
  serial_ = this->declare_parameter<std::string>(
    "serial_number", "missing_serial_number");
  try {
    debug_ = this->declare_parameter(
      "debug", false,
      make_desc("debug", rclcpp::ParameterType::PARAMETER_BOOL));
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    LOG_WARN("bad debug param type: " << e.what());
    debug_ = false;
  }
  LOG_INFO("debug: " << debug_);
  cameraInfoURL_ = this->declare_parameter<std::string>("camerainfo_url", "");
  frameId_ = this->declare_parameter<std::string>("frame_id", name_);
  dumpNodeMap_ = this->declare_parameter<bool>("dump_node_map", false);
  parameterFile_ =
    this->declare_parameter<std::string>("parameter_file", "parameters.cfg");
  LOG_INFO("camera name: " << name_ << " serial: " << serial_);
  callbackHandle_ = this->add_on_set_parameters_callback(std::bind(
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
      parameterList_.push_back(tokens[0]);
    }
  }
  f.close();
  return (true);
}

void FlirSpinnakerROS2::createCameraParameters()
{
  for (const auto name : parameterList_) {
    const auto it = parameterMap_.find(name);
    if (it != parameterMap_.end()) {
      const auto & ni = it->second;  // should always succeed
      try {
        this->declare_parameter(
          name, rclcpp::ParameterValue(), ni.descriptor, false);
      } catch (rclcpp::exceptions::InvalidParameterTypeException & e) {
        LOG_WARN(
          "overwriting bad param with default: " + std::string(e.what()));
        this->declare_parameter(
          name, rclcpp::ParameterValue(), ni.descriptor, true);
      }
    }
  }
}

bool FlirSpinnakerROS2::setEnum(
  const std::string & nodeName, const std::string & v)
{
  LOG_INFO("setting " << nodeName << " to: " << v);
  std::string retV;  // what actually was set
  std::string msg = driver_->setEnum(nodeName, v, &retV);
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

void FlirSpinnakerROS2::setParameter(
  const NodeInfo & ni, const rclcpp::Parameter & p)
{
  switch (ni.type) {
    case NodeInfo::ENUM: {
      std::string s = p.value_to_string();
      // remove quotes
      s.erase(remove(s.begin(), s.end(), '\"'), s.end());
      setEnum(ni.name, s);
      break;
    }
    case NodeInfo::FLOAT: {
      auto bd = get_double_int_param(p);
      if (bd.first) {
        setDouble(ni.name, bd.second);
      } else {
        LOG_WARN("bad non-float " << p.get_name() << " type: " << p.get_type());
      }
      break;
    }
    case NodeInfo::BOOL: {
      auto bb = get_bool_int_param(p);
      if (bb.first) {
        setBool(ni.name, bb.second);
      } else {
        LOG_WARN("bad non-bool " << p.get_name() << " type: " << p.get_type());
      }
      break;
    }
    default:
      LOG_WARN("invalid node type in map: " << ni.type);
  }
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
    if (p.get_type() == rclcpp::PARAMETER_NOT_SET) {
      //LOG_INFO("not setting unset param: " << ni.name);
      continue;
    }
    try {
      setParameter(ni, p);
    } catch (const flir_spinnaker_common::Driver::DriverException & e) {
      LOG_WARN("param " << p.get_name() << " " << e.what());
    }
  }
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  res.reason = "all good!";
  return (res);
}

void FlirSpinnakerROS2::publishImage(const ImageConstPtr & im)
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (imageQueue_.size() < 2) {
      imageQueue_.push_back(im);
      cv_.notify_all();
    }
  }
}

void FlirSpinnakerROS2::run()
{
  while (keepRunning_ && rclcpp::ok()) {
    {
      ImageConstPtr img;
      {  // ------- locked section ---
        std::unique_lock<std::mutex> lock(mutex_);
        // one second timeout
        const std::chrono::microseconds timeout((int64_t)(1000000LL));
        while (imageQueue_.empty() && keepRunning_ && rclcpp::ok()) {
          cv_.wait_for(lock, timeout);
        }
        if (!imageQueue_.empty()) {
          img = imageQueue_.back();
          imageQueue_.pop_back();
        }
      }  // -------- end of locked section
      if (img && keepRunning_ && rclcpp::ok()) {
        doPublish(img);
      }
    }
  }
}

void FlirSpinnakerROS2::doPublish(const ImageConstPtr & im)
{
  // todo: honor the encoding in the image
  imageMsg_->header.stamp = rclcpp::Time(im->time_);
  cameraInfoMsg_->header.stamp = rclcpp::Time(im->time_);

  const std::string encoding =
    sensor_msgs::image_encodings::BAYER_RGGB8;  // looks good

  // will make deep copy. Do we need to?
  bool ret = sensor_msgs::fillImage(
    *imageMsg_, encoding, im->height_, im->width_, im->stride_, im->data_);
  if (!ret) {
    LOG_ERROR("fill image failed!");
  } else {
    //const auto t0 = this->now();
    pub_.publish(imageMsg_, cameraInfoMsg_);
    //const auto t1 = this->now();
    //std::cout << "dt: " << (t1 - t0).nanoseconds() * 1e-9 << std::endl;
  }
}

void FlirSpinnakerROS2::printCameraInfo()
{
  if (cameraRunning_) {
    LOG_INFO("camera has pixel format: " << driver_->getPixelFormat());
  }
}

void FlirSpinnakerROS2::startCamera()
{
  if (!cameraRunning_) {
    LOG_INFO("starting camera!");
    flir_spinnaker_common::Driver::Callback cb =
      std::bind(&FlirSpinnakerROS2::publishImage, this, std::placeholders::_1);
    cameraRunning_ = driver_->startCamera(cb);
    if (!cameraRunning_) {
      LOG_ERROR("failed to start camera!");
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
    this, name_, cameraInfoURL_);
  cameraInfoMsg_ = std::make_shared<sensor_msgs::msg::CameraInfo>(
    infoManager_->getCameraInfo());
  imageMsg_ = std::make_shared<sensor_msgs::msg::Image>();
  imageMsg_->header.frame_id = frameId_;
  cameraInfoMsg_->header.frame_id = frameId_;

  rmw_qos_profile_t qosProf = rmw_qos_profile_default;
  qosProf.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qosProf.depth = 1;  // keep at most one image

  qosProf.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
  //qosProf.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

  //qosProf.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  //qosProf.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
  qosProf.durability =
    RMW_QOS_POLICY_DURABILITY_VOLATILE;  // sender does not have to store
  qosProf.deadline.sec = 5;              // max expect time between msgs pub
  qosProf.deadline.nsec = 0;

  qosProf.lifespan.sec = 1;  // how long until msg are considered expired
  qosProf.lifespan.nsec = 0;

  qosProf.liveliness_lease_duration.sec = 10;  // time to declare client dead
  qosProf.liveliness_lease_duration.nsec = 0;

  pub_ = image_transport::create_camera_publisher(
    this, name_ + "/image_raw", qosProf);
  driver_ = std::make_shared<flir_spinnaker_common::Driver>();
  driver_->setDebug(debug_);
  LOG_INFO("using spinnaker lib version: " + driver_->getLibraryVersion());
  const auto camList = driver_->getSerialNumbers();
  if (camList.empty()) {
    LOG_WARN("driver found no cameras!");
  }
  if (std::find(camList.begin(), camList.end(), serial_) == camList.end()) {
    LOG_ERROR("no camera found with serial number:" << serial_);
    for (const auto cam : camList) {
      LOG_WARN("found cameras: " << cam);
    }
    return (false);
  }
  keepRunning_ = true;
  thread_ = std::make_shared<std::thread>(&FlirSpinnakerROS2::run, this);

  if (driver_->initCamera(serial_)) {
    if (dumpNodeMap_) {
      LOG_INFO("dumping node map!");
      std::string nm = driver_->getNodeMapAsString();
      std::cout << nm;
    }
    startCamera();  // TODO: once ROS2 supports subscriber status callbacks, this can go!
    createCameraParameters();
  } else {
    LOG_ERROR("init camera failed for cam: " << serial_);
  }
  return (true);
}
}  // namespace flir_spinnaker_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(flir_spinnaker_ros2::FlirSpinnakerROS2)
