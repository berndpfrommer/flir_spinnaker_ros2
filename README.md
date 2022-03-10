# FLIR/Spinnaker ROS2 driver

Simple ROS2 driver for the FLIR cameras using the [Spinnaker
SDK](http://softwareservices.flir.com/Spinnaker/latest/index.htmlspinnaker).

NOTE: This driver is not written or supported by FLIR.

# Tested cameras:

The following cameras have been tested:

- Blackfly S (USB3)
- Grashopper (USB3)
- Chameleon (USB3) tested on firmware v1.13.3.00

## Supported platforms

Software:

- ROS2 Galactic (Foxy should work as well)
- Ubuntu 20.04 LTS

# Features

Basic features are supported like setting exposure, gain, and external
triggering. It's straight forward to support new camera types and features by
editing the camera definition (.cfg) files. Unless you need new pixel
formats you may not have to modify any source code. The code is meant
to be a thin wrapper for setting the features available in FLIR's
SpinView program.

# How to build

1) Install the FLIR spinnaker driver.
2) Prepare the ROS2 driver build:
Make sure you have your ROS2 environment sourced:
```
source /opt/ros/galactic/setup.bash
```

Create a workspace (``flir_spinnaker_ros2_ws``), clone this repo, and use ``wstool``
to pull in the remaining dependencies:

```
mkdir -p ~/flir_spinnaker_ros2_ws/src
cd ~/flir_spinnaker_ros2_ws
git clone https://github.com/berndpfrommer/flir_spinnaker_ros2 src/flir_spinnaker_ros2
wstool init src src/flir_spinnaker_ros2/flir_spinnaker_ros2.rosinstall

# or to update an existing space
# wstool merge -t src src/flir_spinnaker_ros2/flir_spinnaker_ros2.rosinstall
# wstool update -t src
```
3) Build the driver and source the workspace:
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
. install/setup.bash
```

# Example usage

How to launch the example file:

    ros2 launch flir_spinnaker_ros2 blackfly_s.launch.py camera_name:=blackfly_0 serial:="'20435008'"


