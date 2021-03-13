# FLIR/Spinnaker ROS2 driver

Simple ROS2 driver for the FLIR cameras using the [Spinnaker
SDK](http://softwareservices.flir.com/Spinnaker/latest/index.htmlspinnaker).

NOTE: This driver is not written or supported by FLIR.

# Tested cameras:

The following cameras have been tested:

- Blackfly S (USB3)
- Grashopper (USB3)
- Chameleon (USB3) tested on firmware v1.13.3.00

# Features

Basic features are supported like setting exposure, gain, and external
triggering. It's straight forward to support new camera types and features by
editing the camera definition (.cfg) files. Unless you need new pixel
formats you may not have to modify any source code. The code is meant
to be a thin wrapper for setting the features available in FLIR's
SpinView program.

# Example usage

How to launch the example file:

    ros2 launch flir_spinnaker_ros2 blackfly_s.launch.py camera_name:=blackfly_0 serial:="'20435008'"


