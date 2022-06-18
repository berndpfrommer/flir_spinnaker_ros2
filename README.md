# FLIR/Spinnaker ROS2 driver

Simple ROS2 driver for the FLIR cameras using the [Spinnaker
SDK](http://softwareservices.flir.com/Spinnaker/latest/index.htmlspinnaker).

NOTE: This driver is not written or supported by FLIR.

## Tested cameras:

The following cameras have been tested:

- Blackfly S (USB3, GigE)
- Grashopper (USB3)
- Chameleon (USB3) tested on firmware v1.13.3.00

Note: if you get other cameras to work, *please report back*, ideally
submit a pull request with the camera config file you have created.

## Supported platforms

Software:

- Ubuntu 20.04 LTS
- ROS2 Galactic and Foxy
- Spinnaker 2.6.0.157 (other versions may work as well but this is
  what the continuous integration builds are using)

The code compiles under Ubuntu 22.04 / Humble but has not been tested
yet with real hardware.

## Features

Basic features are supported like setting exposure, gain, and external
triggering. It's straight forward to support new camera types and features by
editing the camera definition (.cfg) files. Unless you need new pixel
formats you may not have to modify any source code. The code is meant
to be a thin wrapper for setting the features available in FLIR's
SpinView program.

## How to build

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

To automatically install all packages that the ``flir_spinnaker_ros2``
depends upon, run this at the top of your workspace:
```
rosdep install --from-paths src --ignore-src
```

3) Build the driver and source the workspace:
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
. install/setup.bash
```

## Example usage

How to launch the example file:
```
ros2 launch flir_spinnaker_ros2 blackfly_s.launch.py camera_name:=blackfly_0 serial:="'20435008'"
```

## Setting up GigE cameras

The Spinnaker SDK abstracts away the transport layer so a GigE camera
should work the same way as USB3: you point it to the serial
number and you're set.

There are a few GigE-specific settings in the Transport Layer Control
group that are important, in particular enabling jumbo frames from the
camera per FLIR's recommendations. The following line in your
camera-specific config file will create a ROS2 parameter
``gev_scps_packet_size``:
```
gev_scps_packet_size int "TransportLayerControl/GigEVision/GevSCPSPacketSize"
```
that you can then set in your ROS2 launch file:
```
 "gev_scps_packet_size": 9000
```
As far as setting up the camera's IP address: you can set up DHCP on
your network or configure a static persistent IP using SpinView 
in "Transport Layer Control">"GigE Vision". Check the box for "Current
IP Configuration Persistent IP" first to enable it, then set your
desired addresses under "Persistent IP Address", "Persistent Subnet
Mask" and "Persistent Gateway". NOTE: these look like regular IPs, but
to set them you have to enter the 32-bit integer representation of the
IP address/mask. By hand/calculator: convert the IP octets from
decimal to hex, then combine them and convert to a 32-bit integer, ex:
192.168.0.1 -> 0xC0A80001 -> 3232235521.

The "Transport Layer Control">"GigE Vision" section of SpinView is
also where you'll find that "SCPS Packet Size" setting, which you can
change when not capturing frames, and verify it works in SpinView and
without needing to spin up a custom launch file to get started, though
it helps, and you'll probably want one anyway to specify your camera's
serial number.

For more tips on GigE setup look at FLIR's support pages
[here](https://www.flir.com/support-center/iis/machine-vision/knowledge-base/lost-ethernet-data-packets-on-linux-systems/)
and
[here](https://www.flir.com/support-center/iis/machine-vision/application-note/troubleshooting-image-consistency-errors/).

## How to contribute
Please provide feedback if you cannot get your camera working or if
the code does not compile for you. Feedback is crucial for the
software development process.

Bug fixes and config files for new cameras are greatly
appreciated. Before submitting a pull request, run this to see if your
commit passes some basic lint tests:
```
colcon test --packages-select flir_spinnaker_ros2 && colcon test-result --verbose
```


## License

This software is issued under the Apache License Version 2.0.
