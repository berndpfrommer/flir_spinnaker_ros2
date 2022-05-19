#!/bin/bash
# set up ROS
flavors=('foxy', 'galactic', 'humble')

#
# probe for the ROS2 flavor
#
for flavor in "${flavors[@]}"
do
    if [[ -f "/opt/ros/${flavor}/setup.bash" ]]; then
	source /opt/ros/${flavor}/setup.bash
    fi
done

# run wstool to bring in the additional repositories required
wstool init src ./src/flir_spinnaker_ros2/flir_spinnaker_ros2.rosinstall

# build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
