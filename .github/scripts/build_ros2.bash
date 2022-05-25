#!/bin/bash
# set up ROS
distros=('foxy' 'galactic' 'humble')

#
# probe for the ROS2 distro
#
for distro in "${distros[@]}"
do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
	source /opt/ros/${distro}/setup.bash
	break
    fi
done

echo "found ros version: ${ROS_VERSION} distro: ${ROS_DISTRO}"
# run wstool to bring in the additional repositories required
wstool init src ./src/flir_spinnaker_ros2/flir_spinnaker_ros2.rosinstall

# apt list --installed | grep -i 'ament-clang-format'
ls -la /opt/ros/${ROS_DISTRO}/share/ament_cmake_clang_format/cmake/

# build and test
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && colcon test && colcon test-result --verbose


