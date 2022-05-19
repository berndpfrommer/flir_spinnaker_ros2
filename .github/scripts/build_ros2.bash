#!/bin/bash
# set up ROS
flavors=('foxy', 'galactic')

for flavor in "${flavors[@]}"
do
    if [[ -f "/opt/ros/${flavor}/setup.bash" ]]; then
	source /opt/ros/${flavor}/setup.bash
    fi
done

# make workspace
mkdir -p ../ws/src
cd ../ws

# link source into catkin workspace
ln -s $src_dir src/apriltag

# build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
