# How to update the docker images

## Create the base image (ROS1 and ROS2 combined)
Change into the ``.github/docker`` directory and build the base image:
```
os_flavor=focal
ros1_flavor=noetic
ros2_flavor=galactic
combined=${os_flavor}_${ros1_flavor}_${ros2_flavor}
docker build -t your_dockerhub_name/${combined} - < Dockerfile.${combined}
```

Upload the base image to dockerhub:
```
docker login
docker push your_dockerhub_name/${combined}
```

## Build docker image with Spinnaker SDK and other missing packages on it

Download the Spinnaker SDK and copy it into the .github/docker
directory. Uncompress/untar it there. Hack the install_spinnaker.sh
file to work interactively. This can be done by substituting ``read
confirm`` by ``confirm=no`` except for the first two prompts where you
need ``confirm=yes``. Also change the dpkg install commands like so:
```
yes | sudo dpkg -i libspinnaker_*.deb
```

Build and push image with remaining packages:
```
docker build -t your_dockerhub_name/${combined}_build - < Dockerfile.${combined}_build
docker push your_dockerhub_name/${combined}_build
```


