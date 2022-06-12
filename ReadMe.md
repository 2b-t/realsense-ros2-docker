[![build](https://github.com/2b-t/realsense-ros2-docker/workflows/build/badge.svg)](https://github.com/2b-t/realsense-ros2-docker/actions/workflows/build.yml)

# Docker for Intel Realsense cameras and ROS2

Author: [Tobit Flatscher](https://github.com/2b-t) (June 2022)

## 0. Overview
This repository contains a Docker and all the documentation required to launch an Intel Realsense camera with ROS2. Currently **ROS2 Humble is not supported yet**, as it is not supported by the official ROS2 wrapper yet.

## 1. Dockerfiles
There are two different Dockerfiles given, one uses existing Debian packages while the other performs a full compilation from source.

### 1.1 Installation Debian packages
The relevant packages generally can be installed from Debian package. This is significantly simpler and less error prone but corresponding Debian packages might not be available for all Ubuntu versions. The corresponding Dockerfile can be found below. It is based on the installation guides for [`librealsense`](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages) and [its ROS wrapper](https://github.com/IntelRealSense/realsense-ros/tree/ros2-beta).

```Dockerfile
FROM ros:galactic-ros-base

ENV WS_DIR="/ros2_ws"
WORKDIR ${WS_DIR}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y \
 && apt-get install -y \
    build-essential \
    cmake \
    git-all \
    software-properties-common

# Install dependencies: See https://github.com/IntelRealSense/realsense-ros/tree/ros2-beta
# Librealsense: See https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages
RUN apt-get update -y \
 && apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
 && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
 && apt-get install -y \
    librealsense2-dbg \
    librealsense2-dev \
    librealsense2-dkms \
    librealsense2-utils

RUN apt-get update -y \
 && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
 && mkdir src \
 && cd src \
 && git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-beta \
 && cd .. \
 && apt-get install -y python3-rosdep \
 && source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rm /etc/ros/rosdep/sources.list.d/20-default.list \
 && rosdep init \
 && rosdep update \
 && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} --skip-keys=librealsense2 -y \
 && colcon build

ENV DEBIAN_FRONTEND=dialog
```

### 1.2 Installation from source
The installation from source is slightly more involved but more general. In order to optimise memory usage a [multi-stage build](https://docs.docker.com/develop/develop-images/multistage-build/) is performed. The Dockerfile is inspired by the [Dockerfile of `librealsense`](https://github.com/IntelRealSense/librealsense/blob/master/scripts/Docker/Dockerfile) contributed by community members:

```Dockerfile
ARG BASE_IMAGE=ros:galactic-ros-base # ros:humble-perception is not supported yet

# The following steps are based on the offical multi-stage build: https://github.com/IntelRealSense/librealsense/blob/master/scripts/Docker/Dockerfile
#################################
#   Librealsense Builder Stage  #
#################################
FROM $BASE_IMAGE as librealsense-builder

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -qq -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \    
    curl \
    python3 \
    python3-dev \
    ca-certificates \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /usr/src
# Get the latest tag of remote repository: https://stackoverflow.com/a/12704727
# Needs to be a single command as ENV can't be set from Bash command: https://stackoverflow.com/questions/34911622/dockerfile-set-env-to-result-of-command
RUN export LIBRS_GIT_TAG=`git -c 'versionsort.suffix=-' \
                         ls-remote --exit-code --refs --sort='version:refname' --tags https://github.com/IntelRealSense/librealsense '*.*.*' \
                         | tail --lines=1 \
                         | cut --delimiter='/' --fields=3`; \
    export LIBRS_VERSION=${LIBRS_VERSION:-${LIBRS_GIT_TAG#"v"}}; \
    curl https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v${LIBRS_VERSION} -o librealsense.tar.gz; \
    tar -zxf librealsense.tar.gz; \
    rm librealsense.tar.gz; \
    ln -s /usr/src/librealsense-${LIBRS_VERSION} /usr/src/librealsense

RUN cd /usr/src/librealsense \
 && mkdir build && cd build \
 && cmake \
    -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
    -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
    -DCMAKE_INSTALL_PREFIX=/opt/librealsense \    
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_PYTHON_BINDINGS:bool=true \
    -DCMAKE_BUILD_TYPE=Release ../ \
 && make -j$(($(nproc)-1)) all \
 && make install
 
 ENV DEBIAN_FRONTEND=dialog

######################################
#   librealsense Base Image Stage    #
######################################
FROM ${BASE_IMAGE} as librealsense

SHELL ["/bin/bash", "-c"]

COPY --from=librealsense-builder /opt/librealsense /usr/local/
COPY --from=librealsense-builder /usr/lib/python3/dist-packages/pyrealsense2 /usr/lib/python3/dist-packages/pyrealsense2
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
ENV PYTHONPATH=${PYTHONPATH}:/usr/local/lib

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y --no-install-recommends \	
    libusb-1.0-0 \
    udev \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common \
 && rm -rf /var/lib/apt/lists/*

# The following steps are based on: https://github.com/IntelRealSense/realsense-ros/tree/ros2-beta
ENV WS_DIR="/ros2_ws"
WORKDIR ${WS_DIR}
RUN apt-get update -y \
 && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
 && mkdir src \
 && cd src \
 && git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-beta \
 && cd .. \
 && apt-get install -y python3-rosdep \
 && source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rm /etc/ros/rosdep/sources.list.d/20-default.list \
 && rosdep init \
 && rosdep update \
 && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} --skip-keys=librealsense2 -y \
 && colcon build

ENV DEBIAN_FRONTEND=dialog

CMD [ "rs-enumerate-devices", "--compact" ]
```

## 2. Launching
The Intel Realsense driver has several serious flaws/bugs. Probably the main one is that it is closely connected to the [kernel version of the Linux operating system](https://github.com/IntelRealSense/librealsense/issues/9360). If the Dockerfile above do not work then you are likely unlucky and it is an incompatible version of the kernel of your host system and you will either have to [downgrade your kernel](https://linuxhint.com/install-linux-kernel-ubuntu/) or switch to another Ubuntu version that is officially supported.
Furthermore from time to time the software will give you cryptic error messages, for some restarting the corresponding software component miht help, for others you will find a fix googling and with others you will have to learn to live.
The Realsense are pretty [picky about USB 3.0 cables](https://github.com/IntelRealSense/librealsense/issues/2045). If your camera is detected via `rs-enumerate-devices`, you can see it `realsense-viewer` but can't output its video stream, then it might be that your cable lacks the bandwidth. Either you can try to turn down the resolution of the camera in the `realsense-viewer` or switch cable (preferably to one that is [already known to work](https://community.intel.com/t5/Items-with-no-label/long-USB-cable-for-realsense-D435i/m-p/694963)).

Build the Docker container with
```shell
$ docker compose -f docker-compose-gui.yml build
```
or directly with the `devcontainer` in Visual Studio Code. For Nvidia graphic cards the file `docker-compose-gui-nvidia.yml` in combination with the `nvidia-runtime` has to be used.
After it is done building **connect the Realsense**, start the container
```shell
$ docker compose -f docker-compose-gui.yml up
```
and see if you can **detect it from inside the Docker** by typing
```shell
$ rs-enumerate-devices --compact
```
Then continue to launch the proprietary visualisation applet
```shell
$ realsense-viewer
```
Turn on the camera inside the application, see if you can see a three-dimensional image. Finally we can **launch the ROS2 wrapper**
```shell
$ ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```
and in another terminal the ROS visualiser Rviz
```shell
$ rosrun rviz2 rviz2
```
In Rviz you can then display the topics published by the Realsense. For the list of available topics use
```shell
$ ros2 topic list
```
in combination with
```shell
$ ros2 topic info <topic_name>
```
to find out what display type has to be selected in Rviz.
