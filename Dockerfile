ARG ROS_VERSION=dashing
ARG UBUNTU_VERSION=bionic
# we start at ubuntu-18.04 (bionic) with ROS 2 Dashing preinstalled as our base image
FROM arm64v8/ros:dashing-ros-base-bionic
ENV PATH="/root/.local/bin:${PATH}"
# Copy qemu from the host machine. This will be the interpreter for anything and everything that runs inside this container. Since the host is x86, an ARM64->x86 interpreter is required.
COPY qemu-aarch64-static /usr/bin
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
# Installed for convenience
RUN apt-get update && apt-get install -y vim
# Add raspicam_node sources to apt
# RUN apt-get update && apt-get install -y apt-transport-https \
#  && echo "deb https://packages.ubiquityrobotics.com/ubuntu/ubiquity xenial main" > /etc/apt/sources.list.d/ubiquity-latest.list \
#  && apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C3032ED8
# Install Raspberry Pi package sources and libraries
# RUN apt-get update && apt-get install -y gnupg lsb-release software-properties-common \
#  && add-apt-repository -y ppa:ubuntu-pi-flavour-makers/ppa \
#  && apt-get update && apt-get install -y libraspberrypi0
# Install Python and colcon
RUN apt-get update && apt-get install -y \
   python \
   python3-apt \
   curl \
  && curl -O https://bootstrap.pypa.io/get-pip.py \
  && python3 get-pip.py \
  && python2 get-pip.py \
  && python3 -m pip install -U colcon-ros-bundle
# Add custom rosdep rules
# COPY custom-rosdep-rules/raspicam-node.yaml /etc/ros/rosdep/custom-rules/raspicam-node.yaml
# RUN echo "yaml file:/etc/ros/rosdep/custom-rules/raspicam-node.yaml" > /etc/ros/rosdep/sources.list.d/22-raspicam-node.list \
RUN echo "yaml https://s3-us-west-2.amazonaws.com/rosdep/python.yaml" > /etc/ros/rosdep/sources.list.d/18-aws-python.list \
   && rosdep update
# Add custom pip rules
COPY custom-pip-rules.conf /etc/pip.conf
# Add custom apt sources for bundling
COPY bionic-sources-arm64.yaml /opt/cross/apt-sources.yaml