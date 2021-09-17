ARG ROS_VERSION=kinetic
ARG UBUNTU_VERSION=xenial
FROM arm32v7/ros:${ROS_VERSION}-ros-base-${UBUNTU_VERSION}

ENV PATH="/root/.local/bin:${PATH}"

# Copy qemu from the host machine
COPY qemu-arm-static /usr/bin

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Update OSRF repo keys
RUN apt-get update && apt-get install curl && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -


# Installed for convenience
RUN apt-get update && apt-get install -y vim

# Add raspicam_node sources to apt
RUN apt-get update && apt-get install -y apt-transport-https \
    && echo "deb https://packages.ubiquityrobotics.com/ubuntu/ubiquity xenial main" > /etc/apt/sources.list.d/ubiquity-latest.list \
    && apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C3032ED8

# Install Raspberry Pi package sources and libraries
RUN apt-get update && apt-get install -y gnupg lsb-release software-properties-common \
    && add-apt-repository -y ppa:ubuntu-pi-flavour-makers/ppa \
    && apt-get update && apt-get install -y libraspberrypi0

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
COPY custom-rosdep-rules/raspicam-node.yaml /etc/ros/rosdep/custom-rules/raspicam-node.yaml
RUN echo "yaml file:/etc/ros/rosdep/custom-rules/raspicam-node.yaml" > /etc/ros/rosdep/sources.list.d/22-raspicam-node.list \
    && echo "yaml https://s3-us-west-2.amazonaws.com/rosdep/python.yaml" > /etc/ros/rosdep/sources.list.d/18-aws-python.list \
    && rosdep update

# Add custom pip rules
COPY custom-pip-rules.conf   /etc/pip.conf

# Add custom apt sources for bundling
COPY ${UBUNTU_VERSION}-sources.yaml /opt/cross/apt-sources.yaml
