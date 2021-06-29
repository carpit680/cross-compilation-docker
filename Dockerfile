ARG ROS_VERSION=foxy
ARG UBUNTU_VERSION=focal

# we start at ubuntu-20.04 (focal) with ROS 2 Foxy preinstalled as our base image
FROM arm64v8/ros:foxy-ros-base-focal
ENV PATH="/root/.local/bin:${PATH}"

# Copy qemu from the host machine. This will be the interpreter for anything and everything that runs inside this container. Since the host is x86, an ARM64->x86 interpreter is required.
COPY qemu-aarch64-static /usr/bin

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Installed for convenience
RUN apt-get update && apt-get install -y nano

# Install Python and colcon
RUN apt-get update && apt-get install -y \
   python \
   python3-apt \
   curl \
   python3-pip \
   python-pip \   
 && python3 -m pip install -U colcon-ros-bundle

# Add custom rosdep rules
RUN echo "yaml https://s3-us-west-2.amazonaws.com/rosdep/python.yaml" > /etc/ros/rosdep/sources.list.d/20-aws-python.list \
   && rosdep update

# Add custom pip rules
COPY custom-pip-rules.conf /etc/pip.conf

# Add custom apt sources for bundling
COPY focal-sources-arm64.yaml /opt/cross/apt-sources.yaml