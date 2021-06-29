#!/bin/bash

install_qemu() {
  # Install dependencies
  apt-get update
  apt-get install -y \
  binfmt-support \
  libglib2.0-dev \
  libfdt-dev \
  libpixman-1-dev \
  zlib1g-dev \
  qemu \
  qemu-user-static

  cp /usr/bin/qemu-aarch64-static $WORK_DIR
}

install_docker() {
  apt-get remove docker docker-engine docker.io
  apt-get update
  apt-get install -y \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common

  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

  apt-key fingerprint 0EBFCD88

  sudo add-apt-repository \
  "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) \
  stable"

  apt-get update
  apt-get install -y docker-ce
  docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
}

set -e

if [[ $EUID -ne 0 ]]; then
  echo "This script must be run as root"
  exit 1
fi

WORK_DIR=$(pwd)

if [ ! -f "$WORK_DIR/Dockerfile.focal" ]; then
  echo "Must be run in the same folder as a Dockerfile"
  exit 1
fi

if [ ! -f "$WORK_DIR/qemu-aarch64-static" ]; then
  echo "Installing qemu..."
  install_qemu $WORK_DIR
fi

if [ ! -x "$(command -v docker)" ]; then
  echo "Docker is not installed. Installing..."
  install_docker
fi

# Build the Docker image
UBUNTU_VERSION=`lsb_release -cs`
if [ $UBUNTU_VERSION = 'focal' ]; then
  ROS_VERSION="foxy"
else
  ROS_VERSION="dashing"
fi

docker build -t ros-cross-compile:arm64 --build-arg ROS_VERSION=${ROS_VERSION} --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} -f ${WORK_DIR}/Dockerfile.${UBUNTU_VERSION} "${WORK_DIR}"

