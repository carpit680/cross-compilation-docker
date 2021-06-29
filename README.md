# cross-compilation-docker
Cross-compilation docker repository for ROS robot application using colcon for AWS Robomaker deployment to armhf or arm64.

This README describes in detail the entire process of building and bundling a robot application using colcon for AWS Robomaker deployment.

## Some Context

AWS Robomaker requires a ROS2 robot application bundle specific to the robot architecture and this bundle is created using colcon. 

The robot in our case is either an armhf or an arm64(backward compatible with armhf but this document focuses on arm64 from now on) based and to build and bundle our application for arm64 on an amd64 host we follow the steps given below to set up the cross-compilation docker on any amd64 Linux host. There aren’t any cross-compilation-dockers for ROS2 foxy at the time of writing this document.



## Pre-requisites

* amd64 based Linux host

* ROS2 package

* git

* All other pre-requisites are installed by the build_image.bash script.

## Setup

1. Clone foxy-arm64 branch of cross-compilation-docker repository.
```
    git clone --single-branch --branch foxy-arm64 https://github.com/carpit680/cross-compilation-docker
```
2. Go to the cross-compilation directory.
```
   cd cross-compilation-docker
```
3. Build cross-compilation docker image, install qemu-aarch64and set up all the necessary components.

> **Warning:** Only if you have made changes since the last time you executed this.

```
    sudo bash build_image.bash
```

## Building and Bundling

Now that the cross-compilation docker is set up, we will start building and bundling the robot application.

1. Go to your ROS workspace.
```
    cd /path/to/your/ros/workspace
```
2. Run the cross-compilation docker and mount the workspace as ws.
```
    sudo docker run -v $(pwd):/ws -it ros-cross-compile:arm64
```
3. Go to the workspace inside the docker container
```
    cd ws
```
4. Update the repositories.
```
    apt update
```
5. Install all the required dependencies for your ROS package.
```
    rosdep install --from-paths src --ignore-src -r -y
```
6. Build the package and store the resultant build and install files in relevant folders.
```
    colcon build --build-base arm64_build --install-base arm64_install
```
7. Bundle the package and store the resultant build, bundle, and install files in relevant folders.
```
    colcon bundle --build-base arm64_build --install-base arm64_install --bundle-base arm64_bundle --apt-sources-list /opt/cross/apt-sources.yaml
```
8. Exit the Docker container.
```
    exit
```
9. Rename the output.tar file to <bundle-name>.armhf.tar
```
    mv amrhf_bundle/output.tar armhf_bundle/<bundle-name>.armhf.tar
```
Now, copy the robot application bundle to your S3 bucket and give it an appropriate name with the arm64.tar extension.
```
aws s3 cp arm64_bundle/output.tar s3://<s3-bucket-name>/<bundle-name>.arm64.tar
```
