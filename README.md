# cross-compilation-docker
cross-compilation docker repository for ROS robot application using colcon for AWS Robomaker deployment to armhf or arm64

You’re back online
We've lost our connection to you. Your changes will be saved when we reconnect. Trying to reconnect


Normal text
















A



Publish

Close

Lab Robotics
Pages
Lab Robotics Home
LRO Software
How-to articles
Building and bundling

Building and bundling
This page describes in detail the entire process of building and bundling a robot application using colcon for AWS Robomaker deployment.

Some Context
AWS Robomaker requires a ROS robot application bundle specific to the robot architecture and this bundle is created using colcon. 

The robot in our case is either an armhf or an arm64(backward compatible with armhf and so this document focuses on armhf from now on) based and to build and bundle our application for armhf on an amd64 host we usually use AWS’s cross-compilation docker(follow The Alternative steps to use that) but it occasionally doesn’t work due to any buggy software updates and so, we have the option to follow the steps given below to set up the cross-compilation docker on any amd64 Linux host to be independent of the AWS platform for development and further reduce the costs. This comes with a downside regularly having to update the repository manually but also gives reliability.

Pre-requisites
amd64 based Linux host

ROS package

All other pre-requisites are installed in the steps given below. Skip whatever is already installed.

Setup
Install git and Wget if not already installed.

sudo apt install git wget
Use quick install script for Ros Melodic desktop.

wget https://raw.githubusercontent.com/carpit680/quick_install/master/ros/ros_install.sh && chmod 755 ros_install.sh && ./ros_install.sh melodic
Source .bashrc or .zshrc file to access ROS.

source ~/.bashrc # or ~/.zshrc
Use quick install script for docker.

wget https://get.docker.com && mv index.html docker_install.sh && chmod 755 docker_install.sh && ./docker_install.sh
Install the qemu packages.

sudo apt-get install qemu binfmt-support qemu-user-static 
This step will execute the registering scripts.

sudo docker run --rm --privileged multiarch/qemu-user-static --reset -p yes 
Clone cross-compilation repository.

git clone https://github.com/carpit680/cross-compilation-docker
Clone your ROS package if not done already.

git clone <link-to-your-ros-package> 
Go to the cross-compilation directory.

cd cross-compilation-docker/
Build cross-compilation docker image and set up all the necessary components.

sudo bash bin/build_image.bash
Building and Bundling
Now that the cross-compilation docker is set up, we will start building and bundling the robot application.

Go to your ROS workspace.

cd /path/to/your/ros/workspace
Run the cross-compilation docker and mount the workspace as ws.

sudo docker run -v $(pwd):/ws -it ros-cross-compile:armhf
Go to the workspace inside the docker container

cd ws
Update the repositories.

apt update
Install all the required dependencies for your ROS package.

rosdep install --from-paths src --ignore-src -r -y
Build the package and store the resultant build and install files in relevant folders.

colcon build --build-base armhf_build --install-base armhf_install
Bundle the package and store the resultant build, bundle, and install files in relevant folders.

colcon bundle --build-base armhf_build --install-base armhf_install --bundle-base armhf_bundle --apt-sources-list /opt/cross/apt-sources.yaml
Exit the Docker container.

exit
Rename the output.tar file to <bundle-name>.armhf.tar

mv amrhf_bundle/output.tar armhf_bundle/<bundle-name>.armhf.tar
Now, upload this robot application bundle to your S3 bucket to create a robot application for your specific architecture.

The alternative
The only other alternative at the time of compiling this document is using AWS’s own cross-compilation docker that has similar(at least at the time of writing) building and bundling steps as my method is based on AWS’s method itself. Their method would be more up-to-date but at times not reliable and it would definitely cost us in terms of money.

AWS instructions for cross-compilation for ARMHF:

cd /opt/robomaker/cross-compilation-dockerfile/
sudo bin/build_image.bash      
cd /path/to/your/ros/workspace
sudo docker run -v $(pwd):/ws -it ros-cross-compile:armhf
cd ws
apt update
rosdep install --from-paths src --ignore-src -r -y
colcon build --build-base armhf_build --install-base armhf_install
colcon bundle --build-base armhf_build --install-base armhf_install --bundle-base armhf_bundle --apt-sources-list /opt/cross/apt-sources.yaml
exit
Now, copy the robot application bundle to your S3 bucket and give it an apt name with the armhf.tar extension.

aws s3 cp armhf_bundle/output.tar s3://<s3-bucket-name>/<bundle-name>.armhf.tar

Resources
For extra reading
Qualcomm Robomaker deployment tutorial

Cross-compilation docker in depth

Content by Label
