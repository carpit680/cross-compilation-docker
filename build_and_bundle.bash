#!/bin/bash

cd ws

apt update

rosdep install --from-paths src --ignore-src -r -y

pip3 install -U markupsafe==2.0.0

colcon build --build-base armhf_build --install-base armhf_install

colcon bundle --build-base armhf_build --install-base armhf_install --bundle-base armhf_bundle --apt-sources-list /opt/cross/apt-sources.yaml

mv armhf_bundle/output.tar armhf_bundle/foodlGo_robot_bundle.armhf.tar

aws s3 cp armhf_bundle/output.tar s3://foodl-robomaker-bucket/foodlGo_robot_bundle.armhf.tar
