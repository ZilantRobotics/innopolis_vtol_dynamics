# This software is distributed under the terms of the GPL v3 License.
# Copyright (c) 2021-2024 Dmitry Ponomarev.
# Author: Dmitry Ponomarev <ponomarevda96@gmail.com>
ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO
LABEL description="UAV simulator"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/uav_hitl_simulator

# 1. Install requirements
COPY scripts/install.sh scripts/install.sh
RUN apt-get update && ./scripts/install.sh --yes && apt-get clean

# 2. Copy the source files
COPY . /catkin_ws/src/uav_hitl_simulator

# 3. Build ROS
RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd /catkin_ws/ && git config --global http.sslverify false && catkin build
RUN echo source scripts/run_sim.sh ros >> ~/.bashrc

CMD echo "main process has been started"                                        &&  \
    source /opt/ros/$ROS_DISTRO/setup.bash && source /catkin_ws/devel/setup.bash &&  \
    echo "container has been finished"
