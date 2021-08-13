FROM ros:melodic
LABEL description="Inno VTOL simulator"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/inno_vtol_simulator


# 1. Install basic requirements
RUN apt-get update                                                              &&  \
    apt-get upgrade -y                                                          &&  \
    apt-get install -y  git                                                         \
                        iproute2                                                    \
                        ros-melodic-catkin                                          \
                        python-catkin-tools                                         \
                        net-tools                                                   \
                        python-pip                                                  \
                        tcpdump


# 2. Copy repository
# @todo use git clone with submodules instead of COPY
COPY uav_dynamics/ uav_dynamics/
COPY inno_sim_interface/ inno_sim_interface/
COPY communicators/ communicators/
COPY scripts/uavcan_tools/ scripts/uavcan_tools/
COPY catkin_build.sh catkin_build.sh


# 3. Install requirements
# 3.1. innopolis_vtol_dynamics
RUN sudo apt-get install -y ros-melodic-mav-msgs                                    \
                            ros-melodic-tf                                          \
                            ros-melodic-tf2                                         \
                            ros-melodic-tf2-ros                                     \
                            ros-melodic-mavlink                                     \
                            ros-melodic-mavros

# 3.2. geographiclib_conversions
RUN mkdir -p /usr/local/share/GeographicLib/magnetic                            &&  \
    cd uav_dynamics/geographiclib_conversions/wmm2020/magnetic                  &&  \
    cp wmm2020.wmm /usr/local/share/GeographicLib/magnetic/wmm2020.wmm          &&  \
    cp wmm2020.wmm.cof /usr/local/share/GeographicLib/magnetic/wmm2020.wmm.cof

# 3.3. inno-sim-interface
RUN sudo apt-get install -y ros-melodic-rosauth                                 &&  \
    pip install bson pymongo protobuf Pillow twisted                            &&  \
    sudo apt-get install -y ros-melodic-turtlesim

# 3.4. drone_communicators
RUN cd communicators/drone_communicators                                        &&  \
    ./scripts/install_requirements.sh                                           &&  \
    ./scripts/install_libuavcan.sh

# 3.5 uavcan_tools
RUN sudo apt-get install -y udev

# 4. Build ROS
RUN source /opt/ros/melodic/setup.bash                                          &&  \
    cd ../../                                                                   &&  \
    catkin build


CMD echo "main process has been started"                                        &&  \
    source /opt/ros/melodic/setup.bash                                          &&  \
    source /catkin_ws/devel/setup.bash                                          &&  \
    roslaunch innopolis_vtol_dynamics hitl.launch                               &&  \
    echo "container has been finished"