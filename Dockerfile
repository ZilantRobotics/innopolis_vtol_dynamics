ARG ROS_DISTRO=melodic

FROM ros:$ROS_DISTRO
LABEL description="Inno VTOL simulator"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/inno_vtol_simulator


# 1. Install basic requirements
RUN apt-get update                                                              &&  \
    apt-get upgrade -y                                                          &&  \
    apt-get install -y  git                                                         \
                        iproute2                                                    \
                        ros-$ROS_DISTRO-catkin                                      \
                        net-tools                                                   \
                        tcpdump                                                     \
                        python3-pip                                                 \
                        python3-catkin-tools
RUN if [[ "$ROS_DISTRO" = "melodic" ]] ; then apt-get install -y python-pip python-catkin-tools ; fi

# 2. Install packages requirements
# 2.1. innopolis_vtol_dynamics
RUN sudo apt-get install -y ros-$ROS_DISTRO-mavros                                  \
                            ros-$ROS_DISTRO-mavlink                                 \
                            ros-$ROS_DISTRO-tf                                      \
                            ros-$ROS_DISTRO-tf2                                     \
                            ros-$ROS_DISTRO-tf2-ros                                 \
                            psmisc

# 2.2. inno-sim-interface
RUN sudo apt-get install -y ros-$ROS_DISTRO-rosauth                             &&  \
    pip install bson pymongo protobuf Pillow twisted

# 2.3 uavcan_tools
RUN sudo apt-get install -y udev

# 3. Copy repository
# @todo use git clone with submodules instead of COPY
COPY uav_dynamics/ uav_dynamics/
COPY inno_sim_interface/ inno_sim_interface/
COPY communicators/ communicators/
COPY catkin_build.sh catkin_build.sh

# 4. Setup packages
# 4.1. geographiclib_conversions
RUN mkdir -p /usr/local/share/GeographicLib/magnetic                            &&  \
    cd uav_dynamics/geographiclib_conversions/wmm2020/magnetic                  &&  \
    cp wmm2020.wmm /usr/local/share/GeographicLib/magnetic/wmm2020.wmm          &&  \
    cp wmm2020.wmm.cof /usr/local/share/GeographicLib/magnetic/wmm2020.wmm.cof

# 4.2. drone_communicators
RUN cd communicators/drone_communicators                                        &&  \
    ./scripts/install_requirements.sh                                           &&  \
    ./scripts/install_libuavcan.sh

# 5. Build ROS
RUN source /opt/ros/$ROS_DISTRO/setup.bash                                      &&  \
    cd ../../                                                                   &&  \
    catkin build

# 6. Copy scripts
COPY scripts/ scripts/

CMD echo "main process has been started"                                        &&  \
    source /opt/ros/$ROS_DISTRO/setup.bash                                      &&  \
    source /catkin_ws/devel/setup.bash                                          &&  \
    roslaunch innopolis_vtol_dynamics hitl.launch                               &&  \
    echo "container has been finished"