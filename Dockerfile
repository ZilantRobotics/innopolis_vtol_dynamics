FROM ros:melodic
LABEL description="Inno VTOL simulator"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src/inno_vtol_simulator


# 1. Install basic requirements
RUN apt-get update                                  &&  \
    apt-get upgrade -y                              &&  \
    apt-get install -y  git                             \
                        iproute2                        \
                        ros-melodic-catkin              \
                        python-catkin-tools             \
                        net-tools                       \
                        python-pip                      \
                        tcpdump


# 2. Install PX4-Autopilot
# Here we should use our inno-dynamics version of PX4-Autopilot
# But it is not public at this moment
# For HITL it should have Inno VTOL airframe and mixer
# Without correct mixer VTOL can't perform transition correctly
RUN cd ../../..                                     &&  \
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive  &&  \
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh  &&  \
    wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh &&  \
    bash ubuntu_sim_ros_melodic.sh


# Build PX4-Autopilot
# Hack: Last using version based on 96a6b5c, it's better to update it
# There is problem with GPS in newer versions, try to fix it in future
RUN cd ../../../PX4-Autopilot                       &&  \
    git checkout 96a6b5c                            &&  \
    git submodule update --init --recursive && make clean && make distclean     &&  \
    DONT_RUN=1 make px4_sitl gazebo_standard_vtol
# (optional) make px4_fmu-v5_default


# 3. Copy repository
# @todo use git clone with submodules instead of COPY
COPY uav_dynamics/ uav_dynamics/
COPY inno_sim_interface/ inno_sim_interface/
COPY communicators/ communicators/
COPY catkin_build.sh catkin_build.sh


# 4. Install requirements
# 4.1. innopolis_vtol_dynamics
RUN sudo apt-get install -y ros-melodic-mav-msgs        \
                            ros-melodic-tf              \
                            ros-melodic-tf2             \
                            ros-melodic-tf2-ros         \
                            ros-melodic-mavlink         \
                            ros-melodic-mavros

# 4.2. geographiclib_conversions
RUN mkdir -p /usr/local/share/GeographicLib/magnetic && \
    cp uav_dynamics/geographiclib_conversions/wmm2020/magnetic/wmm2020.wmm /usr/local/share/GeographicLib/magnetic/wmm2020.wmm   &&  \
    cp uav_dynamics/geographiclib_conversions/wmm2020/magnetic/wmm2020.wmm.cof /usr/local/share/GeographicLib/magnetic/wmm2020.wmm.cof

# 4.3. inno-sim-interface
RUN sudo apt-get install -y ros-melodic-rosauth
RUN pip install bson pymongo protobuf Pillow twisted

# 4.4. drone_communicators
RUN cd communicators/drone_communicators            &&  \
    ./scripts/install_requirements.sh               &&  \
    ./scripts/install_libuavcan.sh


# 5. Build ROS
RUN source /opt/ros/melodic/setup.bash              &&  \
    cd ../../                                       &&  \
    catkin build


CMD echo "main process has been started"            &&  \
    source /opt/ros/melodic/setup.bash              &&  \
    source /catkin_ws/devel/setup.bash              &&  \
    roslaunch innopolis_vtol_dynamics hitl.launch   &&  \
    echo "container has been finished"