# This Dockerfile is a hint about how to run PX4 ROS-noetic flight stack in Docker
# Tested in Ubuntu-22.04 and PX4-Autopolot v1.14.0.
# Suitable for Quadrotor MAVLink SITL mode of UAV HITL Simulator: https://github.com/RaccoonlabDev/innopolis_vtol_dynamics.git
# Origin: https://gist.github.com/PonomarevDA/1a600056a3c01cebae878b642dafe6ea
# Usage:
# 1. Download:
# mkdir PX4-Autopilot-Docker && cd PX4-Autopilot-Docker
# wget https://gist.githubusercontent.com/PonomarevDA/1a600056a3c01cebae878b642dafe6ea/raw/Dockerfile
# 2. Build the Docker image:
# docker build -t ponomarevda/px4-sitl .
# 3. Run in interactive mode:
# docker container run --rm -it --net=host ponomarevda/px4-sitl /bin/bash
# 4. Then run launch file:
# roslaunch px4 px4.launch vehicle:=iris
# Or run at once:
# docker run --rm -it --net=host ponomarevda/px4-sitl roslaunch px4 px4.launch vehicle:=iris

ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO
LABEL description="PX4 MAVLink SITL simulator"
SHELL ["/bin/bash", "-c"]
WORKDIR /root/catkin_ws/

# 1. Install basic requirements
RUN apt-get update                                  &&  \
    apt-get upgrade -y                              &&  \
    apt-get install -y git                              \
                       ros-$ROS_DISTRO-catkin           \
                       python3-pip                      \
                       python3-catkin-tools

# 2. Build ROS
RUN source /opt/ros/$ROS_DISTRO/setup.bash          &&  \
    git config --global http.sslverify false        &&  \
    mkdir ~/catkin_ws/src                           &&  \
    catkin build

# 3. Install PX4 ubuntu requirements
RUN apt-get install -y wget
RUN wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/ubuntu.sh && \
    wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/requirements.txt
RUN bash ubuntu.sh

# 4. Install PX4 ubuntu sim ros requirements
RUN wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh
RUN source /opt/ros/$ROS_DISTRO/setup.bash  && \
    source ~/catkin_ws/devel/setup.bash     && \
    bash ubuntu_sim_ros_melodic.sh

# 5. Clone PX4-Autopilot
RUN git clone --branch main https://github.com/PX4/PX4-Autopilot --recursive ~/PX4-Autopilot

# 6. Build PX4 for VTOL
RUN cd ~/PX4-Autopilot && DONT_RUN=1 make px4_sitl gazebo_standard_vtol

# 7. Append to .bashrc
RUN rm ~/.bashrc && cp /etc/skel/.bashrc ~/.bashrc
RUN source ~/.bashrc                                                && \
    cd ~/PX4-Autopilot                                              && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash"   >> ~/.bashrc    && \
    echo "source ~/catkin_ws/devel/setup.bash"      >> ~/.bashrc    && \
    echo "export ROS_PACKAGE_PATH=/opt/ros/noetic/share:~/PX4-Autopilot:~/PX4-Autopilot/Tools/sitl_gazebo" >> ~/.bashrc && \
    echo "cd ~/PX4-Autopilot"                       >> ~/.bashrc    && \
    echo "echo Hello, there. This is PX4 Flight Stack container." >> ~/.bashrc    && \
    echo "echo Your next steps can be:" >> ~/.bashrc && \
    echo "echo -   roslaunch px4 px4.launch vehicle:=iris" >> ~/.bashrc && \
    echo "echo -   roslaunch px4 px4.launch vehicle:=standard_vtol" >> ~/.bashrc && \
    echo "echo For details please visit: https://docs.px4.io/main/en/sim_gazebo_gz/vehicles.html" >> ~/.bashrc && \
    echo "echo P.S. ROS and PX4 environment variables have been set." >> ~/.bashrc

CMD echo -e "\nContainer has been started!" && \
    source ~/.bashrc                        && \
    roslaunch px4 px4.launch vehicle:=iris  && \
    echo -e "\nContainer has been finished!"
