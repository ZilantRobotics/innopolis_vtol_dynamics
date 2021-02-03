/****************************************************************************
 *
 *   Copyright (c) 2020 ThunderFly s.r.o.. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_communicator.cpp
 *
 * @author Dmitry Ponomarev <ponomarevda96@gmail.com>
 * @author Roman Fedorenko <frontwise@gmail.com>
 * @author ThunderFly s.r.o., Vít Hanousek <info@thunderfly.cz>
 * @url https://github.com/ThunderFly-aerospace
 *
 * PX4 communication socket.
 */

#include <iostream>
#include <ros/ros.h>
#include <mavlink/v2.0/common/mavlink.h>
#include <poll.h>
#include <netinet/tcp.h>

#include "mavlink_communicator.h"
#include "cs_converter.hpp"

const std::string NODE_NAME = "Mavlink PX4 Communicator";

int main(int argc, char **argv){
    // 1. Init node
    ros::init(argc, argv, NODE_NAME.c_str());
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::NodeHandle nodeHandler;

    // 2. Define which mavlink actuators format should we use (
    // - quad rotors with actuators cmd size = 4
    // - or VTOL with actuators cmd size = 8)
    std::string vehicle;
    if(!nodeHandler.getParam("/inno_dynamics_sim/vehicle", vehicle)){
        ROS_ERROR_STREAM(NODE_NAME << "There is no /inno_dynamics_sim/vehicle");
        ros::shutdown();
    }
    bool isCopterAirframe;
    const std::string VEHICLE_IRIS = "iris";
    const std::string VEHICLE_STANDARD_VTOL = "standard_vtol";
    if(vehicle == VEHICLE_STANDARD_VTOL){
        isCopterAirframe = false;
    }else if(vehicle == VEHICLE_IRIS){
        isCopterAirframe = true;
    }else{
        ROS_ERROR_STREAM(NODE_NAME << "There is no at least one of required simulator parameters.");
        ros::shutdown();
    }

    // 3. Get altitude reference position
    float altRef = 0;
    const std::string SIM_PARAMS_PATH = "/uav/sim_params/";
    if(!ros::param::get(SIM_PARAMS_PATH + "alt_ref", altRef)){
        ROS_ERROR_STREAM(NODE_NAME << "There is no reference altitude parameter.");
        ros::shutdown();
    }
    altRef = 0;

    int px4id = 0;

    MavlinkCommunicator communicator(nodeHandler, altRef);
    if(communicator.Init(px4id, isCopterAirframe) != 0) {
        ROS_ERROR("Unable to Init PX4 Communication");
        ros::shutdown();
    }

    ros::spin();
    return 0;
}

MavlinkCommunicator::MavlinkCommunicator(ros::NodeHandle nodeHandler, float alt_home) :
    nodeHandler_(nodeHandler), ALT_HOME(alt_home){
    normalDistribution_ = std::normal_distribution<double>(0.0f, 0.1f);

    magNoise_ = 0.0000051;
    baroAltNoise_ = 0.0001;
    tempNoise_ = 0.001;
    absPressureNoise_ = 0.001;
    // Let's use a rough guess of 0.01 hPa as the standard devitiation which roughly yields
    // about +/- 1 m/s noise.
    diffPressureNoise_ = 0.01;
}

int MavlinkCommunicator::Init(int portOffset, bool is_copter_airframe){
    isCopterAirframe_ = is_copter_airframe;

    memset((char *) &simulatorMavlinkAddr_, 0, sizeof(simulatorMavlinkAddr_));
    memset((char *) &px4MavlinkAddr_, 0, sizeof(px4MavlinkAddr_));
    simulatorMavlinkAddr_.sin_family = AF_INET;
    simulatorMavlinkAddr_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    simulatorMavlinkAddr_.sin_port = htons(PORT_BASE + portOffset);

    if ((listenMavlinkSock_ = socket(AF_INET, SOCK_STREAM, 0)) < 0){
        ROS_ERROR_STREAM(NODE_NAME << ": Creating TCP socket failed: " << strerror(errno));
        return -1;
    }

    // do not accumulate messages by waiting for ACK
    int yes = 1;
    int result = setsockopt(listenMavlinkSock_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
    if (result != 0){
        ROS_ERROR_STREAM(NODE_NAME << ": setsockopt failed: " << strerror(errno));
    }

    // try to close as fast as posible
    struct linger nolinger;
    nolinger.l_onoff = 1;
    nolinger.l_linger = 0;
    result = setsockopt(listenMavlinkSock_, SOL_SOCKET, SO_LINGER, &nolinger, sizeof(nolinger));
    if (result != 0){
        ROS_ERROR_STREAM(NODE_NAME << ": setsockopt failed: " << strerror(errno));
    }

    // The socket reuse is necessary for reconnecting to the same address
    // if the socket does not close but gets stuck in TIME_WAIT. This can happen
    // if the server is suddenly closed, for example, if the robot is deleted in gazebo.
    int socket_reuse = 1;
    result = setsockopt(listenMavlinkSock_, SOL_SOCKET, SO_REUSEADDR, &socket_reuse, sizeof(socket_reuse));
    if (result != 0){
        ROS_ERROR_STREAM(NODE_NAME << ": setsockopt failed: " << strerror(errno));
    }

    // Same as above but for a given port
    result = setsockopt(listenMavlinkSock_, SOL_SOCKET, SO_REUSEPORT, &socket_reuse, sizeof(socket_reuse));
    if (result != 0){
        ROS_ERROR_STREAM(NODE_NAME << ": setsockopt failed: " << strerror(errno));
    }


    if (bind(listenMavlinkSock_, (struct sockaddr *)&simulatorMavlinkAddr_, sizeof(simulatorMavlinkAddr_)) < 0){
        ROS_ERROR_STREAM(NODE_NAME << ": bind failed: " << strerror(errno));
    }

    errno = 0;
    result = listen(listenMavlinkSock_, 5);
    if (result < 0){
        ROS_ERROR_STREAM(NODE_NAME << ": listen failed: " << strerror(errno));
    }

    unsigned int px4_addr_len = sizeof(px4MavlinkAddr_);
    ROS_INFO_STREAM(NODE_NAME << ": waiting for connection from PX4...");
    while(true) {
        px4MavlinkSock_ = accept(listenMavlinkSock_,
                                (struct sockaddr *)&px4MavlinkAddr_,
                                &px4_addr_len);
        if (px4MavlinkSock_ < 0){
            ROS_ERROR_STREAM(NODE_NAME << ": accept failed: " << strerror(errno));
        }else{
            ROS_INFO_STREAM(NODE_NAME << ": PX4 Connected.");
            break;
        }
    }

    constexpr char MAG_TOPIC_NAME[]      = "/uav/mag";
    constexpr char IMU_TOPIC_NAME[]      = "/uav/imu";
    constexpr char GPS_POSE_TOPIC_NAME[] = "/uav/gps_position";
    constexpr char ATTITUDE_TOPIC_NAME[] = "/uav/attitude";
    constexpr char VELOCITY_TOPIC_NAME[] = "/uav/velocity";

    constexpr char ACTUATOR_TOPIC_NAME[] = "/uav/actuators";
    constexpr char ARM_TOPIC_NAME[]      = "/uav/arm";

    attitudeSub_ = nodeHandler_.subscribe(ATTITUDE_TOPIC_NAME,
                                          1,
                                          &MavlinkCommunicator::attitudeCallback,
                                          this);
    gpsSub_ = nodeHandler_.subscribe(GPS_POSE_TOPIC_NAME,
                                    1,
                                    &MavlinkCommunicator::gpsCallback,
                                    this);
    imuSub_ = nodeHandler_.subscribe(IMU_TOPIC_NAME,
                                     1,
                                     &MavlinkCommunicator::imuCallback,
                                     this);
    magSub_ = nodeHandler_.subscribe(MAG_TOPIC_NAME,
                                     1,
                                     &MavlinkCommunicator::magCallback,
                                     this);
    velocitySub_ = nodeHandler_.subscribe(VELOCITY_TOPIC_NAME,
                                          1,
                                          &MavlinkCommunicator::velocityCallback,
                                          this);

    armPub_ = nodeHandler_.advertise<std_msgs::Bool>(ARM_TOPIC_NAME, 1);
    actuatorsPub_ = nodeHandler_.advertise<sensor_msgs::Joy>(ACTUATOR_TOPIC_NAME, 1);

    mainTask_ = std::thread(&MavlinkCommunicator::communicate, this);
    mainTask_.detach();

    return result;
}

void MavlinkCommunicator::communicate(){
    while(ros::ok()){
        auto crnt_time = std::chrono::system_clock::now();
        auto sleed_period = std::chrono::microseconds(int(5000));
        auto time_point = crnt_time + sleed_period;

        auto gpsTimeUsec = gpsPositionMsg_.header.stamp.toNSec() / 1000;
        auto imuTimeUsec = imuMsg_.header.stamp.toNSec() / 1000;

        std::vector<double> actuators(8);
        if(Receive(false, isArmed_, actuators) == 1){
            publishActuators(actuators);
            publishArm();
        }

        /**
         * @note For some reasons sometimes PX4 ignores GPS all messages after first if we
         * send it too soon. So, just ignoring first few messages is ok.
         * @todo Understand why and may be develop a better approach
         */
        if (gpsMsgCounter_ >= 5 && gpsTimeUsec >= lastGpsTimeUsec_ + GPS_PERIOD_US){
            lastGpsTimeUsec_ = gpsTimeUsec;

            if(SendHilGps(gpsTimeUsec, linearVelocityNed_, gpsPosition_) == -1){
                ROS_ERROR_STREAM_THROTTLE(1, NODE_NAME << ": GPS failed." << strerror(errno));
            }
        }
        if (imuTimeUsec >= lastImuTimeUsec_ + IMU_PERIOD_US){
            lastImuTimeUsec_ = imuTimeUsec;

            /**
             * @note Hack: at this moment simulator sends fluToEnu attitudeinstead of frdToNed
             */
            auto attitudeFluToEnu = attitudeFrdToNed_;

            auto linearVelocityEnu = Converter::nedToEnu(linearVelocityNed_);
            auto linearVelocityFrd = Converter::enuToFrd(linearVelocityEnu, attitudeFluToEnu);
            int status = SendHilSensor(imuTimeUsec,
                                       gpsPosition_,
                                       magFrd_,
                                       linearVelocityFrd,
                                       accFrd_,
                                       gyroFrd_);

            if(status == -1){
                ROS_ERROR_STREAM_THROTTLE(1, NODE_NAME << "Imu failed." << strerror(errno));
            }
        }

        std::this_thread::sleep_until(time_point);
    }
}

void MavlinkCommunicator::attitudeCallback(geometry_msgs::QuaternionStamped::Ptr attitude){
    attitudeMsg_ = *attitude;
    attitudeFrdToNed_.x() = attitude->quaternion.x;
    attitudeFrdToNed_.y() = attitude->quaternion.y;
    attitudeFrdToNed_.z() = attitude->quaternion.z;
    attitudeFrdToNed_.w() = attitude->quaternion.w;
}

void MavlinkCommunicator::gpsCallback(sensor_msgs::NavSatFix::Ptr gpsPosition){
    gpsPositionMsg_ = *gpsPosition;
    gpsMsgCounter_++;
    gpsPosition_[0] = gpsPosition->latitude;
    gpsPosition_[1] = gpsPosition->longitude;
    gpsPosition_[2] = gpsPosition->altitude;
}

void MavlinkCommunicator::imuCallback(sensor_msgs::Imu::Ptr imu){
    imuMsg_ = *imu;
    accFrd_[0] = imu->linear_acceleration.x;
    accFrd_[1] = imu->linear_acceleration.y;
    accFrd_[2] = imu->linear_acceleration.z;

    gyroFrd_[0] = imu->angular_velocity.x;
    gyroFrd_[1] = imu->angular_velocity.y;
    gyroFrd_[2] = imu->angular_velocity.z;
}

void MavlinkCommunicator::magCallback(sensor_msgs::MagneticField::Ptr mag){
    magMsg_ = *mag;
    magFrd_[0] = mag->magnetic_field.x;
    magFrd_[1] = mag->magnetic_field.y;
    magFrd_[2] = mag->magnetic_field.z;
}

void MavlinkCommunicator::velocityCallback(geometry_msgs::Twist::Ptr velocity){
    velocityMsg_ = *velocity;
    linearVelocityNed_[0] = velocity->linear.x;
    linearVelocityNed_[1] = velocity->linear.y;
    linearVelocityNed_[2] = velocity->linear.z;
}

void MavlinkCommunicator::publishArm(){
    std_msgs::Bool armMsg;
    armMsg.data = isArmed_;
    armPub_.publish(armMsg);
}

void MavlinkCommunicator::publishActuators(const std::vector<double>& actuators) const{
    // it's better to move it to class members to prevent initialization on each publication
    sensor_msgs::Joy actuatorsMsg;
    actuatorsMsg.header.stamp = ros::Time::now();
    for(auto actuator : actuators){
        actuatorsMsg.axes.push_back(actuator);
    }
    actuatorsPub_.publish(actuatorsMsg);
}

int MavlinkCommunicator::Clean(){
    close(px4MavlinkSock_);
    close(listenMavlinkSock_);
    return 0;
}

/**
 * @return result
 * -1 means error,
 * 0 means ok
 */
int MavlinkCommunicator::SendHilSensor(unsigned int time_usec,
                                   Eigen::Vector3d gpsPosition,
                                   Eigen::Vector3d magFrd,
                                   Eigen::Vector3d linVelFrd,
                                   Eigen::Vector3d accFrd,
                                   Eigen::Vector3d gyroFrd){
    // Output data
    mavlink_hil_sensor_t sensor_msg;
    sensor_msg.time_usec = time_usec;

    // 1. Fill acc and gyro in FRD frame
    sensor_msg.xacc = accFrd[0];
    sensor_msg.yacc = accFrd[1];
    sensor_msg.zacc = accFrd[2];
    sensor_msg.xgyro = gyroFrd[0];
    sensor_msg.ygyro = gyroFrd[1];
    sensor_msg.zgyro = gyroFrd[2];
    sensor_msg.fields_updated = SENS_ACCEL | SENS_GYRO;

    // 2. Fill Magnetc field with noise
    if (time_usec - lastMagTimeUsec_ > MAG_PERIOD_US){
        sensor_msg.xmag = magFrd[0] + magNoise_ * normalDistribution_(randomGenerator_);
        sensor_msg.ymag = magFrd[1] + magNoise_ * normalDistribution_(randomGenerator_);
        sensor_msg.zmag = magFrd[2] + magNoise_ * normalDistribution_(randomGenerator_);
        sensor_msg.fields_updated |= SENS_MAG;
        lastMagTimeUsec_ = time_usec;
    }

    // 3. Fill Barometr and diff pressure
    if (time_usec - lastBaroTimeUsec_ > BARO_PERIOD_US){
        // 3.1. ISA model for pressure, density, temperature
        // (for the tropsphere - valid up to 11km above MSL)
        const float PRESSURE_MSL_HPA = 1013.250f;
        const float TEMPERATURE_MSL_KELVIN = 288.0f;
        const float RHO_MSL = 1.225f;

        const float LAPSE_TEMPERATURE_RATE = 1 / 152.4; // 0.00656

        float alt_msl = gpsPosition.z();

        float temperature_local = TEMPERATURE_MSL_KELVIN - LAPSE_TEMPERATURE_RATE * alt_msl;
        float pressure_ratio = powf((TEMPERATURE_MSL_KELVIN/temperature_local), 5.256f);
        const float density_ratio = powf((TEMPERATURE_MSL_KELVIN/temperature_local), 4.256f);
        float rho = RHO_MSL / density_ratio;

        // 3.2. temperature in Celsius
        sensor_msg.temperature = temperature_local - 273.0f;
        sensor_msg.temperature += tempNoise_ * normalDistribution_(randomGenerator_);

        // 3.3. abs pressure
        sensor_msg.abs_pressure = PRESSURE_MSL_HPA / pressure_ratio;
        sensor_msg.abs_pressure += absPressureNoise_ * normalDistribution_(randomGenerator_);

        // 3.4. pressure altitude including effect of pressure noise
        sensor_msg.pressure_alt = gpsPosition.z();
        sensor_msg.pressure_alt += baroAltNoise_ * normalDistribution_(randomGenerator_);

        // 3.5. diff pressure in hPa (Note: ignoring tailsitter case here)
        sensor_msg.diff_pressure = 0.005f * rho * linVelFrd.norm() * linVelFrd.norm();
        sensor_msg.diff_pressure += diffPressureNoise_ * normalDistribution_(randomGenerator_);

        sensor_msg.fields_updated |= SENS_BARO | SENS_DIFF_PRESS;
        lastBaroTimeUsec_ = time_usec;
    }


    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int packetlen;
    mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
    packetlen = mavlink_msg_to_send_buffer(buffer, &msg);

    if(send(px4MavlinkSock_, buffer, packetlen, 0) != packetlen){
        return -1;
    }
    return 0;
}


/**
 * @return result
 * -1 means error,
 * 0 means ok
 */
int MavlinkCommunicator::SendHilGps(unsigned int time_usec,
                                Eigen::Vector3d linearVelNed,
                                Eigen::Vector3d gpsPosition){
    // Fill gps msg
    mavlink_hil_gps_t hil_gps_msg;
    hil_gps_msg.time_usec = time_usec;
    hil_gps_msg.fix_type = 3;
    hil_gps_msg.lat = gpsPosition.x() * 1e7;
    hil_gps_msg.lon = gpsPosition.y() * 1e7;
    hil_gps_msg.alt = gpsPosition.z() * 1000;
    hil_gps_msg.eph = 100;
    hil_gps_msg.epv = 100;
    hil_gps_msg.vn = linearVelNed.x() * 100;
    hil_gps_msg.ve = linearVelNed.y() * 100;
    hil_gps_msg.vd = linearVelNed.z() * 100;
    hil_gps_msg.vel = std::sqrt(hil_gps_msg.vn * hil_gps_msg.vn + hil_gps_msg.ve * hil_gps_msg.ve);

    // Course over ground
    double cog = -std::atan2(hil_gps_msg.vn, hil_gps_msg.ve) * 180 / 3.141592654 + 90;

    if (cog < 0) {
        cog += 360;
    }
    hil_gps_msg.cog = cog * 100;
    hil_gps_msg.satellites_visible = 10;

    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_gps_msg);
    int packetlen = mavlink_msg_to_send_buffer(buffer, &msg);
    if(packetlen == 0 || send(px4MavlinkSock_, buffer, packetlen, 0) != packetlen){
        return -1;
    }
    return 0;
}

/**
 * @return status
 * -1 means error,
 * 0 means there is no rx command
 * 1 means there is an actuator command
 */
int MavlinkCommunicator::Receive(bool blocking, bool &armed, std::vector<double>& command){
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    struct pollfd fds[1] = {};
    fds[0].fd = px4MavlinkSock_;
    fds[0].events = POLLIN;

    int p = poll(&fds[0], 1, (blocking?-1:2));
    if(p < 0){
        return -1;
    }else if(p == 0){
        return 0;
    }else if(fds[0].revents & POLLIN){
        unsigned int slen = sizeof(px4MavlinkAddr_);
        unsigned int len = recvfrom(px4MavlinkSock_,
                                    buffer,
                                    sizeof(buffer),
                                    0,
                                    (struct sockaddr *)&px4MavlinkAddr_,
                                    &slen);
        mavlink_status_t status;
        for (unsigned i = 0; i < len; ++i){
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)){
                if(msg.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS){
                    mavlink_hil_actuator_controls_t controls;
                    mavlink_msg_hil_actuator_controls_decode(&msg, &controls);

                    armed = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED);
                    isArmed_ = armed;
                    if(armed){
                        command[0] = controls.controls[0];
                        command[1] = controls.controls[1];
                        command[2] = controls.controls[2];
                        command[3] = controls.controls[3];
                        if(isCopterAirframe_ == false){
                            command[4] = controls.controls[4];
                            command[5] = controls.controls[5];
                            command[6] = controls.controls[6];
                            command[7] = controls.controls[7];
                        }
                    }
                    return 1;
                }else if (msg.msgid == MAVLINK_MSG_ID_ESTIMATOR_STATUS){
                    ROS_ERROR_STREAM_THROTTLE(2, NODE_NAME << ": MAVLINK_MSG_ID_ESTIMATOR_STATUS");
                }else{
                    ROS_WARN_STREAM(NODE_NAME << ": unknown msg with msgid = " << msg.msgid);
                }
            }
        }
        ROS_WARN_STREAM(NODE_NAME << ": No cmd");
        return 0;
    }
    return -1;
}
