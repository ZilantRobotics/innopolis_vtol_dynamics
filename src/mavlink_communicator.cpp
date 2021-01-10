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

MavlinkCommunicator::MavlinkCommunicator(float alt_home) :
    ALT_HOME(alt_home){
    standard_normal_distribution_ = std::normal_distribution<double>(0.0f, 0.1f);

    mag_noise = 0.0000051;
    baro_alt_noise = 0.0001;
    temp_noise = 0.001;
    abs_pressure_noise = 0.001;
    // Let's use a rough guess of 0.01 hPa as the standard devitiation which roughly yields
    // about +/- 1 m/s noise.
    diff_pressure_noise = 0.01;

    last_mag_time_usec = 0;
    last_baro_time_usec = 0;
}

int MavlinkCommunicator::Init(int portOffset, bool is_copter_airframe){
    is_copter_airframe_ = is_copter_airframe;

    memset((char *) &simulator_mavlink_addr, 0, sizeof(px4_mavlink_addr));
    memset((char *) &px4_mavlink_addr, 0, sizeof(px4_mavlink_addr));
    simulator_mavlink_addr.sin_family = AF_INET;
    simulator_mavlink_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    simulator_mavlink_addr.sin_port = htons(portBase + portOffset);

    if ((listenMavlinkSock = socket(AF_INET, SOCK_STREAM, 0)) < 0){
        std::cerr << "PX4 Communicator: Creating TCP socket failed: " << strerror(errno) << std::endl;
    }

    // do not accumulate messages by waiting for ACK
    int yes = 1;
    int result = setsockopt(listenMavlinkSock, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
    if (result != 0){
        std::cerr << "PX4 Communicator: setsockopt failed: " << strerror(errno) << std::endl;
    }

    // try to close as fast as posible
    struct linger nolinger;
    nolinger.l_onoff = 1;
    nolinger.l_linger = 0;
    result = setsockopt(listenMavlinkSock, SOL_SOCKET, SO_LINGER, &nolinger, sizeof(nolinger));
    if (result != 0){
        std::cerr << "PX4 Communicator: setsockopt failed: " << strerror(errno) << std::endl;
    }

    // The socket reuse is necessary for reconnecting to the same address
    // if the socket does not close but gets stuck in TIME_WAIT. This can happen
    // if the server is suddenly closed, for example, if the robot is deleted in gazebo.
    int socket_reuse = 1;
    result = setsockopt(listenMavlinkSock, SOL_SOCKET, SO_REUSEADDR, &socket_reuse, sizeof(socket_reuse));
    if (result != 0){
         std::cerr << "PX4 Communicator: setsockopt failed: " << strerror(errno) << std::endl;
    }

    // Same as above but for a given port
    result = setsockopt(listenMavlinkSock, SOL_SOCKET, SO_REUSEPORT, &socket_reuse, sizeof(socket_reuse));
    if (result != 0){
        std::cerr << "PX4 Communicator: setsockopt failed: " << strerror(errno) << std::endl;
    }


    if (bind(listenMavlinkSock, (struct sockaddr *)&simulator_mavlink_addr, sizeof(simulator_mavlink_addr)) < 0){
        std::cerr << "PX4 Communicator: bind failed:  " << strerror(errno) << std::endl;
    }

    errno = 0;
    result = listen(listenMavlinkSock, 5);
    if (result < 0){
        std::cerr << "PX4 Communicator: listen failed: " << strerror(errno) << std::endl;
    }

    unsigned int px4_addr_len = sizeof(px4_mavlink_addr);
    while(true) {
        px4MavlinkSock = accept(listenMavlinkSock,
                                (struct sockaddr *)&px4_mavlink_addr,
                                &px4_addr_len);
        if (px4MavlinkSock < 0){
            std::cerr << "PX4 Communicator: accept failed: " << strerror(errno) << std::endl;
        }else{
            std::cerr << "PX4 Communicator: PX4 Connected."<< std::endl;
            break;
        }
    }

    return result;
}


int MavlinkCommunicator::Clean(){
    close(px4MavlinkSock);
    close(listenMavlinkSock);
    return 0;
}

int MavlinkCommunicator::SendHilSensor(unsigned int time_usec,
                                   Eigen::Vector3d pose_geodetic,
                                   Eigen::Quaterniond q_enu_flu,
                                   Eigen::Vector3d vel_frd,
                                   Eigen::Vector3d acc_frd,
                                   Eigen::Vector3d gyro_frd){
    // Output data
    mavlink_hil_sensor_t sensor_msg;
    sensor_msg.time_usec = time_usec;

    // 1. Fill acc and gyro in FRD frame
    sensor_msg.xacc = acc_frd[0];
    sensor_msg.yacc = acc_frd[1];
    sensor_msg.zacc = acc_frd[2];
    sensor_msg.xgyro = gyro_frd[0];
    sensor_msg.ygyro = gyro_frd[1];
    sensor_msg.zgyro = gyro_frd[2];
    sensor_msg.fields_updated = SENS_ACCEL | SENS_GYRO;

    // 2. Fill Magnetc field with noise
    if (time_usec - last_mag_time_usec > mag_period_usec){
        Eigen::Vector3d mag_enu;
        geographiclib_conversions::MagneticField(pose_geodetic.x(), pose_geodetic.y(), pose_geodetic.z(),
                                                mag_enu.x(), mag_enu.y(), mag_enu.z());
        static const auto q_frd_flu = Eigen::Quaterniond(0, 1, 0, 0);
        Eigen::Vector3d mag_frd = q_frd_flu * q_enu_flu.inverse() * mag_enu;
        sensor_msg.xmag = mag_frd[0] + mag_noise * standard_normal_distribution_(random_generator_);
        sensor_msg.ymag = mag_frd[1] + mag_noise * standard_normal_distribution_(random_generator_);
        sensor_msg.zmag = mag_frd[2] + mag_noise * standard_normal_distribution_(random_generator_);
        sensor_msg.fields_updated |= SENS_MAG;
        last_mag_time_usec = time_usec;
    }

    // 3. Fill Barometr and diff pressure
    if (time_usec - last_baro_time_usec > baro_period_usec){
        // 3.1. abs_pressure using an ISA model for the tropsphere (valid up to 11km above MSL)
        const float LAPSE_RATE = 0.0065f;       // reduction in temperature with altitude(Kelvin/m)
        const float TEMPERATURE_MSL = 288.0f;   // temperature at MSL (Kelvin)
        float alt_msl = pose_geodetic.z();
        float temperature_local = TEMPERATURE_MSL - LAPSE_RATE * alt_msl;
        float pressure_ratio = powf((TEMPERATURE_MSL/temperature_local), 5.256f);
        const float PRESSURE_MSL = 101325.0f;
        sensor_msg.abs_pressure = PRESSURE_MSL / pressure_ratio * 0.01f;    // convert to hPa
        sensor_msg.abs_pressure += abs_pressure_noise * standard_normal_distribution_(random_generator_);

        // 3.2. density using an ISA model for the tropsphere (valid up to 11km above MSL)
        const float density_ratio = powf((TEMPERATURE_MSL/temperature_local), 4.256f);
        float rho = 1.225f / density_ratio;

        // 3.3. pressure altitude including effect of pressure noise
        sensor_msg.pressure_alt = pose_geodetic.z();
        sensor_msg.pressure_alt += baro_alt_noise * standard_normal_distribution_(random_generator_);

        // 3.4. temperature in Celsius
        sensor_msg.temperature = temperature_local - 273.0f;
        sensor_msg.temperature += temp_noise * standard_normal_distribution_(random_generator_);

        // 3.5. diff pressure in hPa (Note: ignoring tailsitter case here)
        sensor_msg.diff_pressure = 0.005f * rho * vel_frd.norm() * vel_frd.norm();
        sensor_msg.diff_pressure += diff_pressure_noise * standard_normal_distribution_(random_generator_);

        sensor_msg.fields_updated |= SENS_BARO | SENS_DIFF_PRESS;
        last_baro_time_usec = time_usec;
    }


    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int packetlen;
    mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
    packetlen = mavlink_msg_to_send_buffer(buffer, &msg);

    if(send(px4MavlinkSock, buffer, packetlen, 0) != packetlen){
        return -1;
    }else{
        ROS_INFO_STREAM_THROTTLE(2, "PX4 Communicator: Send \033[1;31m hil_sensor \033[0m" <<
            sensor_msg.temperature  << ", " << sensor_msg.abs_pressure  << ", " <<
            sensor_msg.pressure_alt << ", " << sensor_msg.diff_pressure << ".");
    }
    return 0;
}


int MavlinkCommunicator::SendHilGps(unsigned int time_usec,
                                Eigen::Vector3d vel_ned,
                                Eigen::Vector3d pose_geodetic){
    // Fill gps msg
    mavlink_hil_gps_t hil_gps_msg;
    hil_gps_msg.time_usec = time_usec;
    hil_gps_msg.fix_type = 3;
    hil_gps_msg.lat = pose_geodetic.x() * 1e7;
    hil_gps_msg.lon = pose_geodetic.y() * 1e7;
    hil_gps_msg.alt = pose_geodetic.z() * 1000;
    hil_gps_msg.eph = 100;
    hil_gps_msg.epv = 100;
    hil_gps_msg.vn = vel_ned.x() * 100;
    hil_gps_msg.ve = vel_ned.y() * 100;
    hil_gps_msg.vd = vel_ned.z() * 100;
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
    if(send(px4MavlinkSock, buffer, packetlen, 0) != packetlen){
        return -1;
    }else{
        ROS_INFO_STREAM_THROTTLE(2, "PX4 Communicator: Send \033[1;33m hil_gps \033[0m" << " [" <<
                                 hil_gps_msg.lat << ", " <<
                                 hil_gps_msg.lon << ", " <<
                                 hil_gps_msg.alt << "].");
    }
    return 0;
}

int MavlinkCommunicator::Receive(bool blocking, bool &armed, std::vector<double>& command){
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    struct pollfd fds[1] = {};
    fds[0].fd = px4MavlinkSock;
    fds[0].events = POLLIN;

    int p = poll(&fds[0], 1, (blocking?-1:2));
    if(p < 0){
        std::cerr << "PX4 Communicator: PX4 Pool error\n" << std::endl;
    }else if(p == 0){
        // std::cerr << "PX4 Communicator:No PX data" << std::endl;
    }else if(fds[0].revents & POLLIN){
        unsigned int slen = sizeof(px4_mavlink_addr);
        unsigned int len = recvfrom(px4MavlinkSock,
                                    buffer,
                                    sizeof(buffer),
                                    0,
                                    (struct sockaddr *)&px4_mavlink_addr,
                                    &slen);
        mavlink_status_t status;
        for (unsigned i = 0; i < len; ++i){
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)){
                if(msg.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS){
                    mavlink_hil_actuator_controls_t controls;
                    mavlink_msg_hil_actuator_controls_decode(&msg, &controls);

                    if(command.size() < 4){
                        std::cerr << "command.size() < 4" << std::endl;
                        return -1;
                    }

                    armed = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED);
                    if(armed){
                        command[0] = controls.controls[0];
                        command[1] = controls.controls[1];
                        command[2] = controls.controls[2];
                        command[3] = controls.controls[3];
                        if(is_copter_airframe_ == false){
                            command[4] = controls.controls[4];
                            command[5] = controls.controls[5];
                            command[6] = controls.controls[6];
                            command[7] = controls.controls[7];
                        }

                        ROS_WARN_STREAM_THROTTLE(0.2, "Recv \033[1;29m control->cmd \033[0m [" <<
                                "mc: "      << command[0] << ", " << command[1] << ", " <<
                                               command[2] << ", " << command[3] << ", " <<
                                "fw rpy: (" << command[4] << ", " <<
                                               command[5] << ", " <<
                                               command[6] << "), " <<
                                "throttle " << command[7] << "].");
                    }
                }else if (msg.msgid == MAVLINK_MSG_ID_ESTIMATOR_STATUS){
                    ROS_INFO_STREAM_THROTTLE(2, "MAVLINK_MSG_ID_ESTIMATOR_STATUS");
                }
                return 1;
            }
        }
    }

    return 0;
}
