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
 * @file px4_communicator.cpp
 *
 * @author Roman Fedorenko <frontwise@gmail.com>
 * @author ThunderFly s.r.o., Vít Hanousek <info@thunderfly.cz>
 * @url https://github.com/ThunderFly-aerospace
 *
 * PX4 communication socket.
 */

#include "px4_communicator.h"
#include <iostream>
#include <ros/ros.h>


PX4Communicator::PX4Communicator(float alt_home) :
    ALT_HOME(alt_home)
{
    standard_normal_distribution_ = std::normal_distribution<double>(0.0f, 0.1f);

    mag_nois = 0.0000051;
    baro_alt_nois = 0.0001;
    temp_nois = 0.001;
    abs_pressure_nois = 0.001;
    // Let's use a rough guess of 0.01 hPa as the standard devitiation which roughly yields
    // about +/- 1 m/s noise.
    diff_pressure_nois = 0.01;

    last_gps_time_usec = -1;
}

int PX4Communicator::Init(int portOffset, MulticopterDynamicsSim *s)
{
    sim = s;

    memset((char *) &simulator_mavlink_addr, 0, sizeof(px4_mavlink_addr));
    memset((char *) &px4_mavlink_addr, 0, sizeof(px4_mavlink_addr));
    simulator_mavlink_addr.sin_family = AF_INET;
    simulator_mavlink_addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    simulator_mavlink_addr.sin_port = htons(portBase+portOffset);

    if ((listenMavlinkSock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        std::cerr<<"PX4 Communicator: Creating TCP socket failed: " << strerror(errno) << std::endl;
    }

    //do not accumulate messages by waiting for ACK
    int yes = 1;
    int result = setsockopt(listenMavlinkSock, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
    if (result != 0)
    {
        std::cerr<<"PX4 Communicator: setsockopt failed: " << strerror(errno) << std::endl;
    }

    //try to close as fast as posible 
    struct linger nolinger;
    nolinger.l_onoff = 1;
    nolinger.l_linger = 0;
    result = setsockopt(listenMavlinkSock, SOL_SOCKET, SO_LINGER, &nolinger, sizeof(nolinger));
    if (result != 0)
    {
        std::cerr<<"PX4 Communicator: setsockopt failed: " << strerror(errno) << std::endl;
    }

    // The socket reuse is necessary for reconnecting to the same address
    // if the socket does not close but gets stuck in TIME_WAIT. This can happen
    // if the server is suddenly closed, for example, if the robot is deleted in gazebo.
    int socket_reuse = 1;
    result = setsockopt(listenMavlinkSock, SOL_SOCKET, SO_REUSEADDR, &socket_reuse, sizeof(socket_reuse));
    if (result != 0) 
    {
         std::cerr<<"PX4 Communicator: setsockopt failed: " << strerror(errno) << std::endl;
    }

    // Same as above but for a given port
    result = setsockopt(listenMavlinkSock, SOL_SOCKET, SO_REUSEPORT, &socket_reuse, sizeof(socket_reuse));
    if (result != 0) 
    {
        std::cerr<<"PX4 Communicator: setsockopt failed: " << strerror(errno) << std::endl;
    }


    if (bind(listenMavlinkSock, (struct sockaddr *)&simulator_mavlink_addr, sizeof(simulator_mavlink_addr)) < 0)
    {
        std::cerr<<"PX4 Communicator: bind failed:  " << strerror(errno) << std::endl;
    }

    errno = 0;
    result=listen(listenMavlinkSock, 5);
    if (result < 0)
    {
        std::cerr<<"PX4 Communicator: listen failed: " << strerror(errno) << std::endl;
    }

    unsigned int px4_addr_len=sizeof(px4_mavlink_addr);
    while(true)
    {
        px4MavlinkSock = accept(listenMavlinkSock, (struct sockaddr *)&px4_mavlink_addr, &px4_addr_len);
        if (px4MavlinkSock<0)
        {
            std::cerr<<"PX4 Communicator: accept failed: " << strerror(errno) << std::endl;
        }
        else
        {
            std::cerr<<"PX4 Communicator: PX4 Connected."<< std::endl;
            break;
        }
    }

    return result;
}

/*void PX4Communicator::CheckClientReconect()
{
    struct pollfd fds[1] = {};
    fds[0].fd = listenMavlinkSock;
    fds[0].events = POLLIN;

    int p=poll(&fds[0], 1, 1);
    if(p<0)
        fprintf(stderr,"Pool for new client error\n");

    if(p==0)
    {
        //fprintf(stderr,"No new Client\n");
    }
    else
    {
        fprintf(stderr,"New Client Connected to Bridge\n");
        close(px4MavlinkSock);
        unsigned int px4_addr_len=sizeof(px4_mavlink_addr);;
        px4MavlinkSock = accept(listenMavlinkSock, (struct sockaddr *)&px4_mavlink_addr, &px4_addr_len);
    }
}*/

// TODO: move to libmavconn from mavros

int PX4Communicator::Clean()
{
    close(px4MavlinkSock);
    close(listenMavlinkSock);
    return 0;
}

int PX4Communicator::SendHilSensor(unsigned int time_usec)
{
    // Input data:
    Eigen::Vector3d pos_enu = sim->getVehiclePosition();
    Eigen::Quaterniond q_enu_flu = sim->getVehicleAttitude();

    // Output data
    mavlink_hil_sensor_t sensor_msg;
    sensor_msg.time_usec = time_usec;

    // Fill acc and gyro (Note: in IMU frame, considered equal to flu)
    Eigen::Vector3d acc_flu(0,0,-9.8);
    Eigen::Vector3d gyro_flu(0,0,0);
    sim->getIMUMeasurement(acc_flu, gyro_flu);
    Eigen::Vector3d acc_frd = q_frd_flu * acc_flu;
    Eigen::Vector3d gyro_frd = q_frd_flu * gyro_flu;
    sensor_msg.xacc = acc_frd[0];
    sensor_msg.yacc = acc_frd[1];
    sensor_msg.zacc = acc_frd[2];
    sensor_msg.xgyro = gyro_frd[0];
    sensor_msg.ygyro = gyro_frd[1];
    sensor_msg.zgyro = gyro_frd[2];
    // TODO: bit 31: full reset of attitude/position/velocities/etc was performed in sim.
    sensor_msg.fields_updated = SENS_ACCEL | SENS_GYRO;

    // Fill Magnetc field
    double latitude_deg = 0;
    double longitude_deg = 0;
    double altitude_m = 0;
    sim->geodetic_converter_.enu2Geodetic(pos_enu.x(), pos_enu.y(), pos_enu.z(),
                                          &latitude_deg, &longitude_deg, &altitude_m);

    double enu_x = 0, enu_y = 0, enu_z = 0;
    geographiclib_conversions::MagneticField(latitude_deg, longitude_deg, altitude_m, enu_x, enu_y, enu_z);

    Eigen::Vector3d mag_enu(enu_x, enu_y, enu_z);
    Eigen::Vector3d mag_frd = q_frd_flu * q_enu_flu.inverse() * mag_enu;
    sensor_msg.xmag = mag_frd[0] + mag_nois * standard_normal_distribution_(random_generator_);
    sensor_msg.ymag = mag_frd[1] + mag_nois * standard_normal_distribution_(random_generator_);
    sensor_msg.zmag = mag_frd[2] + mag_nois * standard_normal_distribution_(random_generator_);
    if(last_mag_time_usec < 0)
        last_mag_time_usec = 0;
    if (time_usec - last_mag_time_usec > 1e6/10)
    {
        sensor_msg.fields_updated |= SENS_MAG;
        last_mag_time_usec = time_usec;
    }

    // calculate abs_pressure using an ISA model for the tropsphere (valid up to 11km above MSL)
    const float LAPSE_RATE = 0.0065f; // reduction in temperature with altitude (Kelvin/m)
    const float TEMPERATURE_MSL = 288.0f; // temperature at MSL (Kelvin)
    float alt_msl = ALT_HOME + pos_enu.z();

    float temperature_local = TEMPERATURE_MSL - LAPSE_RATE * alt_msl;
    float pressure_ratio = powf((TEMPERATURE_MSL/temperature_local), 5.256f);
    const float PRESSURE_MSL = 101325.0f;
    float absolute_pressure = PRESSURE_MSL / pressure_ratio;

    // convert to hPa
    absolute_pressure *= 0.01f;

    // calculate density using an ISA model for the tropsphere (valid up to 11km above MSL)
    const float density_ratio = powf((TEMPERATURE_MSL/temperature_local), 4.256f);
    float rho = 1.225f / density_ratio;

    // calculate pressure altitude including effect of pressure noise
    double pressure_altitude = alt_msl;

    // calculate temperature in Celsius
    double temperature = temperature_local - 273.0f;

    // diff pressure
    Eigen::Vector3d vel_enu = sim->getVehicleVelocity();
    Eigen::Vector3d vel_frd = q_frd_flu * q_enu_flu.inverse() * vel_enu;
    // calculate differential pressure in hPa
    // Note: ignoring tailsitter case here
    double diff_pressure = 0.005f * rho * vel_frd.x() * vel_frd.x();// + diff_pressure_noise;

    sensor_msg.temperature = temperature + temp_nois * standard_normal_distribution_(random_generator_);
    sensor_msg.abs_pressure = absolute_pressure + abs_pressure_nois * standard_normal_distribution_(random_generator_);
    sensor_msg.pressure_alt = pressure_altitude + baro_alt_nois * standard_normal_distribution_(random_generator_);
    sensor_msg.diff_pressure = diff_pressure + diff_pressure_nois * standard_normal_distribution_(random_generator_) ;

    if(last_baro_time_usec < 0)
        last_baro_time_usec = 0;
    if (time_usec - last_baro_time_usec > 1e6/10)
    {
        sensor_msg.fields_updated |= SENS_BARO | SENS_DIFF_PRESS;
        last_baro_time_usec = time_usec;
    }


    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int packetlen;
    mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
    packetlen = mavlink_msg_to_send_buffer(buffer, &msg);

    if(send(px4MavlinkSock, buffer, packetlen, 0) != packetlen)
    {
        return -1;
    }else{
        ROS_INFO_STREAM_THROTTLE(2, "PX4 Communicator: Send \033[1;31m hil_sensor \033[0m" <<
            sensor_msg.temperature  << ", " << sensor_msg.abs_pressure  << ", " <<
            sensor_msg.pressure_alt << ", " << sensor_msg.diff_pressure << ".");
    }
    return 0;
}

int PX4Communicator::SendHilGps(unsigned int time_usec){
    Eigen::Vector3d vel_enu = sim->getVehicleVelocity();
    Eigen::Vector3d vel_ned = q_ned_enu * vel_enu;
    double speed_north_mps = vel_ned.x();
    double speed_east_mps = vel_ned.y();
    double speed_down_mps = vel_ned.z();

    Eigen::Vector3d pos_enu = sim->getVehiclePosition();
    double latitude_deg = 0;
    double longitude_deg = 0;
    double altitude_m = 0;
    double east = pos_enu.x();
    double north = pos_enu.y();
    double up = pos_enu.z();
    sim->geodetic_converter_.enu2Geodetic(east, north, up,
                                          &latitude_deg, &longitude_deg, &altitude_m);

    mavlink_hil_gps_t hil_gps_msg;
    hil_gps_msg.time_usec = time_usec;// fgData.elapsed_sec * 1e6;
    hil_gps_msg.fix_type = 3;
    hil_gps_msg.lat = latitude_deg * 1e7;
    hil_gps_msg.lon = longitude_deg * 1e7;
    hil_gps_msg.alt = altitude_m * 1000;
    hil_gps_msg.eph = 100;
    hil_gps_msg.epv = 100;
    hil_gps_msg.vn = speed_north_mps * 100;
    hil_gps_msg.ve = speed_east_mps * 100;
    hil_gps_msg.vd = speed_down_mps * 100;
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
    if(send(px4MavlinkSock, buffer, packetlen, 0) != packetlen)
    {
        return -1;
    }else
    {
        ROS_INFO_STREAM_THROTTLE(2, "PX4 Communicator: Send \033[1;33m hil_gps \033[0m" << " [" <<
                                 hil_gps_msg.lat << ", " << hil_gps_msg.lon << ", " << hil_gps_msg.alt << "].");
    }
    return 0;
}

int PX4Communicator::Receive(bool blocking, bool &armed, std::vector<double>& command)
{
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    struct pollfd fds[1] = {};
    fds[0].fd = px4MavlinkSock;
    fds[0].events = POLLIN;

    int p = poll(&fds[0], 1, (blocking?-1:2));
    if(p < 0){
        std::cerr << "PX4 Communicator: PX4 Pool error\n" << std::endl;
    }
    if(p == 0){
        // std::cerr << "PX4 Communicator:No PX data" << std::endl;
    }
    else if(fds[0].revents & POLLIN){
        unsigned int slen=sizeof(px4_mavlink_addr);
        unsigned int len = recvfrom(px4MavlinkSock, buffer, sizeof(buffer), 0, (struct sockaddr *)&px4_mavlink_addr, &slen);
        mavlink_status_t status;
        for (unsigned i = 0; i < len; ++i)
        {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status) &&
                msg.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS)
            {
                mavlink_hil_actuator_controls_t controls;
                mavlink_msg_hil_actuator_controls_decode(&msg, &controls);

                if(command.size() < 4)
                {
                    std::cerr << "command.size() < 4" << std::endl;
                    return -1;
                }

                armed = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED);
                if(armed)
                {
                    command[3] = controls.controls[0];  // PX4: motor 1, front right
                    command[1] = controls.controls[1];  // PX4: motor 2, tail left
                    command[0] = controls.controls[2];  // PX4: motor 3, front left
                    command[2] = controls.controls[3];  // PX4: motor 4, tail right
                    ROS_WARN_STREAM_THROTTLE(0.2, "PX4 Communicator: Recv \033[1;29m control \033[0m" << " [" <<
                            controls.controls[0] << ", " << controls.controls[1] << ", " <<
                            controls.controls[2] << ", " << controls.controls[3] << "].");
                }
                return 1;
            }
        }
    }

    return 0;
}
