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


PX4Communicator::PX4Communicator()
{    
	standard_normal_distribution_ = std::normal_distribution<double>(0.0f, 1.0f);

	// acc_nois = 0.00001;
	// gyro_nois = 0.0001;
	mag_nois = 0.0000051;
	// baro_alt_nois = 0.001;
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

    // sleep(5);

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


int PX4Communicator::Send(unsigned int time_usec)
{

    double temperature = 17.3;

    Eigen::Vector3d pos_enu = sim->getVehiclePosition();
    Eigen::Quaterniond q_enu_flu = sim->getVehicleAttitude();

    // acc
    Eigen::Vector3d acc_flu(0,0,-9.8);
    Eigen::Vector3d gyro_flu(0,0,0);
    // Note: in IMU frame, considered equal to flu
    sim->getIMUMeasurement(acc_flu, gyro_flu);
    // std::cout << "acc_flu " << acc_flu.transpose() << std::endl;
    // std::cout << "gyro_flu " << gyro_flu.transpose() << std::endl;
    Eigen::Vector3d acc_frd = q_frd_flu * acc_flu;

    Eigen::Vector3d gyro_frd = q_frd_flu * gyro_flu;
    // std::cout << "acc_frd " << acc_frd.transpose()  << std::endl;
    // std::cout << "gyro_frd " << gyro_frd.transpose()  << std::endl;


    // llh
    double latitude_deg = 0;
	double longitude_deg = 0;
	double altitude_m = 0;
    double east = pos_enu.x();
    double north = pos_enu.y();
    double up = pos_enu.z();
    sim->geodetic_converter_.enu2Geodetic(east, north, up, 
                                         &latitude_deg, &longitude_deg, &altitude_m);

    // vel
    Eigen::Vector3d vel_enu = sim->getVehicleVelocity();
    Eigen::Vector3d vel_ned = q_ned_enu * vel_enu;

	double speed_north_mps = vel_ned.x();
	double speed_east_mps = vel_ned.y();
	double speed_down_mps = vel_ned.z();

    // Magnetc field
    double Bx = 0, By = 0, Bz = 0;  // ENU
    geographiclib_conversions::MagneticField(latitude_deg, longitude_deg, altitude_m, Bx, By, Bz);
    Eigen::Vector3d mag_enu(Bx, By, Bz);  // NED
    Eigen::Vector3d mag_frd = q_frd_flu * q_enu_flu.inverse() * mag_enu;
    // std::cout << "mag: " << mag_frd.transpose() << std::endl;

    // pressure
    float alt_home = 0;     // TODO: move to params
    float pose_n_z = -pos_enu.z(); // convert Z-component from ENU to NED

    // calculate abs_pressure using an ISA model for the tropsphere (valid up to 11km above MSL)
    const float lapse_rate = 0.0065f; // reduction in temperature with altitude (Kelvin/m)
    const float temperature_msl = 288.0f; // temperature at MSL (Kelvin)
    float alt_msl = alt_home - pose_n_z;
    float temperature_local = temperature_msl - lapse_rate * alt_msl;
    float pressure_ratio = powf((temperature_msl/temperature_local), 5.256f);
    const float pressure_msl = 101325.0f; // pressure at MSL
    float absolute_pressure = pressure_msl / pressure_ratio;

    // convert to hPa
    absolute_pressure *= 0.01f;

    // calculate density using an ISA model for the tropsphere (valid up to 11km above MSL)
    const float density_ratio = powf((temperature_msl/temperature_local), 4.256f);
    float rho = 1.225f / density_ratio;

    // calculate pressure altitude including effect of pressure noise
    double pressure_altitude = alt_msl;

    // calculate temperature in Celsius
    temperature = temperature_local - 273.0f;

    // diff pressure
   	double diff_pressure = 0;
    Eigen::Vector3d vel_frd = q_frd_flu * q_enu_flu.inverse() * vel_enu;
    // calculate differential pressure in hPa
    // Note: ignoring tailsitter case here
    diff_pressure = 0.005f * rho * vel_frd.x() * vel_frd.x();// + diff_pressure_noise;
    


    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int packetlen;

    mavlink_hil_sensor_t sensor_msg;

	sensor_msg.time_usec = time_usec;

	sensor_msg.xacc = acc_frd[0];// + acc_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.yacc = acc_frd[1];// + acc_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.zacc = acc_frd[2];// + acc_nois * standard_normal_distribution_(random_generator_);

	sensor_msg.xgyro = gyro_frd[0];// + gyro_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.ygyro = gyro_frd[1];// + gyro_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.zgyro = gyro_frd[2];// + gyro_nois * standard_normal_distribution_(random_generator_);

	sensor_msg.xmag = mag_frd[0] + mag_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.ymag = mag_frd[1] + mag_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.zmag = mag_frd[2] + mag_nois * standard_normal_distribution_(random_generator_);

	sensor_msg.temperature = temperature + temp_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.abs_pressure = absolute_pressure + abs_pressure_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.pressure_alt = pressure_altitude + baro_alt_nois * standard_normal_distribution_(random_generator_);
	sensor_msg.diff_pressure = diff_pressure + diff_pressure_nois * standard_normal_distribution_(random_generator_) ;

    // TODO: bit 31: full reset of attitude/position/velocities/etc was performed in sim.
    sensor_msg.fields_updated = SENS_ACCEL | SENS_GYRO;

	if(last_mag_time_usec < 0)
        last_mag_time_usec = 0;
	if(last_baro_time_usec < 0)
        last_baro_time_usec = 0;

    // magnetometer pubRate 100
    // barometer pubRate 50
    if (time_usec - last_mag_time_usec > 10e6/100)
    {
        sensor_msg.fields_updated = sensor_msg.fields_updated | SENS_MAG;
        last_mag_time_usec = time_usec;
    }

    if (time_usec - last_baro_time_usec > 10e6/100)
    {
        sensor_msg.fields_updated = sensor_msg.fields_updated | SENS_BARO | SENS_DIFF_PRESS;
        last_baro_time_usec = time_usec;
    }

    ROS_INFO_STREAM_THROTTLE(0.2, "\033[1;33m acc " << sensor_msg.xacc << " " << sensor_msg.yacc << " " << sensor_msg.zacc <<
                                                       " gyro " << sensor_msg.xgyro << " " << sensor_msg.ygyro << " " << sensor_msg.zgyro <<
                                                       " mag " << sensor_msg.xmag << " " << sensor_msg.ymag << " " << sensor_msg.zmag << 
                                                        "\033[0m");


    mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
    packetlen = mavlink_msg_to_send_buffer(buffer, &msg);
    if(send(px4MavlinkSock, buffer, packetlen, 0)!=packetlen)
    {
        std::cerr << "PX4 Communicator: Sent to PX4 failed: "<< strerror(errno) <<std::endl;
        return -1;
    }
    if(last_gps_time_usec < 0)
        last_gps_time_usec = time_usec;

    if( time_usec - last_gps_time_usec > 10e5/5)
    {
        std::cout << "GPS" << std::endl;
        last_gps_time_usec = time_usec;

        mavlink_hil_gps_t hil_gps_msg;
        hil_gps_msg.time_usec = time_usec;// fgData.elapsed_sec * 1e6;
        hil_gps_msg.fix_type = 3;
        hil_gps_msg.lat = latitude_deg * 1e7;
        hil_gps_msg.lon = longitude_deg * 1e7;
        hil_gps_msg.alt = altitude_m * 1000;
        hil_gps_msg.eph =  100;
        hil_gps_msg.epv = 100;
        hil_gps_msg.vn = speed_north_mps * 100;
        hil_gps_msg.ve = speed_east_mps * 100;
        hil_gps_msg.vd = speed_down_mps * 100;
        hil_gps_msg.vel = std::sqrt(hil_gps_msg.vn * hil_gps_msg.vn + hil_gps_msg.ve * hil_gps_msg.ve);
        double cog = -std::atan2(hil_gps_msg.vn, hil_gps_msg.ve) * 180 / 3.141592654 + 90;

        if (cog < 0) {
            cog += 360;
        }

        hil_gps_msg.cog = cog * 100;        // Course over ground
        hil_gps_msg.satellites_visible = 10;


        mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_gps_msg);
        packetlen = mavlink_msg_to_send_buffer(buffer, &msg);
        if(send(px4MavlinkSock, buffer, packetlen, 0)!=packetlen)
        {
            std::cerr << "PX4 Communicator: Sent to PX4 failed: " << strerror(errno) <<std::endl;
            return -1;
        }
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

        int p=poll(&fds[0], 1, (blocking?-1:2));
        if(p<0)
            std::cerr<<"PX4 Communicator: PX4 Pool error\n" << std::endl;

        if(p==0)
        {
            // std::cerr<<"PX4 Communicator:No PX data" <<std::endl;
        }
        else
        {
            if(fds[0].revents & POLLIN)
            {
                unsigned int slen=sizeof(px4_mavlink_addr);
                unsigned int len = recvfrom(px4MavlinkSock, buffer, sizeof(buffer), 0, (struct sockaddr *)&px4_mavlink_addr, &slen);
                if (len > 0)
                {
                    mavlink_status_t status;
                    for (unsigned i = 0; i < len; ++i)
                    {
                      if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
                      {
                            if(msg.msgid==MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS)
                            {
                                    mavlink_hil_actuator_controls_t controls;
                                    mavlink_msg_hil_actuator_controls_decode(&msg, &controls);
                                    // vehicle->setPXControls(controls);
                                    // std::cout << "Received controls: [";
                                    // for (size_t i = 0; i < 9; i++)
                                    //     std::cout << controls.controls[i] << " ";
                                    // std::cout << "]" << std::endl;

                                    ROS_INFO_STREAM_THROTTLE(0.2, "Controls from PX4 \033[1;31m" << controls.controls[0] << " "
                                                                                      << controls.controls[1] << " "
                                                                                      << controls.controls[2] << " "
                                                                                      << controls.controls[3] << " "
                                                                                      << controls.controls[4] << " "
                                                                                      << controls.controls[5] << " "
                                                                                      << controls.controls[6] << " "
                                                                                      << controls.controls[7] << " "
                                                                                      << controls.controls[8] << "\033[0m");
                                     for (size_t i = 0; i < 4; i++)
                                        if(controls.controls[i] < 0)
                                            controls.controls[i] = 0;
                                    
                                    // sim->setMotorPercent(controls.controls[0], 3);  // PX4: motor 1, front right
                                    // sim->setMotorPercent(controls.controls[1], 1);  // PX4: motor 2, tail left
                                    // sim->setMotorPercent(controls.controls[2], 0);  // PX4: motor 3, front left
                                    // sim->setMotorPercent(controls.controls[3], 2);  // PX4: motor 4, tail right
                                    
                                    if(command.size() < 4)
                                    {
                                        std::cerr << "command.size() < 4" << std::endl;
                                        return -1;
                                    }

                                    armed = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED);
                                    if(armed)
                                    {
                                        command[3] = controls.controls[0];      // PX4: motor 1, front right
                                        command[1] = controls.controls[1];      // PX4: motor 2, tail left
                                        command[0] = controls.controls[2];      // PX4: motor 3, front left
                                        command[2] = controls.controls[3];      // PX4: motor 4, tail right
                                    }
                                    
                                    return 1;
                            }
                      }
                    }
                }
            }
        }

    return 0;
}

