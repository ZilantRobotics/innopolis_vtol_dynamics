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
 * @file mavlink_communicator.h
 *
 * @author Roman Fedorenko <frontwise@gmail.com>
 * @author ThunderFly s.r.o., Vít Hanousek <info@thunderfly.cz>
 * @url https://github.com/ThunderFly-aerospace
 *
 * PX4 communication socket.
 */

#ifndef PX4_COMMUNICATOR_H
#define PX4_COMMUNICATOR_H

#include <netinet/in.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
#include <geographiclib_conversions/geodetic_conv.hpp>


class MavlinkCommunicator
{
public:
    MavlinkCommunicator(float lat_home);

    /**
     * @brief Init connection with PX4 using TCP
     * @param is_copter_airframe - input
     * Mavlink actuator msg say nothing about is it VTOL or copter and it is always has 8 channels,
     * so we can't understand airframe in real time.
     * The problem is that in copter airframe mavlink fill last 4 channels by random values,
     * so we should initially manually define airframe and just fill last 4 channels by zeros.
     * @note Be sure if your VTOL mixer has shifted values in control surface, for example
     * aileron from 0 to 1, where 0.5 is default - in copter mode communicator fill it by zeros.
     */
    int Init(int portOffset, bool is_copter_airframe);

    /**
     * @brief Close mavlink sockets
     */
    int Clean();

    /**
     * @brief Send hil_sensor (#107) and hil_gps (#113) to PX4 via mavlink
     */
    int SendHilSensor(unsigned int time_usec,
                      Eigen::Vector3d pose_geodetic,
                      Eigen::Quaterniond q_enu_flu,
                      Eigen::Vector3d vel_frd,
                      Eigen::Vector3d acc_frd,
                      Eigen::Vector3d gyro_frd);
    int SendHilGps(unsigned int time_usec,
                   Eigen::Vector3d vel_ned,
                   Eigen::Vector3d pose_geodetic);


    /**
     * @brief Receive hil_actuator_controls (#93) from PX4 via mavlink
     * @param blocking - input
     * @param armed - output
     * @param command - output
     */
    int Receive(bool blocking, bool &armed, std::vector<double>& command);

private:
    static const unsigned int SENS_ACCEL       = 0b111;
    static const unsigned int SENS_GYRO        = 0b111000;
    static const unsigned int SENS_MAG         = 0b111000000;
    static const unsigned int SENS_BARO        = 0b1101000000000;
    static const unsigned int SENS_DIFF_PRESS  = 0b10000000000;

    struct sockaddr_in  px4_mavlink_addr;
    struct sockaddr_in  simulator_mavlink_addr;
    int listenMavlinkSock;
    int px4MavlinkSock;

    const int portBase = 4560;

    std::default_random_engine random_generator_;
    std::normal_distribution<double> standard_normal_distribution_;
    double mag_noise;
    double baro_alt_noise;
    double temp_noise;
    double abs_pressure_noise;
    double diff_pressure_noise;

    unsigned int last_mag_time_usec;
    unsigned int last_baro_time_usec;

    static constexpr uint64_t mag_period_usec = 1e6 / 10;
    static constexpr uint64_t baro_period_usec = 1e6 / 10;

    float ALT_HOME;
    bool is_copter_airframe_;
};


#endif
