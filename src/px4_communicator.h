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
 * @file px4_communicator.h
 *
 * @author Roman Fedorenko <frontwise@gmail.com>
 * @author ThunderFly s.r.o., Vít Hanousek <info@thunderfly.cz>
 * @url https://github.com/ThunderFly-aerospace
 *
 * PX4 communication socket.
 */

#ifndef PX4_COMMUNICATOR_H
#define PX4_COMMUNICATOR_H


#include <stdio.h>
#include <mavlink/v2.0/common/mavlink.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>

#include "../libs/multicopterDynamicsSim/multicopterDynamicsSim.hpp"
#include "uavDynamicsSimBase.hpp"

#define TIMEOUTS 5
#define TIMEOUTUS 0


const unsigned int SENS_ACCEL       = 0b111;
const unsigned int SENS_GYRO        = 0b111000;
const unsigned int SENS_MAG         = 0b111000000;
const unsigned int SENS_BARO        = 0b1101000000000;
const unsigned int SENS_DIFF_PRESS  = 0b10000000000;

/*
 * @brief Quaternion for rotation between ENU and NED frames
 *
 * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
 * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
 */
static const auto q_ned_enu = Eigen::Quaterniond(0, 0.70711, 0.70711, 0);       // q_ng
// NED_px4 = q_ng * ENU_ros

/**
 * @brief Quaternion for rotation between body FLU and body FRD frames
 *
 * +PI rotation around X (Forward) axis rotates from Forward, Right, Down (aircraft)
 * to Forward, Left, Up (base_link) frames and vice-versa.
 */
static const auto q_frd_flu = Eigen::Quaterniond(0, 1, 0, 0);       // q_br
// FRD_px4(b) = q_br * FLU_ros(r)


class PX4Communicator
{

private:
    UavDynamicsSimBase *sim;

    struct sockaddr_in  px4_mavlink_addr;
    struct sockaddr_in  simulator_mavlink_addr;
    int listenMavlinkSock;
    int px4MavlinkSock;

    const int portBase = 4560;

    std::default_random_engine random_generator_;
    std::normal_distribution<double> standard_normal_distribution_;
    double mag_nois;
    double baro_alt_nois;
    double temp_nois;
    double abs_pressure_nois;
    double diff_pressure_nois;

    unsigned int last_gps_time_usec;
    unsigned int last_mag_time_usec;
    unsigned int last_baro_time_usec;

    float ALT_HOME;
    bool is_copter_airframe_;
public:
    PX4Communicator(float lat_home);

    /**
     * @brief Init connection with PX4 using TCP
     * @param is_copter_airframe - input (copter requires only 4 control channels and vtol requires
     * 8 channels, but for mavlink there is no difference, so it can fill last controls channels
     * by random values in copter airframe, so we should not read them in this case to allow to run
     * any dynamics simulators in any airframe)
     */
    int Init(int portOffset, UavDynamicsSimBase *s, bool is_copter_airframe);

    int Clean();

    /**
     * @brief Send hil_sensor (#107) and hil_gps (#113) to PX4 via mavlink
     */
    int SendHilSensor(unsigned int time_usec);
    int SendHilGps(unsigned int time_usec);


    /**
     * @brief Receive hil_actuator_controls (#93) from PX4 via mavlink
     * @param blocking - input
     * @param armed - output
     * @param command - output
     */
    int Receive(bool blocking, bool &armed, std::vector<double>& command);
};


#endif
