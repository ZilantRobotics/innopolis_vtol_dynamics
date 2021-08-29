/**
 * @file vtolDynamicsSim.cpp
 * @author ponomarevda96@gmail.com
 * @brief Vtol dynamics simulator class implementation
 */
#include <iostream>
#include <cmath>
#include <boost/algorithm/clamp.hpp>
#include <algorithm>
#include "vtolDynamicsSim.hpp"
#include <ros/package.h>
#include <array>
#include "cs_converter.hpp"

#define STORE_SIM_PARAMETERS    true
#define FORCES_LOG              false
#define MOMENTS_LOG             false
#define AERODYNAMICS_LOG        false

InnoVtolDynamicsSim::InnoVtolDynamicsSim(): distribution_(0.0, 1.0){
    state_.angularVel.setZero();
    state_.linearVel.setZero();
    state_.windVelocity.setZero();
    state_.windVariance = 0;
    state_.accelBias.setZero();
    state_.gyroBias.setZero();
    state_.Fspecific << 0, 0, -params_.gravity;
    for(size_t idx = 0; idx < 8; idx++){
        state_.prevActuators.push_back(0);
        state_.crntActuators.push_back(0);
    }
}

int8_t InnoVtolDynamicsSim::init(){
    loadTables("/uav/aerodynamics_coeffs/");
    loadParams("/uav/vtol_params/");
    return 0;
}

template<int ROWS, int COLS, int ORDER>
Eigen::MatrixXd getTableNew(const std::string& path, const char* name){
    std::vector<double> data;

    if(ros::param::get(path + name, data) == false){
        throw std::runtime_error(std::string("Wrong parameter name: ") + name);
    }

    return Eigen::Matrix<double, ROWS, COLS, ORDER>(data.data());
}


void InnoVtolDynamicsSim::loadTables(const std::string& path){
    tables_.CS_rudder = getTableNew<8, 20, Eigen::RowMajor>(path, "CS_rudder_table");
    tables_.CS_beta = getTableNew<8, 90, Eigen::RowMajor>(path, "CS_beta");
    tables_.AoA = getTableNew<1, 47, Eigen::RowMajor>(path, "AoA");
    tables_.AoS = getTableNew<90, 1, Eigen::ColMajor>(path, "AoS");
    tables_.actuator = getTableNew<20, 1, Eigen::ColMajor>(path, "actuator_table");
    tables_.airspeed = getTableNew<8, 1, Eigen::ColMajor>(path, "airspeed_table");
    tables_.CLPolynomial = getTableNew<8, 8, Eigen::RowMajor>(path, "CLPolynomial");
    tables_.CSPolynomial = getTableNew<8, 8, Eigen::RowMajor>(path, "CSPolynomial");
    tables_.CDPolynomial = getTableNew<8, 6, Eigen::RowMajor>(path, "CDPolynomial");
    tables_.CmxPolynomial = getTableNew<8, 8, Eigen::RowMajor>(path, "CmxPolynomial");
    tables_.CmyPolynomial = getTableNew<8, 8, Eigen::RowMajor>(path, "CmyPolynomial");
    tables_.CmzPolynomial = getTableNew<8, 8, Eigen::RowMajor>(path, "CmzPolynomial");
    tables_.CmxAileron = getTableNew<8, 20, Eigen::RowMajor>(path, "CmxAileron");
    tables_.CmyElevator = getTableNew<8, 20, Eigen::RowMajor>(path, "CmyElevator");
    tables_.CmzRudder = getTableNew<8, 20, Eigen::RowMajor>(path, "CmzRudder");
    tables_.prop = getTableNew<40, 5, Eigen::RowMajor>(path, "prop");
    if(ros::param::get(path + "actuatorTimeConstants", tables_.actuatorTimeConstants) == false){
        throw std::runtime_error(std::string("Wrong parameter name: ") + "actuatorTimeConstants");
    }
}

void InnoVtolDynamicsSim::loadParams(const std::string& path){
    double propLocX, propLocY, propLocZ;

    if(!ros::param::get(path + "mass", params_.mass) ||
        !ros::param::get(path + "gravity", params_.gravity) ||
        !ros::param::get(path + "atmoRho", params_.atmoRho) ||
        !ros::param::get(path + "wingArea", params_.wingArea) ||
        !ros::param::get(path + "characteristicLength", params_.characteristicLength) ||
        !ros::param::get(path + "propellersLocationX", propLocX) ||
        !ros::param::get(path + "propellersLocationY", propLocY) ||
        !ros::param::get(path + "propellersLocationZ", propLocZ) ||
        !ros::param::get(path + "actuatorMin", params_.actuatorMin) ||
        !ros::param::get(path + "actuatorMax", params_.actuatorMax) ||
        !ros::param::get(path + "accVariance", params_.accVariance) ||
        !ros::param::get(path + "gyroVariance", params_.gyroVariance));

    params_.propellersLocation[0] << propLocX * sin(3.1415/4),  propLocY * sin(3.1415/4), propLocZ;
    params_.propellersLocation[1] <<-propLocX * sin(3.1415/4), -propLocY * sin(3.1415/4), propLocZ;
    params_.propellersLocation[2] << propLocX * sin(3.1415/4), -propLocY * sin(3.1415/4), propLocZ;
    params_.propellersLocation[3] <<-propLocX * sin(3.1415/4),  propLocY * sin(3.1415/4), propLocZ;
    params_.propellersLocation[4] << propLocX, 0, 0;
    params_.inertia = getTableNew<3, 3, Eigen::RowMajor>(path, "inertia");
}

void InnoVtolDynamicsSim::setInitialPosition(const Eigen::Vector3d & position,
                                             const Eigen::Quaterniond& attitude){
    state_.position = position;
    state_.attitude = attitude;
    state_.initialPose = position;
    state_.initialAttitude = attitude;
}
void InnoVtolDynamicsSim::setInitialVelocity(const Eigen::Vector3d & linearVelocity,
                                         const Eigen::Vector3d& angularVelocity){
    state_.linearVel = linearVelocity;
    state_.angularVel = angularVelocity;
}

void InnoVtolDynamicsSim::land(){
    state_.Fspecific << 0, 0, -params_.gravity;
    state_.linearVel.setZero();
    state_.position[2] = 0.00;

    #define YAW_ROTATE_ON_LAND_DEBUG false
    #if YAW_ROTATE_ON_LAND_DEBUG == false
    state_.attitude = state_.initialAttitude;
    state_.angularVel.setZero();
    #elif YAW_ROTATE_ON_LAND_DEBUG == true
    state_.angularVel << 0.000, 0.000, 2*3.1415/60;
    Eigen::Quaterniond attitudeDelta = state_.attitude * Eigen::Quaterniond(0, state_.angularVel(0), state_.angularVel(1), state_.angularVel(2));
    state_.attitude.coeffs() += attitudeDelta.coeffs() * 0.5 * 0.001;
    state_.attitude.normalize();
    #endif
}

int8_t InnoVtolDynamicsSim::calibrate(CalibrationType_t calType){
    constexpr float MAG_ROTATION_SPEED = 2 * 3.1415 / 20;
    static uint8_t prevCalibrationType = 0;
    state_.linearVel.setZero();
    state_.position[2] = 0.00;

    if(calType == WORK_MODE){
        state_.attitude = Eigen::Quaterniond(1, 0, 0, 0);
        state_.angularVel.setZero();
    }else if(calType == MAG_1_NORMAL){
        if(prevCalibrationType != calType){
            state_.attitude = Eigen::Quaterniond(1, 0, 0, 0);
        }
        state_.angularVel << 0.000, 0.000, -MAG_ROTATION_SPEED;
    }else if(calType == MAG_2_OVERTURNED){
        if(prevCalibrationType != calType){
            state_.attitude = Eigen::Quaterniond(0, 1, 0, 0);
        }
        state_.angularVel << 0.000, 0.000, MAG_ROTATION_SPEED;
    }else if(calType == MAG_3_HEAD_DOWN){
        if(prevCalibrationType != calType){
            state_.attitude = Eigen::Quaterniond(0.707, 0, -0.707, 0);
        }
        state_.angularVel << -MAG_ROTATION_SPEED, 0.000, 0.000;
    }else if(calType == MAG_4_HEAD_UP){
        if(prevCalibrationType != calType){
            state_.attitude = Eigen::Quaterniond(0.707, 0, 0.707, 0);
        }
        state_.angularVel << MAG_ROTATION_SPEED, 0.000, 0.000;
    }else if(calType == MAG_5_TURNED_LEFT){
        if(prevCalibrationType != calType){
            state_.attitude = Eigen::Quaterniond(0.707, -0.707, 0, 0);
        }
        state_.angularVel << 0.000, MAG_ROTATION_SPEED, 0.000;
    }else if(calType == MAG_6_TURNED_RIGHT){
        if(prevCalibrationType != calType){
            state_.attitude = Eigen::Quaterniond(0.707, 0.707, 0, 0);
        }
        state_.angularVel << 0.000, -MAG_ROTATION_SPEED, 0.000;
    }else if(calType == ACC_1_NORMAL){
        if(prevCalibrationType != calType){
            state_.attitude = Eigen::Quaterniond(1, 0, 0, 0);
        }
        state_.angularVel.setZero();
    }else if(calType == ACC_2_OVERTURNED){
        if(prevCalibrationType != calType){
            state_.attitude = Eigen::Quaterniond(0, 1, 0, 0);
        }
        state_.angularVel.setZero();
    }else if(calType == ACC_3_HEAD_DOWN){
        if(prevCalibrationType != calType){
            state_.attitude = Eigen::Quaterniond(0.707, 0, -0.707, 0);
        }
        state_.angularVel.setZero();
    }else if(calType == ACC_4_HEAD_UP){
        if(prevCalibrationType != calType){
            state_.attitude = Eigen::Quaterniond(0.707, 0, 0.707, 0);
        }
        state_.angularVel.setZero();
    }else if(calType == ACC_5_TURNED_LEFT){
        if(prevCalibrationType != calType){
            state_.attitude = Eigen::Quaterniond(0.707, -0.707, 0, 0);
        }
        state_.angularVel.setZero();
    }else if(calType == ACC_6_TURNED_RIGHT){
        if(prevCalibrationType != calType){
            state_.attitude = Eigen::Quaterniond(0.707, 0.707, 0, 0);
        }
        state_.angularVel.setZero();
    }else if(calType == 21){ // airspeed
        state_.attitude = Eigen::Quaterniond(1, 0, 0, 0);
        state_.angularVel.setZero();
        state_.linearVel[0] = 10.0;
        state_.linearVel[1] = 10.0;
    }

    if(prevCalibrationType != calType){
        ROS_WARN_STREAM_THROTTLE(1, "init cal " << calType + 0);
        prevCalibrationType = calType;
    }else{
        ROS_WARN_STREAM_THROTTLE(1, "cal " << calType + 0);
    }

    constexpr float DELTA_TIME = 0.001;

    state_.Fspecific = calculateNormalForceWithoutMass();
    Eigen::Quaterniond attitudeDelta = state_.attitude * Eigen::Quaterniond(0, state_.angularVel(0), state_.angularVel(1), state_.angularVel(2));
    state_.attitude.coeffs() += attitudeDelta.coeffs() * 0.5 * DELTA_TIME;
    state_.attitude.normalize();
    return 1;
}

void InnoVtolDynamicsSim::process(double dtSecs,
                              const std::vector<double>& motorCmd,
                              bool isCmdPercent){
    Eigen::Vector3d vel_w = calculateWind();
    Eigen::Matrix3d rotationMatrix = calculateRotationMatrix();
    Eigen::Vector3d airSpeed = calculateAirSpeed(rotationMatrix, state_.linearVel, vel_w);
    double AoA = calculateAnglesOfAtack(airSpeed);
    double AoS = calculateAnglesOfSideslip(airSpeed);
    auto actuators = isCmdPercent ? mapCmdToActuatorInnoVTOL(motorCmd) : motorCmd;
    updateActuators(actuators, dtSecs);
    calculateAerodynamics(airSpeed, AoA, AoS, actuators[5], actuators[6], actuators[7],
                          state_.Faero, state_.Maero);
    calculateNewState(state_.Maero, state_.Faero, actuators, dtSecs);
}


/**
 * @note Map motors indexes from StandardVTOL mixer into internal represenation
 * Input indexes should correspond following mixer:
 * https://github.com/InnopolisAero/Inno_PX4_Firmware/blob/e28f8a7f2e181680353cd23ed5c62c4e9b5858fc/ROMFS/px4fmu_common/mixers-sitl/standard_vtol_sitl.main.mix
 * Output indexes will be:
 * 0-3 - copter indexes, where 0 - right forward, 1 - left backward, 2 - left forward, 3 - right backward
 * 4 - throttle
 * 5 - aileron
 * 6 - elevator
 * 7 - rudder (always equal to zero, because there is no control for it)
 */
std::vector<double> InnoVtolDynamicsSim::mapCmdToActuatorStandardVTOL(const std::vector<double>& cmd) const{
    if(cmd.size() != 8){
        std::cerr << "ERROR: InnoVtolDynamicsSim wrong control size. It is " << cmd.size()
                  << ", but should be 8" << std::endl;
        return cmd;
    }

    std::vector<double> actuators(8);
    actuators[0] = cmd[0];
    actuators[1] = cmd[1];
    actuators[2] = cmd[2];
    actuators[3] = cmd[3];
    actuators[4] = cmd[4];
    actuators[5] = (cmd[5] - cmd[6]) / 2;   // aileron      roll
    actuators[6] = -cmd[7];                 // elevator     pitch
    actuators[7] = 0.0;                     // rudder       yaw

    for(size_t idx = 0; idx < 5; idx++){
        actuators[idx] = boost::algorithm::clamp(actuators[idx], 0.0, +1.0);
        actuators[idx] *= params_.actuatorMax[idx];
    }

    for(size_t idx = 5; idx < 8; idx++){
        actuators[idx] = boost::algorithm::clamp(actuators[idx], -1.0, +1.0);
        actuators[idx] *= (actuators[idx] >= 0) ? params_.actuatorMax[idx] : -params_.actuatorMin[idx];
    }

    return actuators;
}

/**
 * @note Map motors indexes from InnoVTOL mixer into internal represenation
 * @param cmd Input indexes should correspond InnoVTOL PX4 mixer
 * Few notes:
 * 4 - aileron default value is 0.5 and it can be [0, +1], where 0 wants to rotate to the right
 * 5 - elevator default value is 0 and it can be [-1, +1], where -1 wants ...
 * 6 - rudder default value is 0 and it can be [-1, +1], where -1 wants ...
 * 7 - throttle default value is 0 and it can be [0, +1]
 * @return Output indexes will be:
 * 0 - 3 are the same copter indexes
 * 4 - throttle
 * 5 - aileron
 * 6 - elevator
 * 7 - rudder
 */
std::vector<double> InnoVtolDynamicsSim::mapCmdToActuatorInnoVTOL(const std::vector<double>& cmd) const{
    if(cmd.size() != 8){
        std::cerr << "ERROR: InnoVtolDynamicsSim wrong control size. It is " << cmd.size()
                  << ", but should be 8" << std::endl;
        return cmd;
    }

    std::vector<double> actuators(8);
    actuators[0] = cmd[0];
    actuators[1] = cmd[1];
    actuators[2] = cmd[2];
    actuators[3] = cmd[3];

    actuators[4] = cmd[7];
    actuators[5] = cmd[4];      // aileron
    actuators[6] = cmd[5];      // elevator
    actuators[7] = cmd[6];      // rudder

    for(size_t idx = 0; idx < 5; idx++){
        actuators[idx] = boost::algorithm::clamp(actuators[idx], 0.0, +1.0);
        actuators[idx] *= params_.actuatorMax[idx];
    }

    actuators[5] = (actuators[5] - 0.5) * (2);
    for(size_t idx = 5; idx < 8; idx++){
        actuators[idx] = boost::algorithm::clamp(actuators[idx], -1.0, +1.0);
        actuators[idx] *= (actuators[idx] >= 0) ? params_.actuatorMax[idx] : -params_.actuatorMin[idx];
    }

    return actuators;
}

void InnoVtolDynamicsSim::updateActuators(std::vector<double>& cmd, double dtSecs){
    state_.prevActuators = state_.crntActuators;
    for(size_t idx = 0; idx < 8; idx++){
        state_.crntActuators[idx] = cmd[idx] + (state_.prevActuators[idx] - cmd[idx]) * (1 - pow(2.71, -dtSecs/tables_.actuatorTimeConstants[idx]));
        cmd[idx] = state_.crntActuators[idx];
    }
}

Eigen::Vector3d InnoVtolDynamicsSim::calculateWind(){
    Eigen::Vector3d wind;
    wind[0] = sqrt(state_.windVariance) * distribution_(generator_) + state_.windVelocity[0];
    wind[1] = sqrt(state_.windVariance) * distribution_(generator_) + state_.windVelocity[1];
    wind[2] = sqrt(state_.windVariance) * distribution_(generator_) + state_.windVelocity[2];

    /**
     * @todo Implement own gust logic
     * @note innopolis_vtol_indi logic doesn't suit us
     */
    Eigen::Vector3d gust;
    gust.setZero();

    return wind + gust;
}

Eigen::Matrix3d InnoVtolDynamicsSim::calculateRotationMatrix() const{
    return state_.attitude.toRotationMatrix().transpose();
}

Eigen::Vector3d InnoVtolDynamicsSim::calculateAirSpeed(const Eigen::Matrix3d& rotationMatrix,
                                                   const Eigen::Vector3d& velocity,
                                                   const Eigen::Vector3d& windSpeed) const{
    Eigen::Vector3d airspeed = rotationMatrix * (velocity - windSpeed);
    /**
     * @todo limit airspeed, because table values are limited
     */
    if(abs(airspeed[0]) > 40 || abs(airspeed[1]) > 40 || abs(airspeed[2]) > 40){
        airspeed[0] = boost::algorithm::clamp(airspeed[0], -40, +40);
        airspeed[1] = boost::algorithm::clamp(airspeed[1], -40, +40);
        airspeed[2] = boost::algorithm::clamp(airspeed[2], -40, +40);
        std::cout << "Warning: airspeed is out of limit." << std::endl;
    }
    return airspeed;
}

double InnoVtolDynamicsSim::calculateDynamicPressure(double airSpeedMod){
    return params_.atmoRho * airSpeedMod * airSpeedMod * params_.wingArea;
}

/**
 * @return AoA
 * it must be [0, 3.14] if angle is [0, +180]
 * it must be [0, -3.14] if angle is [0, -180]
 */
double InnoVtolDynamicsSim::calculateAnglesOfAtack(const Eigen::Vector3d& airSpeed) const{
    double A = sqrt(airSpeed[0] * airSpeed[0] + airSpeed[2] * airSpeed[2]);
    if(A < 0.001){
        return 0;
    }
    A = airSpeed[2] / A;
    A = boost::algorithm::clamp(A, -1.0, +1.0);
    A = (airSpeed[0] > 0) ? asin(A) : 3.1415 - asin(A);
    return (A > 3.1415) ? A - 2 * 3.1415 : A;
}

double InnoVtolDynamicsSim::calculateAnglesOfSideslip(const Eigen::Vector3d& airSpeed) const{
    double B = airSpeed.norm();
    if(B < 0.001){
        return 0;
    }
    B = airSpeed[1] / B;
    B = boost::algorithm::clamp(B, -1.0, +1.0);
    return asin(B);
}

/**
 * @note definitions:
 * FD, CD - drug force and drug coeeficient respectively
 * FL - lift force and lift coeeficient respectively
 * FS - side force and side coeeficient respectively
 */
void InnoVtolDynamicsSim::calculateAerodynamics(const Eigen::Vector3d& airspeed,
                                            double AoA,
                                            double AoS,
                                            double aileron_pos,
                                            double elevator_pos,
                                            double rudder_pos,
                                            Eigen::Vector3d& Faero,
                                            Eigen::Vector3d& Maero){
    // 0. Common computation
    double AoA_deg = boost::algorithm::clamp(AoA * 180 / 3.1415, -45.0, +45.0);
    double AoS_deg = boost::algorithm::clamp(AoS * 180 / 3.1415, -90.0, +90.0);
    double airspeedMod = airspeed.norm();
    double dynamicPressure = calculateDynamicPressure(airspeedMod);
    double airspeedModClamped = boost::algorithm::clamp(airspeed.norm(), 5, 40);

    // 1. Calculate aero force
    Eigen::VectorXd polynomialCoeffs(7);

    calculateCLPolynomial(airspeedModClamped, polynomialCoeffs);
    double CL = polyval(polynomialCoeffs, AoA_deg);
    Eigen::Vector3d FL = (Eigen::Vector3d(0, 1, 0).cross(airspeed.normalized())) * CL;

    calculateCSPolynomial(airspeedModClamped, polynomialCoeffs);
    double CS = polyval(polynomialCoeffs, AoA_deg);
    double CS_rudder = calculateCSRudder(rudder_pos, airspeedModClamped);
    double CS_beta = calculateCSBeta(AoS_deg, airspeedModClamped);
    Eigen::Vector3d FS = airspeed.cross(Eigen::Vector3d(0, 1, 0).cross(airspeed.normalized())) * (CS + CS_rudder + CS_beta);

    calculateCDPolynomial(airspeedModClamped, polynomialCoeffs);
    double CD = polyval(polynomialCoeffs.block<5, 1>(0, 0), AoA_deg);
    Eigen::Vector3d FD = (-1 * airspeed).normalized() * CD;

    Faero = 0.5 * dynamicPressure * (FL + FS + FD);

    // 2. Calculate aero moment
    calculateCmxPolynomial(airspeedModClamped, polynomialCoeffs);
    auto Cmx = polyval(polynomialCoeffs, AoA_deg);

    calculateCmyPolynomial(airspeedModClamped, polynomialCoeffs);
    auto Cmy = polyval(polynomialCoeffs, AoA_deg);

    calculateCmzPolynomial(airspeedModClamped, polynomialCoeffs);
    auto Cmz = -polyval(polynomialCoeffs, AoA_deg);

    double Cmx_aileron = calculateCmxAileron(aileron_pos, airspeedModClamped);
    /**
     * @note InnoDynamics from octave has some mistake in elevator logic
     * It always generate non positive moment in both positive and negative position
     * Temporary decision is to create positive moment in positive position and
     * negative moment in negative position
     */
    double Cmy_elevator = calculateCmyElevator(abs(elevator_pos), airspeedModClamped);
    double Cmz_rudder = calculateCmzRudder(rudder_pos, airspeedModClamped);

    auto Mx = Cmx + Cmx_aileron * aileron_pos;
    auto My = Cmy + Cmy_elevator * elevator_pos;
    auto Mz = Cmz + Cmz_rudder * rudder_pos;

    Maero = 0.5 * dynamicPressure * params_.characteristicLength * Eigen::Vector3d(Mx, My, Mz);


    state_.Flift << 0.5 * dynamicPressure * params_.characteristicLength * FL;
    state_.Fdrug << 0.5 * dynamicPressure * params_.characteristicLength * FD;
    state_.Fside << 0.5 * dynamicPressure * params_.characteristicLength * FS;
    state_.Msteer << Cmx_aileron * aileron_pos, Cmy_elevator * elevator_pos, Cmz_rudder * rudder_pos;
    state_.Msteer *= 0.5 * dynamicPressure * params_.characteristicLength;
    state_.Mairspeed << Cmx, Cmy, Cmz;
    state_.Mairspeed *= 0.5 * dynamicPressure * params_.characteristicLength;

    #if AERODYNAMICS_LOG == true
    if(abs(Faero[0]) > 20 || abs(Faero[1]) > 20 || abs(Faero[2]) > 20){
        std::cout << "in: AoA_deg=" << AoA_deg << std::endl;
        std::cout << "in: AoS_deg=" << AoS_deg << std::endl;
        std::cout << "in: aileron/elevator/rudder poses=" << aileron_pos << ", " << elevator_pos << ", " << rudder_pos << std::endl;
        std::cout << "in: airspeedMod=" << airspeedMod << std::endl;
        // std::cout << "CL=" << CL << std::endl;
        // std::cout << "CS=" << CS << std::endl;
        // std::cout << "CS_rudder=" << CS_rudder << std::endl;
        // std::cout << "CS_beta=" << CS_beta << std::endl;
        // std::cout << "CD=" << CD << std::endl;
        // std::cout << "FL=" << FL << std::endl;
        // std::cout << "FS=" << FS << std::endl;
        // std::cout << "FD=" << FD << std::endl;
        std::cout << "aileron:"     << Cmx_aileron << ", "  << aileron_pos << ", "  << state_.Msteer[0] << ", Mairspeed=" << state_.Mairspeed[0] << std::endl;
        std::cout << "elevator:"    << Cmy_elevator << ", " << elevator_pos << ", " << state_.Msteer[1] << ", Mairspeed=" << state_.Mairspeed[1] << std::endl;
        std::cout << "rudder:"      << Cmz_rudder << ", "   << rudder_pos << ", "   << state_.Msteer[2] << ", Mairspeed=" << state_.Mairspeed[2] << std::endl;
        std::cout << "out: Maero=" << Maero << std::endl;
        std::cout << "out: Faero=" << Faero << std::endl;
        std::cout << std::endl;
    }

    #endif
}

void InnoVtolDynamicsSim::thruster(double actuator,
                                   double& thrust, double& torque, double& rpm) const{
    constexpr size_t CONTROL_IDX = 0;
    constexpr size_t THRUST_IDX = 1;
    constexpr size_t TORQUE_IDX = 2;
    constexpr size_t RPM_IDX = 4;

    size_t prev_idx = findRow(tables_.prop, actuator);
    size_t next_idx = prev_idx + 1;
    if(next_idx < tables_.prop.rows()){
        auto prev_row = tables_.prop.row(prev_idx);
        auto next_row = tables_.prop.row(next_idx);
        auto t = (actuator - prev_row(CONTROL_IDX)) / (next_row(CONTROL_IDX) - prev_row(CONTROL_IDX));
        thrust = lerp(prev_row(THRUST_IDX), next_row(THRUST_IDX), t);
        torque = lerp(prev_row(TORQUE_IDX), next_row(TORQUE_IDX), t);
        rpm = lerp(prev_row(RPM_IDX), next_row(RPM_IDX), t);
    }
}

void InnoVtolDynamicsSim::calculateNewState(const Eigen::Vector3d& Maero,
                                        const Eigen::Vector3d& Faero,
                                        const std::vector<double>& actuator,
                                        double dt_sec){
    Eigen::VectorXd thrust(5), torque(5);
    for(size_t idx = 0; idx < 5; idx++){
        thruster(actuator[idx], thrust[idx], torque[idx], state_.motorsRpm[idx]);
    }

    for(size_t idx = 0; idx < 4; idx++){
        state_.Fmotors[idx] << 0, 0, -thrust[idx];
    }
    state_.Fmotors[4] << thrust[4], 0, 0;

    std::array<Eigen::Vector3d, 5> motorTorquesInBodyCS;
    motorTorquesInBodyCS[0] << 0, 0, torque[0];
    motorTorquesInBodyCS[1] << 0, 0, torque[1];
    motorTorquesInBodyCS[2] << 0, 0, -torque[2];
    motorTorquesInBodyCS[3] << 0, 0, -torque[3];
    motorTorquesInBodyCS[4] << -torque[4], 0, 0;
    std::array<Eigen::Vector3d, 5> MdueToArmOfForceInBodyCS;
    for(size_t idx = 0; idx < 5; idx++){
        MdueToArmOfForceInBodyCS[idx] = params_.propellersLocation[idx].cross(state_.Fmotors[idx]);
        state_.Mmotors[idx] = motorTorquesInBodyCS[idx] + MdueToArmOfForceInBodyCS[idx];
    }

    auto MtotalInBodyCS = std::accumulate(&state_.Mmotors[0], &state_.Mmotors[5], Maero);
    state_.angularAccel = params_.inertia.inverse() * (MtotalInBodyCS - state_.angularVel.cross(params_.inertia * state_.angularVel));
    state_.angularVel += state_.angularAccel * dt_sec;
    Eigen::Quaterniond attitudeDelta = state_.attitude * Eigen::Quaterniond(0, state_.angularVel(0), state_.angularVel(1), state_.angularVel(2));
    state_.attitude.coeffs() += attitudeDelta.coeffs() * 0.5 * dt_sec;
    state_.attitude.normalize();

    Eigen::Matrix3d rotationMatrix = calculateRotationMatrix();
    Eigen::Vector3d Fspecific = std::accumulate(&state_.Fmotors[0], &state_.Fmotors[5], Faero) / params_.mass;
    Eigen::Vector3d Ftotal = (Fspecific + rotationMatrix * Eigen::Vector3d(0, 0, params_.gravity)) * params_.mass;

    state_.Ftotal = Ftotal;
    state_.Mtotal = MtotalInBodyCS;

    state_.linearAccel = rotationMatrix.inverse() * Ftotal / params_.mass;
    state_.linearVel += state_.linearAccel * dt_sec;
    state_.position += state_.linearVel * dt_sec;

    #if MOMENTS_LOG == true
    static int counter = 0;
    constexpr int max_count = 1000;
    static Eigen::Vector3d MtotalSum(0, 0, 0);
    static Eigen::Vector3d MaeroSum(0, 0, 0);
    static Eigen::Vector3d MsteerSum(0, 0, 0);
    static Eigen::Vector3d Mairspeed(0, 0, 0);
    static Eigen::Vector3d AngularVelocitySum(0, 0, 0);
    if(counter > max_count){
        counter = 0;
        MtotalSum /= max_count;
        MaeroSum /= max_count;
        MsteerSum /= max_count;
        Mairspeed /= max_count;
        AngularVelocitySum /= max_count;
        std::cout << "Mtotal: " << MtotalSum.transpose() << ", " <<
                     "Maero: " << MaeroSum.transpose() << ", " <<
                     "Msteer: " << MsteerSum.transpose() << ", " <<
                     "Mairspeed: " << Mairspeed.transpose() << ", " <<
                     "angVelocity: " << AngularVelocitySum.transpose() << ", " <<
                     std::endl;
        MtotalSum << 0, 0, 0;
        MaeroSum << 0, 0, 0;
        AngularVelocitySum << 0, 0, 0;
    }
    MtotalSum += state_.Mtotal;
    MaeroSum += state_.Maero;
    MsteerSum += state_.Msteer;
    Mairspeed += state_.Mairspeed;
    AngularVelocitySum += state_.angularVel;
    counter++;
    #endif

    if(state_.position[2] >= 0){
        land();
    }else{
        state_.Fspecific = Fspecific;
    }

    #if STORE_SIM_PARAMETERS == true
    state_.MmotorsTotal[0] = std::accumulate(&state_.Mmotors[0][0], &state_.Mmotors[5][0], 0);
    state_.MmotorsTotal[1] = std::accumulate(&state_.Mmotors[0][0], &state_.Mmotors[5][0], 0);
    state_.MmotorsTotal[2] = std::accumulate(&state_.Mmotors[0][0], &state_.Mmotors[5][0], 0);
    state_.bodylinearVel = rotationMatrix * state_.linearVel;
    #endif

    #if FORCES_LOG == true
    std::cout << "- input: dt = "               << dt_sec << std::endl;
    std::cout << "- input: u = "                << actuator[0] << ", " << actuator[1] << ", " << actuator[2] << ", " << actuator[3] << ", "
                                                << actuator[4] << ", " << actuator[5] << ", " << actuator[6] << ", " << actuator[7] << std::endl;
    std::cout << "- input: Faero = "            << Faero.transpose() << std::endl;
    std::cout << "- input: Maero = "            << Maero.transpose() << std::endl;
    std::cout << "- input: state_.angularVel = "  << state_.angularVel.transpose() << std::endl;

    std::cout << "- motorTorquesInBodyCS: "     << motorTorquesInBodyCS[0].transpose() << ", " << motorTorquesInBodyCS[1].transpose() << ", " << motorTorquesInBodyCS[2].transpose() << ", "
                                                << motorTorquesInBodyCS[3].transpose() << ", " << motorTorquesInBodyCS[4].transpose() << std::endl;
    std::cout << "- MdueToArmOfForceInBodyCS: " << MdueToArmOfForceInBodyCS[0].transpose() << ", " << MdueToArmOfForceInBodyCS[1].transpose() << ", " << MdueToArmOfForceInBodyCS[2].transpose() << ", "
                                                << MdueToArmOfForceInBodyCS[3].transpose() << ", " << MdueToArmOfForceInBodyCS[4].transpose() << std::endl;
    std::cout << "- state_.Mmotors: "           << state_.Mmotors[0].transpose() << ", " << state_.Mmotors[1].transpose() << ", " << state_.Mmotors[2].transpose() << ", "
                                                << state_.Mmotors[3].transpose() << ", " << state_.Mmotors[4].transpose() << std::endl;
    std::cout << "- MtotalInBodyCS: "           << MtotalInBodyCS.transpose() << std::endl;

    std::cout << "- new rotationMatrix: "       << rotationMatrix(0, 0) << ", " << rotationMatrix(0, 1) << ", " << rotationMatrix(0, 2) << ";" << std::endl <<
                                                   rotationMatrix(1, 0) << ", " << rotationMatrix(1, 1) << ", " << rotationMatrix(1, 2) << ";" << std::endl <<
                                                   rotationMatrix(2, 0) << ", " << rotationMatrix(2, 1) << ", " << rotationMatrix(2, 2) << ";" << std::endl;

    std::cout << "- FmotorInBodyCS: "           << FmotorInBodyCS[0].transpose() << ", "
                                                << FmotorInBodyCS[1].transpose() << ", "
                                                << FmotorInBodyCS[2].transpose() << ", "
                                                << FmotorInBodyCS[3].transpose() << ", "
                                                << FmotorInBodyCS[4].transpose() << ", "
                                                << std::endl;
    std::cout << "- Fspecific="                 << Fspecific.transpose() << std::endl;

    std::cout << "- Ftotal="                    << Ftotal.transpose() << ", dt=" << dt_sec << ", "
              << ", mass="                      << params_.mass << std::endl;
    std::cout << "- out: state_.linearAccel="   << state_.linearAccel.transpose() << std::endl;
    std::cout << "- out: state_.angularAccel: " << state_.angularAccel.transpose() << std::endl;
    std::cout << "- out: state_.linearVel: "   << state_.linearVel.transpose() << std::endl;
    std::cout << "- out: state_.angularVel: "   << state_.angularVel.transpose() << std::endl;
    std::cout << "- out: state_.position = "    << state_.position.transpose() << std::endl;
    std::cout << "- out: state_.attitude = "    << state_.attitude.coeffs().transpose() << std::endl;
    #endif
}

Eigen::Vector3d InnoVtolDynamicsSim::calculateNormalForceWithoutMass(){
    Eigen::Matrix3d rotationMatrix = calculateRotationMatrix();
    return rotationMatrix * Eigen::Vector3d(0, 0, -params_.gravity);
}

void InnoVtolDynamicsSim::calculateCLPolynomial(double airSpeedMod,
                                                Eigen::VectorXd& polynomialCoeffs) const{
    calculatePolynomialUsingTable(tables_.CLPolynomial, airSpeedMod, polynomialCoeffs);
}
void InnoVtolDynamicsSim::calculateCSPolynomial(double airSpeedMod,
                                                Eigen::VectorXd& polynomialCoeffs) const{
    calculatePolynomialUsingTable(tables_.CSPolynomial, airSpeedMod, polynomialCoeffs);
}
void InnoVtolDynamicsSim::calculateCDPolynomial(double airSpeedMod,
                                                Eigen::VectorXd& polynomialCoeffs) const{
    calculatePolynomialUsingTable(tables_.CDPolynomial, airSpeedMod, polynomialCoeffs);
}
void InnoVtolDynamicsSim::calculateCmxPolynomial(double airSpeedMod,
                                                 Eigen::VectorXd& polynomialCoeffs) const{
    calculatePolynomialUsingTable(tables_.CmxPolynomial, airSpeedMod, polynomialCoeffs);
}
void InnoVtolDynamicsSim::calculateCmyPolynomial(double airSpeedMod,
                                                 Eigen::VectorXd& polynomialCoeffs) const{
    calculatePolynomialUsingTable(tables_.CmyPolynomial, airSpeedMod, polynomialCoeffs);
}
void InnoVtolDynamicsSim::calculateCmzPolynomial(double airSpeedMod,
                                                 Eigen::VectorXd& polynomialCoeffs) const{
    calculatePolynomialUsingTable(tables_.CmzPolynomial, airSpeedMod, polynomialCoeffs);
}
double InnoVtolDynamicsSim::calculateCSRudder(double rudder_pos, double airspeed) const{
    return griddata(-tables_.actuator, tables_.airspeed, tables_.CS_rudder, rudder_pos, airspeed);
}
double InnoVtolDynamicsSim::calculateCSBeta(double AoS_deg, double airspeed) const{
    return griddata(-tables_.AoS, tables_.airspeed, tables_.CS_beta, AoS_deg, airspeed);
}
double InnoVtolDynamicsSim::calculateCmxAileron(double aileron_pos, double airspeed) const{
    return griddata(tables_.actuator, tables_.airspeed, tables_.CmxAileron, aileron_pos, airspeed);
}
double InnoVtolDynamicsSim::calculateCmyElevator(double elevator_pos, double airspeed) const{
    return griddata(tables_.actuator, tables_.airspeed, tables_.CmyElevator, elevator_pos, airspeed);
}
double InnoVtolDynamicsSim::calculateCmzRudder(double rudder_pos, double airspeed) const{
    return griddata(tables_.actuator, tables_.airspeed, tables_.CmzRudder, rudder_pos, airspeed);
}

void InnoVtolDynamicsSim::calculatePolynomialUsingTable(const Eigen::MatrixXd& table,
                                                        double airSpeedMod,
                                                        Eigen::VectorXd& polynomialCoeffs) const{
    size_t prevRowIdx = findRow(table, airSpeedMod);
    if(prevRowIdx + 2 <= table.rows()){
        size_t nextRowIdx = prevRowIdx + 1;
        Eigen::MatrixXd prevRow = table.row(prevRowIdx);
        Eigen::MatrixXd nextRow = table.row(nextRowIdx);
        double t = (airSpeedMod - prevRow(0, 0)) / (nextRow(0, 0) - prevRow(0, 0));
        for(size_t idx = 0; idx < 7; idx++){
            polynomialCoeffs[idx] = lerp(prevRow(0, idx + 1), nextRow(0, idx + 1), t);
        }
    }else{
        for(size_t idx = 0; idx < 7; idx++){
            polynomialCoeffs[idx] = 0;
        }
    }
}

/**
 * @note size should be greater or equel than 2!
 * @todo think about binary search
 */
size_t InnoVtolDynamicsSim::search(const Eigen::MatrixXd& matrix, double key) const{
    size_t row_idx;
    if(matrix(matrix.rows() - 1, 0) > matrix(0, 0)){
        for(row_idx = 1; row_idx < matrix.rows() - 1; row_idx++){
            if(key <= matrix(row_idx, 0)){
                break;
            }
        }
        row_idx--;
    }else{
        for(row_idx = 1; row_idx < matrix.rows() - 1; row_idx++){
            if(key >= matrix(row_idx, 0)){
                break;
            }
        }
        row_idx--;
    }
    return row_idx;
}

// first collomn of the table must be sorted!
size_t InnoVtolDynamicsSim::findRow(const Eigen::MatrixXd& table, double value) const{
    size_t row = 0;
    size_t c = table.rows();
    while(row + 2 < c && table(row + 1, 0) < value){
        row++;
    }
    return row;
}

double InnoVtolDynamicsSim::lerp(double a, double b, double f) const{
    return a + f * (b - a);
}

double InnoVtolDynamicsSim::griddata(const Eigen::MatrixXd& x,
                                 const Eigen::MatrixXd& y,
                                 const Eigen::MatrixXd& z,
                                 double x_val,
                                 double y_val) const{
    size_t x1_idx = search(x, x_val);
    size_t y1_idx = search(y, y_val);
    size_t x2_idx = x1_idx + 1;
    size_t y2_idx = y1_idx + 1;
    double Q11 = z(y1_idx, x1_idx);
    double Q12 = z(y2_idx, x1_idx);
    double Q21 = z(y1_idx, x2_idx);
    double Q22 = z(y2_idx, x2_idx);
    double R1 = ((x(x2_idx) - x_val) * Q11 + (x_val - x(x1_idx)) * Q21) / (x(x2_idx) - x(x1_idx));
    double R2 = ((x(x2_idx) - x_val) * Q12 + (x_val - x(x1_idx)) * Q22) / (x(x2_idx) - x(x1_idx));
    double f =  ((y(y2_idx) - y_val) * R1  + (y_val - y(y1_idx)) * R2)  / (y(y2_idx) - y(y1_idx));
    return f;
}

double InnoVtolDynamicsSim::polyval(const Eigen::VectorXd& poly, double val) const{
    double result = 0;
    for(uint8_t idx = 0; idx < poly.rows(); idx++){
        result += poly[idx] * std::pow(val, poly.rows() - 1 - idx);
    }
    return result;
}

/**
 * @note These methods should return in NED format
 */
Eigen::Vector3d InnoVtolDynamicsSim::getVehiclePosition() const{
    return state_.position;
}
Eigen::Vector3d InnoVtolDynamicsSim::getVehicleVelocity() const{
    return state_.linearVel;
}

/**
 * @note These methods should return in FRD format
 */
Eigen::Quaterniond InnoVtolDynamicsSim::getVehicleAttitude() const{
    return state_.attitude;
}
Eigen::Vector3d InnoVtolDynamicsSim::getVehicleAngularVelocity() const{
    return state_.angularVel;
}
/**
 * @note We consider that z=0 means ground, so if position <=0, Normal force is appeared,
 * it means that in any way specific force will be equal to Gravity force.
 */
void InnoVtolDynamicsSim::getIMUMeasurement(Eigen::Vector3d& accOutFrd,
                                            Eigen::Vector3d& gyroOutFrd){
    Eigen::Vector3d specificForce(state_.Fspecific), angularVelocity(state_.angularVel);
    Eigen::Vector3d accNoise(sqrt(params_.accVariance) * distribution_(generator_),
                             sqrt(params_.accVariance) * distribution_(generator_),
                             sqrt(params_.accVariance) * distribution_(generator_));
    Eigen::Vector3d gyroNoise(sqrt(params_.gyroVariance) * distribution_(generator_),
                             sqrt(params_.gyroVariance) * distribution_(generator_),
                             sqrt(params_.gyroVariance) * distribution_(generator_));
    Eigen::Quaterniond imuOrient(1, 0, 0, 0);
    accOutFrd = imuOrient.inverse() * specificForce + state_.accelBias + accNoise;
    gyroOutFrd = imuOrient.inverse() * angularVelocity + state_.gyroBias + gyroNoise;
}

/**
 * @note These methods should be private
 */
void InnoVtolDynamicsSim::setWindParameter(Eigen::Vector3d windMeanVelocity,
                                       double windVariance){
    state_.windVelocity = windMeanVelocity;
    state_.windVariance = windVariance;
}
Eigen::Vector3d InnoVtolDynamicsSim::getAngularAcceleration() const{
    return state_.angularAccel;
}
Eigen::Vector3d InnoVtolDynamicsSim::getLinearAcceleration() const{
    return state_.linearAccel;
}
Eigen::Vector3d InnoVtolDynamicsSim::getFaero() const{
    return state_.Faero;
}
Eigen::Vector3d InnoVtolDynamicsSim::getFtotal() const{
    return state_.Ftotal;
}
Eigen::Vector3d InnoVtolDynamicsSim::getMsteer() const{
    return state_.Msteer;
}
Eigen::Vector3d InnoVtolDynamicsSim::getMairspeed() const{
    return state_.Mairspeed;
}
Eigen::Vector3d InnoVtolDynamicsSim::getMmotorsTotal() const{
    return state_.MmotorsTotal;
}
Eigen::Vector3d InnoVtolDynamicsSim::getBodyLinearVelocity() const{
    return state_.bodylinearVel;
}
Eigen::Vector3d InnoVtolDynamicsSim::getMaero() const{
    return state_.Maero;
}
Eigen::Vector3d InnoVtolDynamicsSim::getMtotal() const{
    return state_.Mtotal;
}
Eigen::Vector3d InnoVtolDynamicsSim::getFlift() const{
    return state_.Flift;
}
Eigen::Vector3d InnoVtolDynamicsSim::getFdrug() const{
    return state_.Fdrug;
}
Eigen::Vector3d InnoVtolDynamicsSim::getFside() const{
    return state_.Fside;
}
const std::array<Eigen::Vector3d, 5>& InnoVtolDynamicsSim::getFmotors() const{
    return state_.Fmotors;
}
const std::array<Eigen::Vector3d, 5>& InnoVtolDynamicsSim::getMmotors() const{
    return state_.Mmotors;
}

bool InnoVtolDynamicsSim::getMotorsRpm(std::vector<double>& motorsRpm) {
    motorsRpm.push_back(state_.motorsRpm[0]);
    motorsRpm.push_back(state_.motorsRpm[1]);
    motorsRpm.push_back(state_.motorsRpm[2]);
    motorsRpm.push_back(state_.motorsRpm[3]);
    motorsRpm.push_back(state_.motorsRpm[4]);
    return true;
}
