/**
 * @file vtolDynamicsSim.cpp
 * @author ponomarevda96@gmail.com
 * @brief Vtol dynamics simulator class implementation
 */
#include <iostream>
#include <cmath>
#include <boost/algorithm/clamp.hpp>
#include <algorithm>
#include "yaml-cpp/yaml.h"
#include "vtolDynamicsSim.hpp"
#include <ros/package.h>
#include <array>


InnoVtolDynamicsSim::InnoVtolDynamicsSim(): distribution_(0.0, 1.0){
    state_.angularVel.setZero();
    state_.linearVel.setZero();
    state_.windVelocity.setZero();
    state_.windVariance = 0;
}

int8_t InnoVtolDynamicsSim::init(){
    std::string configPath = ros::package::getPath("innopolis_vtol_dynamics") + "/config/";
    loadTables(configPath + "aerodynamics_coeffs.yaml");
    loadParams(configPath + "vtol_params.yaml");
    state_.Ftotal = Eigen::Vector3d(0, 0, params_.gravity * params_.mass);
    return 0;
}

void InnoVtolDynamicsSim::loadTables(const std::string& path){
    YAML::Node config = YAML::LoadFile(path);
    std::vector<double> vectorTable;

    vectorTable = config["CS_rudder_table"].as< std::vector<double> >();
    tables_.CS_rudder = Eigen::Map<Eigen::Matrix<double, 8, 20, Eigen::RowMajor>>((double*)&vectorTable[0], 8, 20);

    vectorTable = config["CS_beta"].as< std::vector<double> >();
    tables_.CS_beta = Eigen::Map<Eigen::Matrix<double, 8, 90, Eigen::RowMajor>>((double*)&vectorTable[0], 8, 90);

    vectorTable = config["AoA"].as< std::vector<double> >();
    tables_.AoA = Eigen::Map<Eigen::Matrix<double, 1, 47, Eigen::RowMajor>>((double*)&vectorTable[0], 1, 47);

    vectorTable = config["AoS"].as< std::vector<double> >();
    tables_.AoS = Eigen::Map<Eigen::Matrix<double, 1, 90, Eigen::RowMajor>>((double*)&vectorTable[0], 1, 90);
    tables_.AoS = tables_.AoS.transpose();

    vectorTable = config["actuator_table"].as< std::vector<double> >();
    tables_.actuator = Eigen::Map<Eigen::Matrix<double, 1, 20, Eigen::RowMajor>>((double*)&vectorTable[0], 1, 20);
    tables_.actuator = tables_.actuator.transpose();

    vectorTable = config["airspeed_table"].as< std::vector<double> >();
    tables_.airspeed = Eigen::Map<Eigen::Matrix<double, 1, 8, Eigen::RowMajor>>((double*)&vectorTable[0], 1, 8);
    tables_.airspeed = tables_.airspeed.transpose();

    vectorTable = config["CLPolynomial"].as< std::vector<double> >();
    tables_.CLPolynomial = Eigen::Map<Eigen::Matrix<double, 8, 8, Eigen::RowMajor>>((double*)&vectorTable[0], 8, 8);

    vectorTable = config["CSPolynomial"].as< std::vector<double> >();
    tables_.CSPolynomial = Eigen::Map<Eigen::Matrix<double, 8, 8, Eigen::RowMajor>>((double*)&vectorTable[0], 8, 8);

    vectorTable = config["CDPolynomial"].as< std::vector<double> >();
    tables_.CDPolynomial = Eigen::Map<Eigen::Matrix<double, 8, 6, Eigen::RowMajor>>((double*)&vectorTable[0], 8, 6);

    vectorTable = config["CmxPolynomial"].as< std::vector<double> >();
    tables_.CmxPolynomial = Eigen::Map<Eigen::Matrix<double, 8, 8, Eigen::RowMajor>>((double*)&vectorTable[0], 8, 8);

    vectorTable = config["CmyPolynomial"].as< std::vector<double> >();
    tables_.CmyPolynomial = Eigen::Map<Eigen::Matrix<double, 8, 8, Eigen::RowMajor>>((double*)&vectorTable[0], 8, 8);

    vectorTable = config["CmzPolynomial"].as< std::vector<double> >();
    tables_.CmzPolynomial = Eigen::Map<Eigen::Matrix<double, 8, 8, Eigen::RowMajor>>((double*)&vectorTable[0], 8, 8);

    vectorTable = config["CmxAileron"].as< std::vector<double> >();
    tables_.CmxAileron = Eigen::Map<Eigen::Matrix<double, 8, 20, Eigen::RowMajor>>((double*)&vectorTable[0], 8, 20);

    vectorTable = config["CmyElevator"].as< std::vector<double> >();
    tables_.CmyElevator = Eigen::Map<Eigen::Matrix<double, 8, 20, Eigen::RowMajor>>((double*)&vectorTable[0], 8, 20);

    vectorTable = config["CmzRudder"].as< std::vector<double> >();
    tables_.CmzRudder = Eigen::Map<Eigen::Matrix<double, 8, 20, Eigen::RowMajor>>((double*)&vectorTable[0], 8, 20);

    vectorTable = config["prop"].as< std::vector<double> >();
    tables_.prop = Eigen::Map<Eigen::Matrix<double, 40, 4, Eigen::RowMajor>>((double*)&vectorTable[0], 40, 4);
}

void InnoVtolDynamicsSim::loadParams(const std::string& path){
    YAML::Node config = YAML::LoadFile(path);
    params_.mass = config["mass"].as<double>();
    params_.gravity = config["gravity"].as<double>();
    params_.atmoRho = config["atmoRho"].as<double>();
    params_.wingArea = config["wingArea"].as<double>();
    params_.characteristicLength = config["characteristicLength"].as<double>();

    double propellersLocationX, propellersLocationY, propellersLocationZ;
    propellersLocationX = config["propellersLocationX"].as<double>();
    propellersLocationY = config["propellersLocationY"].as<double>();
    propellersLocationZ = config["propellersLocationZ"].as<double>();
    params_.propellersLocation[0] = Eigen::Vector3d(propellersLocationX * sin(3.1415/4),
                                                    propellersLocationY * sin(3.1415/4),
                                                    propellersLocationZ);
    params_.propellersLocation[1] = Eigen::Vector3d(-propellersLocationX * sin(3.1415/4),
                                                    -propellersLocationY * sin(3.1415/4),
                                                    propellersLocationZ);
    params_.propellersLocation[2] = Eigen::Vector3d(propellersLocationX * sin(3.1415/4),
                                                    -propellersLocationY * sin(3.1415/4),
                                                    propellersLocationZ);
    params_.propellersLocation[3] = Eigen::Vector3d(-propellersLocationX * sin(3.1415/4),
                                                    propellersLocationY * sin(3.1415/4),
                                                    propellersLocationZ);
    params_.propellersLocation[4] = Eigen::Vector3d(propellersLocationX,
                                                    0,
                                                    0);

    std::vector<double> vectorTable;
    vectorTable = config["inertia"].as< std::vector<double> >();
    params_.inertia = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>((double*)&vectorTable[0], 3, 3);

    params_.actuatorMin = config["actuatorMin"].as< std::vector<double> >();
    params_.actuatorMax = config["actuatorMax"].as< std::vector<double> >();
    params_.accVariance = config["accVariance"].as<double>();
    params_.gyroVariance = config["gyroVariance"].as<double>();
}

void InnoVtolDynamicsSim::setReferencePosition(double latRef, double lonRef, double altRef){
    geodetic_converter_.initialiseReference(latRef, lonRef, altRef);
}
void InnoVtolDynamicsSim::setInitialPosition(const Eigen::Vector3d & position,
                                         const Eigen::Quaterniond& attitude){
    state_.position = position;
    state_.attitude = attitude;
}
void InnoVtolDynamicsSim::setInitialVelocity(const Eigen::Vector3d & linearVelocity,
                                         const Eigen::Vector3d& angularVelocity){
    state_.linearVel = linearVelocity;
    state_.angularVel = angularVelocity;
}

void InnoVtolDynamicsSim::initStaticMotorTransform(){
    Eigen::Isometry3d motorFrame = Eigen::Isometry3d::Identity();
    auto time = ros::Time::now();

    motorFrame.translation() = params_.propellersLocation[0];
    publishStaticMotorTransform(time, "uav/imu", "uav/motor0", motorFrame);

    motorFrame.translation() = params_.propellersLocation[1];
    publishStaticMotorTransform(time, "uav/imu", "uav/motor1", motorFrame);

    motorFrame.translation() = params_.propellersLocation[2];
    publishStaticMotorTransform(time, "uav/imu", "uav/motor2", motorFrame);

    motorFrame.translation() = params_.propellersLocation[3];
    publishStaticMotorTransform(time, "uav/imu", "uav/motor3", motorFrame);
}

void InnoVtolDynamicsSim::process(double dt_secs,
                              const std::vector<double>& motorCmd,
                              bool isCmdPercent){
    Eigen::Vector3d vel_w = calculateWind();
    Eigen::Matrix3d rotationMatrix = calculateRotationMatrix();
    auto linearVelInveted = state_.linearVel;
    Eigen::Vector3d airSpeed = calculateAirSpeed(rotationMatrix, linearVelInveted, vel_w);
    double AoA = -calculateAnglesOfAtack(airSpeed);
    double AoS = -calculateAnglesOfSideslip(airSpeed);
    double Cmx_a, Cmy_e, Cmz_r;
    auto actuators = isCmdPercent ? mapCmdToActuator(motorCmd) : motorCmd;
    calculateAerodynamics(airSpeed, AoA, AoS, actuators[5], actuators[6], actuators[7],
                          state_.Faero, state_.Maero, Cmx_a, Cmy_e, Cmz_r);

    /**
     * @note In InnoDynamics the altitude is directed to the bottom, but in this simulator
     * it is directed to the top, so we perform invertion.
     * forces z axis was inverted
     * Moments and angular x, y were inverted
     */
    Eigen::Vector3d FaeroInverted(state_.Faero[0], state_.Faero[1], -state_.Faero[2]);
    Eigen::Vector3d MaeroInverted(-state_.Maero[0], -state_.Maero[1], state_.Maero[2]);
    calculateNewState(MaeroInverted, FaeroInverted, actuators, dt_secs);
}

/**
 * @note inno dynamics motor indexes is not correspond to px4 configuration
 */
std::vector<double> InnoVtolDynamicsSim::mapCmdToActuator(const std::vector<double>& cmd) const{
    if(cmd.size() != 8){
        std::cerr << "ERROR: InnoVtolDynamicsSim wrong control size. It is " << cmd.size()
                  << ", but should be 8" << std::endl;
        return cmd;
    }

    std::vector<double> actuators(8);
    actuators[0] = cmd[0];
    actuators[1] = cmd[2];
    actuators[2] = cmd[3];
    actuators[3] = cmd[1];

    actuators[4] = cmd[4];

    /**
     * @note this is hack, because in this moment we use standard vtol airframe (2 ailerons and elevator)
     * https://github.com/InnopolisAero/Inno_PX4_Firmware/blob/e28f8a7f2e181680353cd23ed5c62c4e9b5858fc/ROMFS/px4fmu_common/mixers-sitl/standard_vtol_sitl.main.mix
     * but this dynamics expected Inno VTOL airframe (aileron + elevator + rudder)
     */
    actuators[5] = (cmd[5] - cmd[6]) / 2;   // aileron     roll
    actuators[6] = -cmd[7];                  // elevator     pitch
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
                                                   const Eigen::Vector3d& estimatedVelocity,
                                                   const Eigen::Vector3d& windSpeed){
    return rotationMatrix * (estimatedVelocity - windSpeed);
}

double InnoVtolDynamicsSim::calculateDynamicPressure(double airSpeedMod){
    return params_.atmoRho * airSpeedMod * airSpeedMod * params_.wingArea;
}

double InnoVtolDynamicsSim::calculateAnglesOfAtack(const Eigen::Vector3d& airSpeed) const{
    double A = sqrt(airSpeed[0] * airSpeed[0] + airSpeed[2] * airSpeed[2]);
    if(A == 0){
        return 0;
    }
    A = airSpeed[2] / A;
    A = boost::algorithm::clamp(A, -1.0, +1.0);
    A = (airSpeed[0] > 0) ? asin(A) : 3.1415 - asin(A);
    return (A > 3.1415) ? A - 2 * 3.1415 : A;
}

double InnoVtolDynamicsSim::calculateAnglesOfSideslip(const Eigen::Vector3d& airSpeed) const{
    double B = sqrt(airSpeed[0]*airSpeed[0] + airSpeed[1]*airSpeed[1] + airSpeed[2]*airSpeed[2]);
    if(B == 0){
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
                                            Eigen::Vector3d& Maero,
                                            double& Cmx_aileron,
                                            double& Cmy_elevator,
                                            double& Cmx_rudder){
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

    #define AERODYNAMICS_LOG false
    #if AERODYNAMICS_LOG == true
    std::cout << "AoA_deg=" << AoA_deg << std::endl;
    std::cout << "AoS_deg=" << AoS_deg << std::endl;
    std::cout << "rudder_pos=" << rudder_pos << std::endl;
    std::cout << "airspeedMod=" << airspeedMod << std::endl;
    std::cout << "CL=" << CL << std::endl;
    std::cout << "CS=" << CS << std::endl;
    std::cout << "CS_rudder=" << CS_rudder << std::endl;
    std::cout << "CS_beta=" << CS_beta << std::endl;
    std::cout << "CD=" << CD << std::endl;
    std::cout << "FL=" << FL << std::endl;
    std::cout << "FS=" << FS << std::endl;
    std::cout << "FD=" << FD << std::endl;
    std::cout << "Faero=" << Faero << std::endl;
    #endif

    // 2. Calculate aero moment
    calculateCmxPolynomial(airspeedModClamped, polynomialCoeffs);
    auto Cmx = polyval(polynomialCoeffs, AoA_deg);

    calculateCmyPolynomial(airspeedModClamped, polynomialCoeffs);
    auto Cmy = polyval(polynomialCoeffs, AoA_deg);

    calculateCmzPolynomial(airspeedModClamped, polynomialCoeffs);
    auto Cmz = -polyval(polynomialCoeffs, AoA_deg);

    Cmx_aileron = calculateCmxAileron(aileron_pos, airspeedModClamped);
    Cmy_elevator = calculateCmyElevator(elevator_pos, airspeedModClamped);
    Cmx_rudder = calculateCmzRudder(rudder_pos, airspeedModClamped);

    auto Mx = Cmx + Cmx_aileron * aileron_pos;
    auto My = Cmy + Cmy_elevator * elevator_pos;
    auto Mz = Cmz + Cmx_rudder * rudder_pos;

    Maero = 0.5 * dynamicPressure * params_.characteristicLength * Eigen::Vector3d(Mx, My, Mz);
}

void InnoVtolDynamicsSim::thruster(double actuator, double& thrust, double& torque) const{
    constexpr size_t ACTUATOR_IDX = 0;
    constexpr size_t THRUST_IDX = 1;
    constexpr size_t TORQUE_IDX = 2;

    size_t prev_idx = findRow(tables_.prop, actuator);
    size_t next_idx = prev_idx + 1;
    if(next_idx < tables_.prop.rows()){
        auto prev_row = tables_.prop.row(prev_idx);
        auto next_row = tables_.prop.row(next_idx);
        auto t = (actuator - prev_row(ACTUATOR_IDX)) / (next_row(ACTUATOR_IDX) - prev_row(ACTUATOR_IDX));
        thrust = lerp(prev_row(THRUST_IDX), next_row(THRUST_IDX), t);
        torque = lerp(prev_row(TORQUE_IDX), next_row(TORQUE_IDX), t);
    }
}

void InnoVtolDynamicsSim::calculateNewState(const Eigen::Vector3d& Maero,
                                        const Eigen::Vector3d& Faero,
                                        const std::vector<double>& actuator,
                                        double dt_sec){
    Eigen::VectorXd thrust(5), torque(5);
    for(size_t idx = 0; idx < 5; idx++){
        thruster(actuator[idx], thrust[idx], torque[idx]);
    }

    std::array<Eigen::Vector3d, 5> FmotorInBodyCS;
    for(size_t idx = 0; idx < 4; idx++){
        FmotorInBodyCS[idx] << 0, 0, thrust[idx];
    }
    FmotorInBodyCS[4] << thrust[4], 0, 0;

    std::array<Eigen::Vector3d, 5> motorTorquesInBodyCS;
    motorTorquesInBodyCS[0] << 0, 0, torque[0];
    motorTorquesInBodyCS[1] << 0, 0, torque[1];
    motorTorquesInBodyCS[2] << 0, 0, -torque[2];
    motorTorquesInBodyCS[3] << 0, 0, -torque[3];
    motorTorquesInBodyCS[4] << torque[4], 0, 0;
    std::array<Eigen::Vector3d, 5> MdueToArmOfForceInBodyCS;
    std::array<Eigen::Vector3d, 5> MmotorsTotal;
    for(size_t idx = 0; idx < 5; idx++){
        MdueToArmOfForceInBodyCS[idx] = params_.propellersLocation[idx].cross(FmotorInBodyCS[idx]);
        MmotorsTotal[idx] = motorTorquesInBodyCS[idx] + MdueToArmOfForceInBodyCS[idx];
    }

    /**
     * @note In InnoDynamics the altitude is directed to the bottom, but in this simulator
     * it is directed to the top, so we perform invertion.
     * forces z axis was inverted
     */
    Eigen::Vector3d AngularVelInverted(state_.angularVel[0], state_.angularVel[1], -state_.angularVel[2]);

    auto MtotalInBodyCS = std::accumulate(&MmotorsTotal[0], &MmotorsTotal[5], Maero);
    state_.angularAccel = params_.inertia.inverse() * (MtotalInBodyCS - AngularVelInverted.cross(params_.inertia * AngularVelInverted));
    state_.angularVel += state_.angularAccel * dt_sec;
    Eigen::Quaterniond attitudeDelta = state_.attitude * Eigen::Quaterniond(0, state_.angularVel(0), state_.angularVel(1), state_.angularVel(2));
    attitudeDelta.coeffs() *= 0.5 * dt_sec;
    state_.attitude.coeffs() += attitudeDelta.coeffs();
    state_.attitude.normalize();

    Eigen::Matrix3d rotationMatrix = state_.attitude.toRotationMatrix().transpose();
    Eigen::Vector3d Fspecific = std::accumulate(&FmotorInBodyCS[0], &FmotorInBodyCS[5], Faero);
    Eigen::Vector3d Ftotal = Fspecific - rotationMatrix * Eigen::Vector3d(0, 0, params_.mass * params_.gravity);

    state_.Fspecific = Fspecific;
    state_.Ftotal = Ftotal;
    state_.Mtotal = MtotalInBodyCS;

    state_.linearAccel = rotationMatrix.inverse() * Ftotal / params_.mass;
    state_.linearVel += state_.linearAccel * dt_sec;
    state_.position += state_.linearVel * dt_sec;

    if(state_.position[2] < 0){
        state_.linearVel << 0.0, 0.0, 0.0;
        state_.angularVel << 0.0, 0.0, 0.0;
        state_.position[2] = 0.00;
        state_.attitude.w() = 1;
        state_.attitude.x() = 0;
        state_.attitude.y() = 0;
        state_.attitude.z() = 0;
    }

    #define FORCES_LOG false
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
    std::cout << "- MmotorsTotal: "             << MmotorsTotal[0].transpose() << ", " << MmotorsTotal[1].transpose() << ", " << MmotorsTotal[2].transpose() << ", " << MmotorsTotal[3].transpose() << ", "
                                                << MmotorsTotal[4].transpose() << std::endl;
    std::cout << "- MtotalInBodyCS: "           << MtotalInBodyCS.transpose() << std::endl;

    std::cout << "- new rotationMatrix: "       << rotationMatrix(0,0) << ", " << rotationMatrix(0,1) << ", " << rotationMatrix(0,2) << ";" << std::endl <<
                                                   rotationMatrix(1,0) << ", " << rotationMatrix(1,1) << ", " << rotationMatrix(1,2) << ";" << std::endl <<
                                                   rotationMatrix(2,0) << ", " << rotationMatrix(2,1) << ", " << rotationMatrix(2,2) << ";" << std::endl;

    std::cout << "- FmotorInBodyCS: "           << FmotorInBodyCS[0].transpose() << ", " << FmotorInBodyCS[1].transpose() << ", " << FmotorInBodyCS[2].transpose() << ", "
                                                << FmotorInBodyCS[3].transpose() << ", " << FmotorInBodyCS[4].transpose() << ", " << std::endl;
    std::cout << "- Fspecific="                 << Fspecific.transpose() << std::endl;

    std::cout << "- Ftotal="                    << Ftotal.transpose() << ", dt=" << dt_sec << ", " << ", mass=" << params_.mass << std::endl;
    std::cout << "- out: state_.linearAccel="   << state_.linearAccel.transpose() << std::endl;
    std::cout << "- out: state_.angularAccel: " << state_.angularAccel.transpose() << std::endl;
    std::cout << "- out: state_.angularVel: "   << state_.angularVel.transpose() << std::endl;
    std::cout << "- out: state_.attitude = "    << state_.attitude.coeffs().transpose() << std::endl;
    #endif
}

void InnoVtolDynamicsSim::calculateCLPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    calculatePolynomialUsingTable(tables_.CLPolynomial, airSpeedMod, polynomialCoeffs);
}
void InnoVtolDynamicsSim::calculateCSPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    calculatePolynomialUsingTable(tables_.CSPolynomial, airSpeedMod, polynomialCoeffs);
}
void InnoVtolDynamicsSim::calculateCDPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    calculatePolynomialUsingTable(tables_.CDPolynomial, airSpeedMod, polynomialCoeffs);
}
void InnoVtolDynamicsSim::calculateCmxPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    calculatePolynomialUsingTable(tables_.CmxPolynomial, airSpeedMod, polynomialCoeffs);
}
void InnoVtolDynamicsSim::calculateCmyPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    calculatePolynomialUsingTable(tables_.CmyPolynomial, airSpeedMod, polynomialCoeffs);
}
void InnoVtolDynamicsSim::calculateCmzPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
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
                                                    Eigen::VectorXd& polynomialCoeffs){
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
    size_t row;
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
                                 double x_value,
                                 double y_value) const{
    size_t x1_idx = search(x, x_value);
    size_t y1_idx = search(y, y_value);
    size_t x2_idx = x1_idx + 1;
    size_t y2_idx = y1_idx + 1;
    double Q11 = z(y1_idx, x1_idx);
    double Q12 = z(y2_idx, x1_idx);
    double Q21 = z(y1_idx, x2_idx);
    double Q22 = z(y2_idx, x2_idx);
    double R1 = ((x(x2_idx) - x_value) * Q11 + (x_value - x(x1_idx)) * Q21) / (x(x2_idx) - x(x1_idx));
    double R2 = ((x(x2_idx) - x_value) * Q12 + (x_value - x(x1_idx)) * Q22) / (x(x2_idx) - x(x1_idx));
    double f =  ((y(y2_idx) - y_value) * R1  + (y_value - y(y1_idx)) * R2)  / (y(y2_idx) - y(y1_idx));
    return f;
}

double InnoVtolDynamicsSim::polyval(const Eigen::VectorXd& poly, double val) const{
    double result = 0;
    for(uint8_t idx = 0; idx < poly.rows(); idx++){
        result += poly[idx] * std::pow(val, poly.rows() - 1 - idx);
    }
    return result;
}


void InnoVtolDynamicsSim::setWindParameter(Eigen::Vector3d windMeanVelocity,
                                       double windVariance){
    state_.windVelocity = windMeanVelocity;
    state_.windVariance = windVariance;
}
void InnoVtolDynamicsSim::setEulerAngles(Eigen::Vector3d eulerAngles){
    state_.eulerAngles = eulerAngles;
}


Eigen::Vector3d InnoVtolDynamicsSim::getAngularAcceleration() const{
    return state_.angularAccel;
}
Eigen::Vector3d InnoVtolDynamicsSim::getVehicleAngularVelocity() const{
    return state_.angularVel;
}
Eigen::Quaterniond InnoVtolDynamicsSim::getVehicleAttitude() const{
    return state_.attitude;
}
Eigen::Vector3d InnoVtolDynamicsSim::getLinearAcceleration() const{
    return state_.linearAccel;
}
Eigen::Vector3d InnoVtolDynamicsSim::getVehicleVelocity() const{
    return state_.linearVel;
}
Eigen::Vector3d InnoVtolDynamicsSim::getVehiclePosition() const{
    return state_.position;
}
Eigen::Vector3d InnoVtolDynamicsSim::getFaero() const{
    return state_.Faero;
}
Eigen::Vector3d InnoVtolDynamicsSim::getFtotal() const{
    return state_.Ftotal;
}
Eigen::Vector3d InnoVtolDynamicsSim::getMaero() const{
    return state_.Maero;
}
Eigen::Vector3d InnoVtolDynamicsSim::getMtotal() const{
    return state_.Mtotal;
}
void InnoVtolDynamicsSim::enu2Geodetic(double east, double north, double up,
                                   double *latitude, double *longitude, double *altitude){
    geodetic_converter_.enu2Geodetic(east, north, up, latitude, longitude, altitude);
}
void InnoVtolDynamicsSim::getIMUMeasurement(Eigen::Vector3d& accOut, Eigen::Vector3d& gyroOut){
    /**
     * @note We consider that z=0 means ground, so if position <=0, Normal force is appeared,
     * it means that in any way specific force will be equal to Gravity force.
     */
    Eigen::Vector3d specificForce;
    if(state_.position[2] <= 0){
        specificForce << 0, 0, params_.gravity;
    }else{
        specificForce = state_.Fspecific / params_.mass;
    }
    Eigen::Vector3d accNoise(sqrt(params_.accVariance) * distribution_(generator_),
                             sqrt(params_.accVariance) * distribution_(generator_),
                             sqrt(params_.accVariance) * distribution_(generator_));
    Eigen::Vector3d gyroNoise(sqrt(params_.gyroVariance) * distribution_(generator_),
                             sqrt(params_.gyroVariance) * distribution_(generator_),
                             sqrt(params_.gyroVariance) * distribution_(generator_));
    Eigen::Quaterniond imuOrient(1, 0, 0, 0);
    accOut = imuOrient.inverse() * specificForce + state_.accelBias + accNoise;
    gyroOut = imuOrient.inverse() * state_.angularVel + state_.gyroBias + gyroNoise;
}