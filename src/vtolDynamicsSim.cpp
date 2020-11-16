/**
 * @file vtolDynamicsSim.cpp
 * @author ponomarevda96@gmail.com
 * @brief Vtol dynamics simulator class implementation
 * 
 */
#include <iostream>
// #include <chrono>
#include <cmath>
#include <boost/algorithm/clamp.hpp>
#include <algorithm>
#include "yaml-cpp/yaml.h"
#include "vtolDynamicsSim.hpp"
#include <ros/package.h>
#include <array>


VtolDynamicsSim::VtolDynamicsSim():
    distribution_(0.0, 1.0){
}

void VtolDynamicsSim::init(){
    std::string configPath = ros::package::getPath("innopolis_vtol_dynamics") + "/config/";
    loadTables(configPath + "aerodynamics_coeffs.yaml");
    loadParams(configPath + "vtol_params.yaml");

    /**
     * @todo now it is zero because it interferes on processStep(), load in in future
     */
    state_.angularVel.setZero();
}

void VtolDynamicsSim::loadTables(const std::string& path){
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

void VtolDynamicsSim::loadParams(const std::string& path){
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
}

void VtolDynamicsSim::processStep(double dt_secs,
                                  const std::vector<double> & motorSpeedCommandIn,
                                  bool isCmdPercent){
    Eigen::Vector3d vel_w = calculateWind();
    Eigen::Matrix3d rotationMatrix = calculateRotationMatrix();
    Eigen::Vector3d airSpeed = calculateAirSpeed(rotationMatrix, state_.estLinearVel, vel_w);
    double airSpeedMod = airSpeed.norm();
    double dynPressure = calculateDynamicPressure(airSpeedMod);
    double AoA = calculateAnglesOfAtack(airSpeed);
    double AoS = calculateAnglesOfSideslip(airSpeed);
    // [Faero, Maero, Cmx_a, Cmy_e, Cmz_r, AoA_min, AoA_max] = aerodynamics(V_mod, Va_b, dyn_press, AoA, AoS, atmo_rho, l, u_0(6), u_0(7), u_0(8));
}

Eigen::Vector3d VtolDynamicsSim::calculateWind(){
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

Eigen::Matrix3d VtolDynamicsSim::calculateRotationMatrix() const{
    double phi = state_.eulerAngles[0];
    double theta = state_.eulerAngles[1];
    double psi = state_.eulerAngles[2];
    Eigen::Quaterniond q_phi(std::cos(phi/2), std::sin(phi/2), 0, 0);
    Eigen::Quaterniond q_theta(std::cos(theta/2), 0, std::sin(theta/2), 0);
    Eigen::Quaterniond q_psi(std::cos(psi/2), 0, 0, std::sin(psi/2));
    Eigen::Quaterniond q = q_psi * q_theta * q_phi;
    return q.toRotationMatrix();
}

Eigen::Vector3d VtolDynamicsSim::calculateAirSpeed(const Eigen::Matrix3d& rotationMatrix,
                                                   const Eigen::Vector3d& estimatedVelocity,
                                                   const Eigen::Vector3d& windSpeed){
    return rotationMatrix * (estimatedVelocity - windSpeed);
}

double VtolDynamicsSim::calculateDynamicPressure(double airSpeedMod){
    return params_.atmoRho * airSpeedMod * airSpeedMod * params_.wingArea;
}

double VtolDynamicsSim::calculateAnglesOfAtack(const Eigen::Vector3d& airSpeed) const{
    double A = sqrt(airSpeed[0] * airSpeed[0] + airSpeed[2] * airSpeed[2]);
    if(A == 0){
        return 0;
    }
    A = airSpeed[0] / A;
    A = boost::algorithm::clamp(A, -1.0, +1.0);
    A = (airSpeed[0] > 0) ? asin(A) : 3.1415 - asin(A);
    return (A > 3.1415) ? A - 2 * 3.1415 : A;
}

double VtolDynamicsSim::calculateAnglesOfSideslip(const Eigen::Vector3d& airSpeed) const{
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
 * FD - drug force
 * FD - drug force
 * FL - lift force
 * FS - side force
 */
void VtolDynamicsSim::calculateAerodynamics(const Eigen::Vector3d& airspeed,
                                            double dynamicPressure,
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
    double airspeedMod = boost::algorithm::clamp(airspeed.norm(), 5, 40);

    // 1. Calculate aero force
    Eigen::VectorXd polynomialCoeffs(7);

    calculateCLPolynomial(airspeedMod, polynomialCoeffs);
    double CL = polyval(polynomialCoeffs, AoA_deg);

    calculateCSPolynomial(airspeedMod, polynomialCoeffs);
    double CS = polyval(polynomialCoeffs, AoA_deg);

    double CS_rudder = calculateCSRudder(rudder_pos, airspeedMod);
    double CS_beta = calculateCSBeta(AoS_deg, airspeedMod);

    calculateCDPolynomial(airspeedMod, polynomialCoeffs);
    double CD = polyval(polynomialCoeffs.block<5, 1>(0, 0), AoA_deg);

    Eigen::Vector3d FL = (Eigen::Vector3d(0, 1, 0).cross(airspeed.normalized())) * CL;
    Eigen::Vector3d FS = airspeed.cross(Eigen::Vector3d(0, 1, 0).cross(airspeed.normalized())) * (CS + CS_rudder + CS_beta);
    Eigen::Vector3d FD = (-1 * airspeed);
    FD = FD.normalized();
    FD *= CD;
    Faero = 0.5 * dynamicPressure * (FL + FS + FD);

    // 2. Calculate aero moment
    calculateCmxPolynomial(airspeedMod, polynomialCoeffs);
    auto Cmx = polyval(polynomialCoeffs, AoA_deg);

    calculateCmyPolynomial(airspeedMod, polynomialCoeffs);
    auto Cmy = polyval(polynomialCoeffs, AoA_deg);

    calculateCmzPolynomial(airspeedMod, polynomialCoeffs);
    auto Cmz = -polyval(polynomialCoeffs, AoA_deg);

    Cmx_aileron = calculateCmxAileron(aileron_pos, airspeedMod);
    Cmy_elevator = calculateCmyElevator(elevator_pos, airspeedMod);
    Cmx_rudder = calculateCmzRudder(rudder_pos, airspeedMod);

    auto Mx = Cmx + Cmx_aileron * aileron_pos;
    auto My = Cmy + Cmy_elevator * elevator_pos;
    auto Mz = Cmz + Cmx_rudder * rudder_pos;

    Maero = 0.5 * dynamicPressure * params_.characteristicLength * Eigen::Vector3d(Mx, My, Mz);
}

void VtolDynamicsSim::thruster(double actuator, double& thrust, double& torque, double& kf, double& km) const{
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
        if(actuator < 1){
            kf = 0;
            km = 0;
        }else{
            kf = thrust / (actuator * actuator);
            km = torque / (actuator * actuator);
        }
    }
}

void VtolDynamicsSim::calculateNewState(const Eigen::Vector3d& Maero,
                                        const Eigen::Vector3d& Faero,
                                        Eigen::VectorXd actuator,
                                        Time_t dt){
    // control min max

    // update control smoothly

    Eigen::VectorXd thrust(5), torque(5), kf(5), km(5);
    for(size_t idx = 0; idx < 5; idx++){
        thruster(actuator[idx], thrust[idx], torque[idx], kf[idx], km[idx]);
    }

    std::array<Eigen::Vector3d, 5> motorForcesInBodyCS;
    for(size_t idx = 0; idx < 4; idx++){
        motorForcesInBodyCS[idx] << 0, 0, -thrust[idx];
    }
    motorForcesInBodyCS[4] << thrust[4], 0, 0;

    std::array<Eigen::Vector3d, 5> motorTorquesInBodyCS;
    motorTorquesInBodyCS[0] << 0, 0, torque[0];
    motorTorquesInBodyCS[1] << 0, 0, torque[1];
    motorTorquesInBodyCS[2] << 0, 0, -torque[2];
    motorTorquesInBodyCS[3] << 0, 0, -torque[3];
    motorTorquesInBodyCS[4] << -torque[4], 0, 0;

    std::array<Eigen::Vector3d, 5> momentsDueToArmOfForceInBodyCS;
    std::array<Eigen::Vector3d, 5> totalMotorMoments;
    for(size_t idx = 0; idx < 5; idx++){
        momentsDueToArmOfForceInBodyCS[idx] = params_.propellersLocation[idx].cross(motorForcesInBodyCS[idx]);
        totalMotorMoments[idx] = motorTorquesInBodyCS[idx] + momentsDueToArmOfForceInBodyCS[idx];
    }

    auto totalMomentInBodyCS = std::accumulate(&totalMotorMoments[0], &totalMotorMoments[5], Maero);

    state_.angularAccel = params_.inertia.inverse() * (totalMomentInBodyCS - state_.angularVel.cross(params_.inertia * state_.angularVel));
    state_.angularVel += state_.angularAccel * dt;
    Eigen::Quaterniond attitudeDelta = state_.attitude * Eigen::Quaterniond(0, state_.angularVel(0), state_.angularVel(1), state_.angularVel(2));
    attitudeDelta.coeffs() *= 2 * dt;
    state_.attitude.coeffs() += attitudeDelta.coeffs();
    state_.attitude.normalize();
    auto roattionMatrix = state_.attitude.toRotationMatrix();

    auto Fspecific = std::accumulate(&motorForcesInBodyCS[0], &motorForcesInBodyCS[5], Faero);
    auto Ftotal = Fspecific + roattionMatrix * Eigen::Vector3d(0, 0, params_.mass * params_.gravity);

    state_.linearAccel = roattionMatrix.inverse() * Ftotal / params_.mass;
    state_.linearVel += state_.linearAccel * dt;
    state_.position += state_.linearVel * dt;

    // (next step) calculate sensors
}

void VtolDynamicsSim::calculateCLPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    calculatePolynomialUsingTable(tables_.CLPolynomial, airSpeedMod, polynomialCoeffs);
}
void VtolDynamicsSim::calculateCSPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    calculatePolynomialUsingTable(tables_.CSPolynomial, airSpeedMod, polynomialCoeffs);
}
void VtolDynamicsSim::calculateCDPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    calculatePolynomialUsingTable(tables_.CDPolynomial, airSpeedMod, polynomialCoeffs);
}
void VtolDynamicsSim::calculateCmxPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    calculatePolynomialUsingTable(tables_.CmxPolynomial, airSpeedMod, polynomialCoeffs);
}
void VtolDynamicsSim::calculateCmyPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    calculatePolynomialUsingTable(tables_.CmyPolynomial, airSpeedMod, polynomialCoeffs);
}
void VtolDynamicsSim::calculateCmzPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    calculatePolynomialUsingTable(tables_.CmzPolynomial, airSpeedMod, polynomialCoeffs);
}
double VtolDynamicsSim::calculateCSRudder(double rudder_pos, double airspeed) const{
    return griddata(-tables_.actuator, tables_.airspeed, tables_.CS_rudder, rudder_pos, airspeed);
}
double VtolDynamicsSim::calculateCSBeta(double AoS_deg, double airspeed) const{
    return griddata(-(tables_.AoS), tables_.airspeed, tables_.CS_beta, AoS_deg, airspeed);
}
double VtolDynamicsSim::calculateCmxAileron(double aileron_pos, double airspeed) const{
    return griddata(tables_.actuator, tables_.airspeed, tables_.CmxAileron, aileron_pos, airspeed);
}
double VtolDynamicsSim::calculateCmyElevator(double elevator_pos, double airspeed) const{
    return griddata(tables_.actuator, tables_.airspeed, tables_.CmyElevator, elevator_pos, airspeed);
}
double VtolDynamicsSim::calculateCmzRudder(double rudder_pos, double airspeed) const{
    return griddata(tables_.actuator, tables_.airspeed, tables_.CmzRudder, rudder_pos, airspeed);
}

void VtolDynamicsSim::calculatePolynomialUsingTable(const Eigen::MatrixXd& table,
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
size_t VtolDynamicsSim::search(const Eigen::MatrixXd& matrix, double key) const{
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
size_t VtolDynamicsSim::findRow(const Eigen::MatrixXd& table, double value) const{
    size_t row;
    size_t c = table.rows();
    while(row + 2 < c && table(row + 1, 0) < value){
        row++;
    }
    return row;
}

double VtolDynamicsSim::lerp(double a, double b, double f) const{
    return a + f * (b - a);
}

double VtolDynamicsSim::griddata(const Eigen::MatrixXd& x,
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

double VtolDynamicsSim::polyval(const Eigen::VectorXd& poly, double val) const{
    double result = 0;
    for(uint8_t idx = 0; idx < poly.rows(); idx++){
        result += poly[idx] * std::pow(val, poly.rows() - 1 - idx);
    }
    return result;
}

void VtolDynamicsSim::setWindParameter(Eigen::Vector3d windMeanVelocity,
                                       double windVariance){
    state_.windVelocity = windMeanVelocity;
    state_.windVariance = windVariance;
}

void VtolDynamicsSim::setEulerAngles(Eigen::Vector3d eulerAngles){
    state_.eulerAngles = eulerAngles;
}

Eigen::Vector3d VtolDynamicsSim::getAngularAcceleration() const{
    return state_.angularAccel;
}
Eigen::Vector3d VtolDynamicsSim::getAngularVelocity() const{
    return state_.angularVel;
}
Eigen::Quaterniond VtolDynamicsSim::getAttitude() const{
    return state_.attitude;
}
Eigen::Vector3d VtolDynamicsSim::getLinearAcceleration() const{
    return state_.linearAccel;
}
Eigen::Vector3d VtolDynamicsSim::getLinearVelocity() const{
    return state_.linearVel;
}
Eigen::Vector3d VtolDynamicsSim::getPosition() const{
    return state_.position;
}
