/**
 * @file vtolDynamicsSim.cpp
 * @author ponomarevda96@gmail.com
 * @brief Vtol dynamics simulator class implementation
 * 
 */
#include <iostream>
#include <chrono>
#include <cmath>
#include <boost/algorithm/clamp.hpp>
#include <algorithm>
#include "yaml-cpp/yaml.h"
#include "vtolDynamicsSim.hpp"
#include <ros/package.h>

VtolDynamicsSim::VtolDynamicsSim():
    distribution_(0.0, 1.0){
}

void VtolDynamicsSim::init(){
    std::string configPath = ros::package::getPath("innopolis_vtol_dynamics") + "/config/";
    loadTables(configPath + "aerodynamics_coeffs.yaml");
    loadParams(configPath + "vtol_params.yaml");
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

    vectorTable = config["airspeed_table"].as< std::vector<double> >();
    tables_.airspeed = Eigen::Map<Eigen::Matrix<double, 1, 8, Eigen::RowMajor>>((double*)&vectorTable[0], 1, 8);

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
}

void VtolDynamicsSim::loadParams(const std::string& path){
    YAML::Node config = YAML::LoadFile(path);
    params_.mass = config["mass"].as<double>();
    params_.gravity = config["gravity"].as<double>();
    params_.atmoRho = config["atmoRho"].as<double>();
    params_.wingArea = config["wingArea"].as<double>();
    params_.characteristicLength = config["characteristicLength"].as<double>();
}

void VtolDynamicsSim::processStep(double dt_secs,
                                  const std::vector<double> & motorSpeedCommandIn,
                                  bool isCmdPercent){
    Eigen::Vector3d vel_w = calculateWind();
    Eigen::Matrix3d rotationMatrix = calculateRotationMatrix();
    Eigen::Vector3d airSpeed = calculateAirSpeed(rotationMatrix, sysState_.estLinearVelocity, vel_w);
    double airSpeedMod = airSpeed.norm();
    double dynPressure = calculateDynamicPressure(airSpeedMod);
    double AoA = calculateAnglesOfAtack(airSpeed);
    double AoS = calculateAnglesOfSideslip(airSpeed);
    // [Faero, Maero, Cmx_a, Cmy_e, Cmz_r, AoA_min, AoA_max] = aerodynamics(V_mod, Va_b, dyn_press, AoA, AoS, atmo_rho, l, u_0(6), u_0(7), u_0(8));

}

Eigen::Vector3d VtolDynamicsSim::calculateWind(){
    Eigen::Vector3d wind;
    wind[0] = sqrt(sysState_.windVariance) * distribution_(generator_) + sysState_.windVelocity[0];
    wind[1] = sqrt(sysState_.windVariance) * distribution_(generator_) + sysState_.windVelocity[1];
    wind[2] = sqrt(sysState_.windVariance) * distribution_(generator_) + sysState_.windVelocity[2];

    /**
     * @todo Implement own gust logic
     * @note innopolis_vtol_indi logic doesn't suit us
     */
    Eigen::Vector3d gust;
    gust.setZero();

    return wind + gust;
}

Eigen::Matrix3d VtolDynamicsSim::calculateRotationMatrix() const{
    double phi = sysState_.eulerAngles[0];
    double theta = sysState_.eulerAngles[1];
    double psi = sysState_.eulerAngles[2];
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

// FD - drug force
// FL - lift force
// FS - side force
// CD ?
// CL ?
void VtolDynamicsSim::calculateAerodynamics(const Eigen::Vector3d& airspeed,
                                            double dynamicPressure,
                                            double AoA,
                                            double AoS,
                                            double aileron_pos,
                                            double elevator_pos,
                                            double rudder_pos,
                                            Eigen::Vector3d& Faero,
                                            Eigen::Vector3d& Maero,
                                            double& Cmx_a,
                                            double& Cmy_e,
                                            double& Cmz_r){
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

    Eigen::Vector3d FL = (Eigen::Vector3d(0, 1, 0).cross(airspeed.normalized()));
    FL = FL * CL;
    Eigen::Vector3d FS = airspeed.cross(Eigen::Vector3d(0, 1, 0).cross(airspeed.normalized())) * (CS + CS_rudder + CS_beta);
    Eigen::Vector3d FD = (-1 * airspeed);
    FD = FD.normalized();
    FD *= CD;
    Faero = 0.5 * dynamicPressure * (FL + FS + FD);
}

void VtolDynamicsSim::calculateCLPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    if(tables_.CLPolynomial.size() != 64){
        return;
    }
    calculatePolynomialUsingTable(tables_.CLPolynomial, airSpeedMod, polynomialCoeffs);
}

void VtolDynamicsSim::calculateCSPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    if(tables_.CSPolynomial.size() != 64){
        return;
    }
    calculatePolynomialUsingTable(tables_.CSPolynomial, airSpeedMod, polynomialCoeffs);
}

void VtolDynamicsSim::calculateCDPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    if(tables_.CDPolynomial.size() != 48){
        std::cerr << "WRONG CD SIZE! Actual size is " << tables_.CDPolynomial.size() << std::endl;
        return;
    }
    calculatePolynomialUsingTable(tables_.CDPolynomial, airSpeedMod, polynomialCoeffs);
}

void VtolDynamicsSim::calculateCmxPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    if(tables_.CmxPolynomial.size() != 64){
        return;
    }
    calculatePolynomialUsingTable(tables_.CmxPolynomial, airSpeedMod, polynomialCoeffs);
}

void VtolDynamicsSim::calculateCmyPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    if(tables_.CmyPolynomial.size() != 64){
        return;
    }
    calculatePolynomialUsingTable(tables_.CmyPolynomial, airSpeedMod, polynomialCoeffs);
}

void VtolDynamicsSim::calculateCmzPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    if(tables_.CmzPolynomial.size() != 64){
        return;
    }
    calculatePolynomialUsingTable(tables_.CmzPolynomial, airSpeedMod, polynomialCoeffs);
}


double VtolDynamicsSim::calculateCSRudder(double rudder_pos, double airspeed) const{
    return griddata(-tables_.actuator.transpose(), tables_.airspeed.transpose(), tables_.CS_rudder, rudder_pos, airspeed);
}

double VtolDynamicsSim::calculateCSBeta(double AoS_deg, double airspeed) const{
    auto result = griddata(-(tables_.AoS), tables_.airspeed, tables_.CS_beta, AoS_deg, airspeed);
    return result;
}


void VtolDynamicsSim::calculatePolynomialUsingTable(const Eigen::MatrixXd& table,
                                                    double airSpeedMod,
                                                    Eigen::VectorXd& polynomialCoeffs){
    size_t prevRowIdx = findRowForPolynomial(table, airSpeedMod);
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

// size should be greater or equel than 2
size_t VtolDynamicsSim::search(const Eigen::MatrixXd& matrix, double key) const{
    size_t row_idx;
    // increase
    if(matrix(matrix.rows() - 1, 0) > matrix(0, 0)){
        for(row_idx = 1; row_idx < matrix.rows() - 1; row_idx++){
            if(key <= matrix(row_idx, 0)){
                break;
            }
        }
        row_idx--;
    }
    // decrease
    else{
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
size_t VtolDynamicsSim::findRowForPolynomial(const Eigen::MatrixXd& table, double value) const{
    size_t row;
    for(row = 0; row < table.rows() - 2; row++){
        if(value <= table((row + 1), 0)){
            break;
        }
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
    sysState_.windVelocity = windMeanVelocity;
    sysState_.windVariance = windVariance;
}

void VtolDynamicsSim::setEulerAngles(Eigen::Vector3d eulerAngles){
    sysState_.eulerAngles = eulerAngles;
}