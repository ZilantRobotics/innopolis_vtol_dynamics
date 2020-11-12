/**
 * @file vtolDynamicsSim.cpp
 * @author ponomarevda96@gmail.com
 * @brief Vtol dynamics simulator class implementation
 * 
 */
#include <iostream>
// #include <algorithm>
#include <chrono>
#include <cmath>
#include <boost/algorithm/clamp.hpp>

#include "vtolDynamicsSim.hpp"


VtolDynamicsSim::VtolDynamicsSim():
    distribution_(0.0, 1.0){
}

void VtolDynamicsSim::init(){
    /**
     * @todo Load parameters from json file
     */
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
    double AoA_deg = boost::algorithm::clamp(AoA * 180 / 3.1415, -45.0, +45.0);
    double AoS_deg = boost::algorithm::clamp(AoS * 180 / 3.1415, -90.0, +90.0);
    double airspeedMod = boost::algorithm::clamp(airspeed.norm(), 5, 40);

    Eigen::VectorXd polynomialCoeffs;
    calculateCLPolynomial(airspeedMod, polynomialCoeffs);
    double CL = polyval(polynomialCoeffs, airspeedMod);

}

void VtolDynamicsSim::calculateCLPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    Eigen::MatrixXf table(8, 8);
    table << 5, -2.758e-11, 8.139e-09, 1.438e-07, -3.095e-05, -0.0003512, 0.05557, 0.4132,
            10, -3.934e-11, 8.204e-09, 1.935e-07, -3.075e-05, -0.0004209, 0.0552,  0.4438,
            15, -5.464e-11, 7.747e-09, 2.369e-07, -2.918e-05, -0.0004564, 0.05447, 0.4545,
            20, -5.087e-11, 7.803e-09, 2.267e-07, -2.926e-05, -0.0004493, 0.05435, 0.4525,
            25, -5.489e-11, 7.949e-09, 2.428e-07, -2.975e-05, -0.0004656, 0.05472, 0.4578,
            30, -4.749e-11, 7.778e-09, 2.219e-07, -2.926e-05, -0.0004567, 0.05433, 0.4599,
            35, -5.911e-11, 7.879e-09, 2.574e-07, -2.961e-05, -0.0004838, 0.05458, 0.4637,
            40, -5.911e-11, 7.879e-09, 2.574e-07, -2.961e-05, -0.0004838, 0.05458, 0.4637;
    calculatePolynomialUsingTable(table, airSpeedMod, polynomialCoeffs);
}

void VtolDynamicsSim::calculateCDPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs){
    Eigen::MatrixXf table(8, 8);
    table << 5,  -1.064e-07, -4.398e-07, 0.0007783, 0.003057, 0.0852,
             10, -1.006e-07, -4.922e-07, 0.0007733, 0.003133, 0.07989,
             15, -9.475e-08, -5.335e-07, 0.0007758, 0.003196, 0.07643,
             20, -9.277e-08, -4.929e-07, 0.0007704, 0.003136, 0.076,
             25, -9.419e-08, -4.522e-07, 0.000773,  0.003132, 0.07486,
             30, -9.502e-08, -5.321e-07, 0.0007726, 0.003174, 0.07427,
             35, -9.697e-08, -5.084e-07, 0.0007757, 0.003149, 0.07311,
             40, -9.697e-08, -5.084e-07, 0.0007757, 0.003149, 0.07311,
    calculatePolynomialUsingTable(table, airSpeedMod, polynomialCoeffs);
}

void VtolDynamicsSim::calculatePolynomialUsingTable(const Eigen::MatrixXf& table,
                                                    double airSpeedMod,
                                                    Eigen::VectorXd& polynomialCoeffs){
    size_t prevRowIdx = findRow(table, airSpeedMod);
    if(prevRowIdx + 2 <= table.rows()){
        size_t nextRowIdx = prevRowIdx + 1;
        Eigen::MatrixXf prevRow = table.row(prevRowIdx);
        Eigen::MatrixXf nextRow = table.row(nextRowIdx);
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

size_t VtolDynamicsSim::findRow(const Eigen::MatrixXf& table, double value) const{
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