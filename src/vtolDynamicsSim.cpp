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

void VtolDynamicsSim::processStep(){
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

void VtolDynamicsSim::setWindParameter(Eigen::Vector3d windMeanVelocity,
                                       double windVariance){
    sysState_.windVelocity = windMeanVelocity;
    sysState_.windVariance = windVariance;
}

void VtolDynamicsSim::setEulerAngles(Eigen::Vector3d eulerAngles){
    sysState_.eulerAngles = eulerAngles;
}