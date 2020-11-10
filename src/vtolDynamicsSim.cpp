/**
 * @file vtolDynamicsSim.cpp
 * @author ponomarevda96@gmail.com
 * @brief Vtol dynamics simulator class implementation
 * 
 */
#include "vtolDynamicsSim.hpp"
#include <iostream>
#include <chrono>
#include <cmath>

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
    // calculate_airspeed();
    // Va_b = calculate true airspeed (abs)
    // dyn_press = atmo_rho*V_mod^2*S; % calculate velocity pressure
    // [AoA, AoS] = calculate_AoA_AoS(Va_b); % calculate AoA and AoS
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

Eigen::Matrix3d VtolDynamicsSim::calculateRotationMatrix(){
    double phi = sysState_.eulerAngles[0];
    double theta = sysState_.eulerAngles[1];
    double psi = sysState_.eulerAngles[2];
    Eigen::Quaterniond q_phi(std::cos(phi/2), std::sin(phi/2), 0, 0);
    Eigen::Quaterniond q_theta(std::cos(theta/2), 0, std::sin(theta/2), 0);
    Eigen::Quaterniond q_psi(std::cos(psi/2), 0, 0, std::sin(psi/2));
    Eigen::Quaterniond q = q_psi * q_theta * q_phi;
    return q.toRotationMatrix();
}

void VtolDynamicsSim::setWindParameter(Eigen::Vector3d windMeanVelocity,
                      double windVariance){
    sysState_.windVelocity = windMeanVelocity;
    sysState_.windVariance = windVariance;
}

void VtolDynamicsSim::setEulerAngles(Eigen::Vector3d eulerAngles){
    sysState_.eulerAngles = eulerAngles;
}