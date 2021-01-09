/**
 * @file cs_converter.hpp
 * @author ponomarevda96@gmail.com
 * @brief Set of convertion method from NED to ENU, from FRD to FLU and vice versa
 */

#ifndef SC_CONVERTER_HPP
#define SC_CONVERTER_HPP

#include <Eigen/Geometry>

namespace Converter
{

// /**
//  * @brief Quaternion for rotation between ENU and NED frames
//  *
//  * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
//  * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
//  * @note Example:
//  * NED_px4 = q_ng * ENU_ros
//  */
const auto q_ned_enu = Eigen::Quaterniond(0, 0.70711, 0.70711, 0);

// /**
//  * @brief Quaternion for rotation between body FLU and body FRD frames
//  *
//  * +PI rotation around X (Forward) axis rotates from Forward, Right, Down (aircraft)
//  * to Forward, Left, Up (base_link) frames and vice-versa.
//  * @note Example:
//  * FRD_px4(b) = q_br * FLU_ros(r)
//  */
const auto q_frd_flu = Eigen::Quaterniond(0, 1, 0, 0);


/**
 * @note getVehiclePosition and getVehicleVelocity
 */
Eigen::Vector3d nedToEnu(Eigen::Vector3d ned){
    return q_ned_enu * ned;
}

/**
 * @note getVehicleVelocity
 */
Eigen::Vector3d enuToNed(Eigen::Vector3d enu){
    return q_ned_enu * enu;
}

/**
 * @note getVehicleAttitude
 */
Eigen::Quaterniond frdToFlu(Eigen::Quaterniond frd){
    return q_frd_flu * frd;
}

/**
 * @note angularVelocity and specificForce
 */
Eigen::Vector3d fluToFrd(Eigen::Vector3d flu){
    return q_frd_flu * flu;
}

/**
 * @note linearVelocity
 */
Eigen::Vector3d enuToFrd(const Eigen::Vector3d& vel_enu, const Eigen::Quaterniond& q_enu_flu){
    return q_frd_flu * q_enu_flu.inverse() * vel_enu;
}

}

#endif  // SC_CONVERTER_HPP
