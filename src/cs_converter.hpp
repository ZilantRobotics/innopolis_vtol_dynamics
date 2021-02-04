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

/**
 * @brief Quaternion for rotation between ENU and NED frames
 *
 * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
 * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
 * @note Example:
 * NED_px4 = Q_ENU_TO_NED * ENU_ros
 * NED_ros = Q_ENU_TO_NED.inverse() * ENU_px4
 */
const auto Q_ENU_TO_NED = Eigen::Quaterniond(0, 0.70711, 0.70711, 0);

/**
 * @brief Quaternion for rotation between body FLU and body FRD frames
 *
 * +PI rotation around X (Forward) axis rotates from Forward, Right, Down (aircraft)
 * to Forward, Left, Up (base_link) frames and vice-versa.
 * @note Example:
 * FRD_px4 = Q_FRD_FLU * FLU_ros
 * FRD_ros = Q_FRD_FLU * FLU_px4
 */
const auto Q_FRD_FLU = Eigen::Quaterniond(0, 1, 0, 0);


/**
 * @note getVehiclePosition and getVehicleVelocity
 */
Eigen::Vector3d nedToEnu(Eigen::Vector3d ned){
    return Q_ENU_TO_NED.inverse() * ned;
}

/**
 * @note getVehicleVelocity
 */
Eigen::Vector3d enuToNed(Eigen::Vector3d enu){
    return Q_ENU_TO_NED * enu;
}


/**
 * @note angularVelocity and specificForce (vtolDynamics)
 */
Eigen::Vector3d frdToFlu(Eigen::Vector3d frd){
    return Q_FRD_FLU * frd;
}

/**
 * @note angularVelocity and specificForce (sendHilSensor)
 */
Eigen::Vector3d fluToFrd(Eigen::Vector3d flu){
    return Q_FRD_FLU * flu;
}

/**
 * @note linearVelocity
 */
Eigen::Vector3d enuToFrd(const Eigen::Vector3d& vel_enu, const Eigen::Quaterniond& q_flu_to_enu){
    return Q_FRD_FLU * q_flu_to_enu.inverse() * vel_enu;
}

Eigen::Vector3d nedToFrd(const Eigen::Vector3d& vel_ned, const Eigen::Quaterniond& q_flu_to_enu){
    return Q_FRD_FLU * q_flu_to_enu.inverse() * (Q_ENU_TO_NED.inverse() * vel_ned);
}

/**
 * @note getVehicleAttitude (vtolDynamics)
 */
Eigen::Quaterniond frdNedTofluEnu(Eigen::Quaterniond q_frd_to_ned){
    return Q_ENU_TO_NED.inverse() * Q_FRD_FLU * q_frd_to_ned;
}

Eigen::Quaterniond fluEnuToFrdNed(Eigen::Quaterniond q_flu_to_enu){
    return Q_ENU_TO_NED * Q_FRD_FLU * q_flu_to_enu;
}

}

#endif  // SC_CONVERTER_HPP
