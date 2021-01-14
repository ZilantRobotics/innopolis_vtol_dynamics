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
 * NED_px4 = q_enu_to_ned * ENU_ros
 * NED_ros = q_enu_to_ned.inverse() * ENU_px4
 */
auto q_enu_to_ned = Eigen::Quaterniond(0, 0.70711, 0.70711, 0);

/**
 * @brief Quaternion for rotation between body FLU and body FRD frames
 *
 * +PI rotation around X (Forward) axis rotates from Forward, Right, Down (aircraft)
 * to Forward, Left, Up (base_link) frames and vice-versa.
 * @note Example:
 * FRD_px4 = q_frd_flu * FLU_ros
 * FRD_ros = q_frd_flu * FLU_px4
 */
const auto q_frd_flu = Eigen::Quaterniond(0, 1, 0, 0);


/**
 * @note getVehiclePosition and getVehicleVelocity
 */
Eigen::Vector3d nedToEnu(Eigen::Vector3d ned){
    return q_enu_to_ned.inverse() * ned;
}

/**
 * @note getVehicleVelocity
 */
Eigen::Vector3d enuToNed(Eigen::Vector3d enu){
    return q_enu_to_ned * enu;
}


/**
 * @note angularVelocity and specificForce (vtolDynamics)
 */
Eigen::Vector3d frdToFlu(Eigen::Vector3d frd){
    return q_frd_flu * frd;
}

/**
 * @note angularVelocity and specificForce (sendHilSensor)
 */
Eigen::Vector3d fluToFrd(Eigen::Vector3d flu){
    return q_frd_flu * flu;
}

/**
 * @note linearVelocity
 */
Eigen::Vector3d enuToFrd(const Eigen::Vector3d& vel_enu, const Eigen::Quaterniond& q_flu_to_enu){
    return q_frd_flu * q_flu_to_enu.inverse() * vel_enu;
}

/**
 * @note getVehicleAttitude (vtolDynamics)
 */
Eigen::Quaterniond frdNedTofluEnu(Eigen::Quaterniond q_frd_to_ned){
    return q_enu_to_ned.inverse() * q_frd_flu * q_frd_to_ned;
}

}

#endif  // SC_CONVERTER_HPP
