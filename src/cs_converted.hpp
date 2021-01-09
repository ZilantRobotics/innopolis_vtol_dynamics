/**
 * @file sc_converter.hpp
 * @author ponomarevda96@gmail.com
 * @brief Set of convertion method from NED to ENU, from FRD to FLU and vice versa
 */

#ifndef SC_CONVERTER_H
#define SC_CONVERTER_H

#include <Eigen/Geometry>

namespace Converter
{

static const auto q_ned_enu = Eigen::Quaterniond(0, 0.70711, 0.70711, 0);
static const auto q_frd_flu = Eigen::Quaterniond(0, 1, 0, 0);

/**
 * @note getVehiclePosition and getVehicleVelocity
 */
Eigen::Vector3d nedToEnu(Eigen::Vector3d ned){
    return q_ned_enu * ned;
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

}

#endif  // SC_CONVERTER_H
