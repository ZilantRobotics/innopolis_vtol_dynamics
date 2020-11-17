/**
 * @file uavDynamicsSimBase.hpp
 * @author ponomarevda96@gmail.com
 * @brief Header file for UAV dynamics
 */

#ifndef UAV_DYNAMICS_SIM_BASE_HPP
#define UAV_DYNAMICS_SIM_BASE_HPP

#include <Eigen/Geometry>
#include <vector>


class UavDynamicsSimBase{
public:
    UavDynamicsSimBase() {};

    /**
     * @brief Use rosparam here to initialize sim 
     * @return -1 if error occures and simulation can't start
     */
    virtual int8_t init() = 0;

    virtual void process(double dt_secs, const std::vector<double> & motorSpeedCommandIn, bool isCmdPercent) = 0;

    virtual Eigen::Vector3d getVehiclePosition() const = 0;
    virtual Eigen::Quaterniond getVehicleAttitude() const = 0;
    virtual Eigen::Vector3d getVehicleVelocity(void) const = 0;
    virtual Eigen::Vector3d getVehicleAngularVelocity(void) const = 0;
    virtual void getIMUMeasurement(Eigen::Vector3d & accOutput, Eigen::Vector3d & gyroOutput) const = 0;
    virtual void enu2Geodetic(double east, double north, double up,
                              double *latitude, double *longitude, double *altitude) = 0;
};


#endif  // UAV_DYNAMICS_SIM_BASE_HPP
