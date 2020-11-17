/**
 * @file multicopterDynamicsSimWrapper.hpp
 * @author ponomarevda96@gmail.com
 */

#ifndef MULTICOPTER_DYNAMICS_WRAPPER_BASE_HPP
#define MULTICOPTER_DYNAMICS_WRAPPER_BASE_HPP

#include <tf2_ros/static_transform_broadcaster.h>
#include "uavDynamicsSimBase.hpp"
#include "../libs/multicopterDynamicsSim/multicopterDynamicsSim.hpp"

class MulticopterDynamicsWrapper: public UavDynamicsSimBase{
public:
    MulticopterDynamicsWrapper();

    virtual int8_t init();

    virtual void process(double dt_secs, const std::vector<double> & motorSpeedCommandIn, bool isCmdPercent);

    virtual Eigen::Vector3d getVehiclePosition() const;
    virtual Eigen::Quaterniond getVehicleAttitude() const;
    virtual Eigen::Vector3d getVehicleVelocity(void) const;
    virtual Eigen::Vector3d getVehicleAngularVelocity(void) const;
    virtual void getIMUMeasurement(Eigen::Vector3d & accOutput, Eigen::Vector3d & gyroOutput) const;
    virtual void enu2Geodetic(double east, double north, double up, double *latitude, double *longitude, double *altitude);

private:
    void publishStaticMotorTransform(const ros::Time & timeStamp,
                                     const char * frame_id,
                                     const char * child_frame_id,
                                     const Eigen::Isometry3d & motorFrame);
    MulticopterDynamicsSim * multicopterSim_;
    tf2_ros::StaticTransformBroadcaster staticTfPub_;
};

#endif  // MULTICOPTER_DYNAMICS_WRAPPER_BASE_HPP