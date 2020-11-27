/**
 * @file flightgogglesDynamicsSim.hpp
 * @author ponomarevda96@gmail.com
 */

#ifndef MULTICOPTER_DYNAMICS_WRAPPER_BASE_HPP
#define MULTICOPTER_DYNAMICS_WRAPPER_BASE_HPP

#include "uavDynamicsSimBase.hpp"
#include "../libs/multicopterDynamicsSim/multicopterDynamicsSim.hpp"

class FlightgogglesDynamics: public UavDynamicsSimBase{
public:
    FlightgogglesDynamics();

    virtual int8_t init();
    virtual void initStaticMotorTransform() override;

    virtual void setReferencePosition(double latRef, double lonRef, double altRef) override;
    virtual void setInitialPosition(const Eigen::Vector3d & position,
                                    const Eigen::Quaterniond& attitude) override;

    virtual void process(double dt_secs, const std::vector<double> & motorSpeedCommandIn, bool isCmdPercent);

    virtual Eigen::Vector3d getVehiclePosition() const;
    virtual Eigen::Quaterniond getVehicleAttitude() const;
    virtual Eigen::Vector3d getVehicleVelocity(void) const;
    virtual Eigen::Vector3d getVehicleAngularVelocity(void) const;
    virtual void getIMUMeasurement(Eigen::Vector3d & accOutput, Eigen::Vector3d & gyroOutput);
    virtual void enu2Geodetic(double east, double north, double up,
                              double *latitude, double *longitude, double *altitude);

private:
    MulticopterDynamicsSim * multicopterSim_;
};

#endif  // MULTICOPTER_DYNAMICS_WRAPPER_BASE_HPP