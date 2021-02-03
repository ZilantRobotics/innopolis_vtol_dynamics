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

    virtual int8_t init() override;
    virtual void initStaticMotorTransform() override;

    virtual void setInitialPosition(const Eigen::Vector3d & position,
                                    const Eigen::Quaterniond& attitude) override;

    virtual void process(double dt_secs, const std::vector<double> & motorSpeedCommandIn, bool isCmdPercent);

    virtual Eigen::Vector3d getVehiclePosition() const;
    virtual Eigen::Quaterniond getVehicleAttitude() const;
    virtual Eigen::Vector3d getVehicleVelocity(void) const;
    virtual Eigen::Vector3d getVehicleAngularVelocity(void) const;
    virtual void getIMUMeasurement(Eigen::Vector3d & accOutput, Eigen::Vector3d & gyroOutput);

private:
    MulticopterDynamicsSim * multicopterSim_;

    /**
     * @brief Convert actuator indexes from PX4 notation to internal Flightgoggles notation
     * @param cmd with indexes: 0 - front right, 1 - tail left, 2 - front left, 3 - tail right
     * @return cmd with indexes: 0 - front left, 1 - tail left, 2 - tail right, 3 - front right
     */
    std::vector<double> mapCmdActuator(std::vector<double> cmd) const;
};

#endif  // MULTICOPTER_DYNAMICS_WRAPPER_BASE_HPP