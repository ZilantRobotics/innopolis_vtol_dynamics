/**
 * @file multicopterDynamicsSimWrapper.cpp
 * @author ponomarevda96@gmail.com
 */

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include "flightgogglesDynamicsSim.hpp"


static const std::string MULTICOPTER_PARAMS_NS = "/uav/multicopter_params/";
template <class T>
static void getParameter(std::string name, T& parameter, T default_value, std::string unit=""){
  if (!ros::param::get(MULTICOPTER_PARAMS_NS + name, parameter)){
    std::cout << "Did not get "
              << name
              << " from the params, defaulting to "
              << default_value
              << " "
              << unit
              << std::endl;
    parameter = default_value;
  }
}

FlightgogglesDynamics::FlightgogglesDynamics(){

}

int8_t FlightgogglesDynamics::init(){
    // Vehicle parameters
    double vehicleMass, motorTimeconstant, motorRotationalInertia,
            thrustCoeff, torqueCoeff, dragCoeff;
    getParameter("vehicle_mass",              vehicleMass,                1.,       "kg");
    getParameter("motor_time_constant",       motorTimeconstant,          0.02,     "sec");
    getParameter("motor_rotational_inertia",  motorRotationalInertia,     6.62e-6,  "kg m^2");
    getParameter("thrust_coefficient",        thrustCoeff,                1.91e-6,  "N/(rad/s)^2");
    getParameter("torque_coefficient",        torqueCoeff,                2.6e-7,   "Nm/(rad/s)^2");
    getParameter("drag_coefficient",          dragCoeff,                  0.1,      "N/(m/s)");

    Eigen::Matrix3d aeroMomentCoefficient = Eigen::Matrix3d::Zero();
    getParameter("aeromoment_coefficient_xx", aeroMomentCoefficient(0,0), 0.003,    "Nm/(rad/s)^2");
    getParameter("aeromoment_coefficient_yy", aeroMomentCoefficient(1,1), 0.003,    "Nm/(rad/s)^2");
    getParameter("aeromoment_coefficient_zz", aeroMomentCoefficient(2,2), 0.003,    "Nm/(rad/s)^2");

    Eigen::Matrix3d vehicleInertia = Eigen::Matrix3d::Zero();
    getParameter("vehicle_inertia_xx",        vehicleInertia(0,0),        0.0049,   "kg m^2");
    getParameter("vehicle_inertia_yy",        vehicleInertia(1,1),        0.0049,   "kg m^2");
    getParameter("vehicle_inertia_zz",        vehicleInertia(2,2),        0.0069,   "kg m^2");
    
    double minPropSpeed, maxPropSpeed, momentProcessNoiseAutoCorrelation, forceProcessNoiseAutoCorrelation;
    minPropSpeed = 0.0;
    getParameter("max_prop_speed",            maxPropSpeed,               2200.0,   "rad/s");
    getParameter("moment_process_noise",momentProcessNoiseAutoCorrelation,1.25e-7,  "(Nm)^2 s");
    getParameter("force_process_noise", forceProcessNoiseAutoCorrelation, 0.0005,   "N^2 s");


    // Set gravity vector according to ROS reference axis system, see header file
    Eigen::Vector3d gravity(0.,0.,-9.81);


    // Create quadcopter simulator
    multicopterSim_ = new MulticopterDynamicsSim(4, thrustCoeff, torqueCoeff,
                        minPropSpeed, maxPropSpeed, motorTimeconstant, motorRotationalInertia,
                        vehicleMass, vehicleInertia,
                        aeroMomentCoefficient, dragCoeff, momentProcessNoiseAutoCorrelation,
                        forceProcessNoiseAutoCorrelation, gravity);

    double initPropSpeed = sqrt(vehicleMass/4.*9.81/thrustCoeff);
    multicopterSim_->setMotorSpeed(initPropSpeed);


    // Get and set IMU parameters
    double accBiasProcessNoiseAutoCorrelation, gyroBiasProcessNoiseAutoCorrelation,
           accBiasInitVar, gyroBiasInitVar,
           accMeasNoiseVariance, gyroMeasNoiseVariance;
    getParameter("accelerometer_biasprocess", accBiasProcessNoiseAutoCorrelation, 1.0e-7, "m^2/s^5");
    getParameter("gyroscope_biasprocess",     accBiasProcessNoiseAutoCorrelation, 1.0e-7, "rad^2/s^3");
    getParameter("accelerometer_biasinitvar", accBiasInitVar,                     0.005,  "(m/s^2)^2");
    getParameter("gyroscope_biasinitvar",     gyroBiasInitVar,                    0.003,  "(rad/s)^2");
    getParameter("accelerometer_variance",    accMeasNoiseVariance,               0.005,  "m^2/s^4");
    getParameter("gyroscope_variance",        gyroMeasNoiseVariance,              0.003,  "rad^2/s^2");
    multicopterSim_->imu_.setBias(accBiasInitVar, gyroBiasInitVar,
                                  accBiasProcessNoiseAutoCorrelation, gyroBiasProcessNoiseAutoCorrelation);
    multicopterSim_->imu_.setNoiseVariance(accMeasNoiseVariance, gyroMeasNoiseVariance);

    return 0;
}

void FlightgogglesDynamics::setInitialPosition(const Eigen::Vector3d & position,
                                               const Eigen::Quaterniond& attitude){
    multicopterSim_->setVehiclePosition(position, attitude);
}

void FlightgogglesDynamics::process(double dt_secs,
                                    const std::vector<double> & motorSpeedCommandIn,
                                    bool isCmdPercent){
    auto actuators = mapCmdActuator(motorSpeedCommandIn);
    multicopterSim_->proceedState_ExplicitEuler(dt_secs, actuators, isCmdPercent);
}

Eigen::Vector3d FlightgogglesDynamics::getVehiclePosition() const{
    return multicopterSim_->getVehiclePosition();
}
Eigen::Quaterniond FlightgogglesDynamics::getVehicleAttitude() const{
    return multicopterSim_->getVehicleAttitude();
}
Eigen::Vector3d FlightgogglesDynamics::getVehicleVelocity(void) const{
    return multicopterSim_->getVehicleVelocity();
}
Eigen::Vector3d FlightgogglesDynamics::getVehicleAngularVelocity(void) const{
    return multicopterSim_->getVehicleAngularVelocity();
}
void FlightgogglesDynamics::getIMUMeasurement(Eigen::Vector3d & accOutput, Eigen::Vector3d & gyroOutput){
    return multicopterSim_->getIMUMeasurement(accOutput, gyroOutput);
}

std::vector<double> FlightgogglesDynamics::mapCmdActuator(std::vector<double> initialCmd) const{
    std::vector<double> mappedCmd;
    mappedCmd.push_back(initialCmd[2]);     // PX4: motor 3, front left
    mappedCmd.push_back(initialCmd[1]);     // PX4: motor 2, tail left
    mappedCmd.push_back(initialCmd[3]);     // PX4: motor 4, tail right
    mappedCmd.push_back(initialCmd[0]);     // PX4: motor 1, front right
    return mappedCmd;
}
