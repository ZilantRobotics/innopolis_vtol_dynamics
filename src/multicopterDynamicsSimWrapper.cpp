/**
 * @file multicopterDynamicsSimWrapper.cpp
 * @author ponomarevda96@gmail.com
 */

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include "multicopterDynamicsSimWrapper.hpp"


static const std::string INNO_DYNAMICS_NS = "/uav/innopolis_vtol_dynamics/";
template <class T>
static void getParameter(std::string name, T& parameter, T default_value, std::string unit=""){
  if (!ros::param::get(INNO_DYNAMICS_NS + name, parameter)){
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

MulticopterDynamicsWrapper::MulticopterDynamicsWrapper(){

}

int8_t MulticopterDynamicsWrapper::init(){
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

    std::vector<double> initPose(7);
    if (!ros::param::get(INNO_DYNAMICS_NS + "init_pose", initPose)) {
        std::cout << "Did NOT find initial pose from param file" << std::endl;
        initPose.at(2) = 0.2;
        initPose.at(6) = 1.0;
    }

    Eigen::Vector3d initPosition;
    initPosition << initPose.at(0), initPose.at(1), initPose.at(2);

    Eigen::Quaterniond initAttitude;
    initAttitude.x() = initPose.at(3);
    initAttitude.y() = initPose.at(4);
    initAttitude.z() = initPose.at(5);
    initAttitude.w() = initPose.at(6);
    initAttitude.normalize();

    double initPropSpeed = sqrt(vehicleMass/4.*9.81/thrustCoeff);

    multicopterSim_->setVehiclePosition(initPosition, initAttitude);
    multicopterSim_->setVehicleInitialAttitude(initAttitude);
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

    // Get home (reference) position
    double latRef, lonRef, altRef;
    getParameter("lat_ref",                   latRef,                            47.3977420);
    getParameter("lon_ref",                   lonRef,                            8.5455940);
    getParameter("alt_ref",                   altRef,                            488.157);

    multicopterSim_->geodetic_converter_.initialiseReference(latRef, lonRef, altRef);

    // Set and publish motor transforms for the four motors
    Eigen::Isometry3d motorFrame = Eigen::Isometry3d::Identity();
    double momentArm;
    getParameter("moment_arm",                momentArm,                  0.08,     "m");

    motorFrame.translation() = Eigen::Vector3d(momentArm,momentArm,0.);
    multicopterSim_->setMotorFrame(motorFrame,1,0);
    publishStaticMotorTransform(ros::Time::now(), "uav/imu", "uav/motor0", motorFrame);

    motorFrame.translation() = Eigen::Vector3d(-momentArm,momentArm,0.);
    multicopterSim_->setMotorFrame(motorFrame,-1,1);
    publishStaticMotorTransform(ros::Time::now(), "uav/imu", "uav/motor1", motorFrame);

    motorFrame.translation() = Eigen::Vector3d(-momentArm,-momentArm,0.);
    multicopterSim_->setMotorFrame(motorFrame,1,2);
    publishStaticMotorTransform(ros::Time::now(), "uav/imu", "uav/motor2", motorFrame);

    motorFrame.translation() = Eigen::Vector3d(momentArm,-momentArm,0.);
    multicopterSim_->setMotorFrame(motorFrame,-1,3);
    publishStaticMotorTransform(ros::Time::now(), "uav/imu", "uav/motor3", motorFrame);
}

void MulticopterDynamicsWrapper::process(double dt_secs,
                                         const std::vector<double> & motorSpeedCommandIn,
                                         bool isCmdPercent){
    multicopterSim_->proceedState_ExplicitEuler(dt_secs, motorSpeedCommandIn, isCmdPercent);
}

Eigen::Vector3d MulticopterDynamicsWrapper::getVehiclePosition() const{
    return multicopterSim_->getVehiclePosition();
}
Eigen::Quaterniond MulticopterDynamicsWrapper::getVehicleAttitude() const{
    return multicopterSim_->getVehicleAttitude();
}
Eigen::Vector3d MulticopterDynamicsWrapper::getVehicleVelocity(void) const{
    return multicopterSim_->getVehicleVelocity();
}
Eigen::Vector3d MulticopterDynamicsWrapper::getVehicleAngularVelocity(void) const{
    return multicopterSim_->getVehicleAngularVelocity();
}
void MulticopterDynamicsWrapper::getIMUMeasurement(Eigen::Vector3d & accOutput, Eigen::Vector3d & gyroOutput) const{
    return multicopterSim_->getIMUMeasurement(accOutput, gyroOutput);
}


void MulticopterDynamicsWrapper::enu2Geodetic(double east, double north, double up,
                                             double *latitude, double *longitude, double *altitude){
    multicopterSim_->geodetic_converter_.enu2Geodetic(east, north, up, latitude, longitude, altitude);
}

/**
 * @brief Publish static transform from UAV centroid to motor
 * 
 * @param timeStamp Tf timestamp
 * @param frame_id Parent (UAV) frame ID
 * @param child_frame_id Child (motor) frame ID
 * @param motorFrame Transformation
 */
void MulticopterDynamicsWrapper::publishStaticMotorTransform(
                  const ros::Time & timeStamp, const char * frame_id,
                  const char * child_frame_id, const Eigen::Isometry3d & motorFrame){

  geometry_msgs::TransformStamped transformMotor;

  transformMotor.header.stamp = timeStamp;
  transformMotor.header.frame_id = frame_id;
  transformMotor.transform.translation.x = motorFrame.translation()(0);
  transformMotor.transform.translation.y = motorFrame.translation()(1);
  transformMotor.transform.translation.z = motorFrame.translation()(2);

  Eigen::Quaterniond motorAttitude(motorFrame.linear());

  transformMotor.transform.rotation.x = motorAttitude.x();
  transformMotor.transform.rotation.y = motorAttitude.y();
  transformMotor.transform.rotation.z = motorAttitude.z();
  transformMotor.transform.rotation.w = motorAttitude.w();
  transformMotor.child_frame_id = child_frame_id;

  staticTfPub_.sendTransform(transformMotor);
}