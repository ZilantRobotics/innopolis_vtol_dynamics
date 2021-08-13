/**
 * @file inertialMeasurementSim.cpp
 * @author Ezra Tal
 * @brief Inertial measurement unit (IMU) simulator class implementation
 * 
 */
#include "inertialMeasurementSim.hpp"
#include <chrono>

/**
 * @brief Construct a new IMU Sim object
 * 
 * @param accMeasNoiseVariance Accelerometer additive noise variance
 * @param gyroMeasNoiseVariance Gyroscope additive noise variance
 * @param accBiasProcessNoiseAutoCorrelation Accelerometer bias process noise auto correlation
 * @param gyroBiasProcessNoiseAutoCorrelation Gyroscope bias process noise auto correlation
 */
inertialMeasurementSim::inertialMeasurementSim(double accMeasNoiseVariance, double gyroMeasNoiseVariance,
                        double accBiasProcessNoiseAutoCorrelation, double gyroBiasProcessNoiseAutoCorrelation){

    // RNG seed from current time
    randomNumberGenerator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
                            
    accMeasNoiseVariance_ = accMeasNoiseVariance;
    gyroMeasNoiseVariance_ = gyroMeasNoiseVariance;
    accBiasProcessNoiseAutoCorrelation_ = accBiasProcessNoiseAutoCorrelation;
    gyroBiasProcessNoiseAutoCorrelation_ = gyroBiasProcessNoiseAutoCorrelation;
}

/**
 * @brief Set bias properties
 * 
 * @param accBias Accelerometer bias value
 * @param gyroBias Gyroscope bias value
 * @param accBiasProcessNoiseAutoCorrelation Accelerometer bias process noise auto correlation
 * @param gyroBiasProcessNoiseAutoCorrelation Gyroscope bias process noise auto correlation
 */
void inertialMeasurementSim::setBias(const Eigen::Vector3d & accBias, const Eigen::Vector3d & gyroBias,
                                     double accBiasProcessNoiseAutoCorrelation,
                                     double gyroBiasProcessNoiseAutoCorrelation){
    accBias_ = accBias;
    gyroBias_ = gyroBias;
    accBiasProcessNoiseAutoCorrelation_ = accBiasProcessNoiseAutoCorrelation;
    gyroBiasProcessNoiseAutoCorrelation_ = gyroBiasProcessNoiseAutoCorrelation;
}

/**
 * @brief Set bias properties
 * 
 * @param accBias Accelerometer bias variance
 * @param gyroBias Gyroscope bias variance
 * @param accBiasProcessNoiseAutoCorrelation Accelerometer bias process noise auto correlation
 * @param gyroBiasProcessNoiseAutoCorrelation Gyroscope bias process noise auto correlation
 */
void inertialMeasurementSim::setBias(double accBiasVariance, double gyroBiasVariance,
                                     double accBiasProcessNoiseAutoCorrelation,
                                     double gyroBiasProcessNoiseAutoCorrelation){

    accBias_ << sqrt(accBiasVariance)*standardNormalDistribution_(randomNumberGenerator_),
                sqrt(accBiasVariance)*standardNormalDistribution_(randomNumberGenerator_),
                sqrt(accBiasVariance)*standardNormalDistribution_(randomNumberGenerator_);

    gyroBias_ << sqrt(gyroBiasVariance)*standardNormalDistribution_(randomNumberGenerator_),
                 sqrt(gyroBiasVariance)*standardNormalDistribution_(randomNumberGenerator_),
                 sqrt(gyroBiasVariance)*standardNormalDistribution_(randomNumberGenerator_);

    accBiasProcessNoiseAutoCorrelation_ = accBiasProcessNoiseAutoCorrelation;
    gyroBiasProcessNoiseAutoCorrelation_ = gyroBiasProcessNoiseAutoCorrelation;
}

/**
 * @brief Set bias
 * 
 * @param accBias Accelerometer bias variance
 * @param gyroBias Gyroscope bias variance
 */
void inertialMeasurementSim::setBias(double accBiasVariance, double gyroBiasVariance){

    accBias_ << sqrt(accBiasVariance)*standardNormalDistribution_(randomNumberGenerator_),
                sqrt(accBiasVariance)*standardNormalDistribution_(randomNumberGenerator_),
                sqrt(accBiasVariance)*standardNormalDistribution_(randomNumberGenerator_);

    gyroBias_ << sqrt(gyroBiasVariance)*standardNormalDistribution_(randomNumberGenerator_),
                 sqrt(gyroBiasVariance)*standardNormalDistribution_(randomNumberGenerator_),
                 sqrt(gyroBiasVariance)*standardNormalDistribution_(randomNumberGenerator_);
}

/**
 * @brief Set additive noise variances
 * 
 * @param accMeasNoiseVariance Accelerometer additive noise variance
 * @param gyroMeasNoiseVariance Gyroscope additive noise variance
 */
void inertialMeasurementSim::setNoiseVariance(double accMeasNoiseVariance, double gyroMeasNoiseVariance){
    accMeasNoiseVariance_ = accMeasNoiseVariance;
    gyroMeasNoiseVariance_ = gyroMeasNoiseVariance;
}

/**
 * @brief Set IMU orientation with regard to body-frame
 * 
 * @param imuOrient IMU orientation with regard to body-frame
 */
void inertialMeasurementSim::setOrientation(const Eigen::Quaterniond & imuOrient){
    imuOrient_ = imuOrient;
}

/**
 * @brief Get IMU measurement
 * 
 * @param accOutput Accelerometer output in IMU frame
 * @param gyroOutput Gyroscope output in IMU frame
 * @param specificForce Specific force in body-frame
 * @param angularVelocity Angular velocity in body-frame
 */
void inertialMeasurementSim::getMeasurement(Eigen::Vector3d & accOutput, Eigen::Vector3d & gyroOutput,
                const Eigen::Vector3d specificForce, const Eigen::Vector3d angularVelocity){

    Eigen::Vector3d accNoise;

    accNoise << sqrt(accMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_),
                sqrt(accMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_),
                sqrt(accMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_);

    Eigen::Vector3d gyroNoise;

    gyroNoise << sqrt(gyroMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_),
                 sqrt(gyroMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_),
                 sqrt(gyroMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_);

    accOutput = imuOrient_.inverse()*specificForce + accBias_ + accNoise;
    gyroOutput = imuOrient_.inverse()*angularVelocity + gyroBias_ + gyroNoise;
}

/**
 * @brief Proceed accelerometer and gyroscope bias dynamics
 * 
 * @param dt_secs Time step
 */
void inertialMeasurementSim::proceedBiasDynamics(double dt_secs){
    Eigen::Vector3d accBiasDerivative;

    accBiasDerivative << sqrt(accBiasProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
                         sqrt(accBiasProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
                         sqrt(accBiasProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_);

    Eigen::Vector3d gyroBiasDerivative;

    gyroBiasDerivative << sqrt(gyroBiasProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
                          sqrt(gyroBiasProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
                          sqrt(gyroBiasProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_);

    accBias_ += dt_secs*accBiasDerivative;
    gyroBias_ += dt_secs*gyroBiasDerivative;
}