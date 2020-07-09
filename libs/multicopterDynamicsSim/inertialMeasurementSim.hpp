/**
 * @file inertialMeasurementSim.hpp
 * @author Ezra Tal
 * @brief Inertial measurement unit (IMU) simulator class header file
 * 
 */

#ifndef INERTIALMEASUREMENTSIM_H
#define INERTIALMEASUREMENTSIM_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>

/**
 * @brief Inertial measurement unit (IMU) simulator class
 * 
 */
class inertialMeasurementSim
{
    public:
        inertialMeasurementSim(double accMeasNoiseVariance, double gyroMeasNoiseVariance,
                               double accBiasProcessNoiseAutoCorrelation, double gyroBiasProcessNoiseAutoCorrelation);

        void setBias(const Eigen::Vector3d & accBias, const Eigen::Vector3d & gyroBias,
                                     double accBiasProcessNoiseAutoCorrelation,
                                     double gyroBiasProcessNoiseAutoCorrelation);
        void setBias(double accBiasVariance, double gyroBiasVariance,
                                     double accBiasProcessNoiseAutoCorrelation,
                                     double gyroBiasProcessNoiseAutoCorrelation);

        void setBias(double accBiasVariance, double gyroBiasVariance);

        void setNoiseVariance(double accMeasNoiseVariance, double gyroMeasNoiseVariance);

        void setOrientation(const Eigen::Quaterniond & imuOrient);

        void getMeasurement(Eigen::Vector3d & accOutput, Eigen::Vector3d & gyroOutput,
                      const Eigen::Vector3d specificForce, const Eigen::Vector3d angularVelocity);

        void proceedBiasDynamics(double dt_secs);

    private:
        /// @name Std normal RNG
        //@{
        std::default_random_engine randomNumberGenerator_;
        std::normal_distribution<double> standardNormalDistribution_ = std::normal_distribution<double>(0.0,1.0);
        //@}

        /// @name Measurement noise variance
        //@{
        double accMeasNoiseVariance_ = 0.; // m^2/s^4
        double gyroMeasNoiseVariance_ = 0.; // rad^2/s^2
        //@}

        /// @name Bias dynamics process noise auto correlation
        //@{
        double accBiasProcessNoiseAutoCorrelation_ = 0.; // m^2/s^5
        double gyroBiasProcessNoiseAutoCorrelation_ = 0.; // rad^2/s^3
        //@}

        /// @name Bias states
        //@{
        Eigen::Vector3d accBias_ = Eigen::Vector3d::Zero(); // m/s^2
        Eigen::Vector3d gyroBias_ = Eigen::Vector3d::Zero(); // rad/s
        //@}

        /// @name Orientation of IMU wrt body-fixed frame
        //@{
        Eigen::Quaterniond imuOrient_ = Eigen::Quaterniond::Identity();
        //@}
};

#endif // INERTIALMEASUREMENTSIM_H