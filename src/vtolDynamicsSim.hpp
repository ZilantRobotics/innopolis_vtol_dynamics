/**
 * @file vtolDynamicsSim.hpp
 * @author ponomarevda96@gmail.com
 * @brief Vtol dynamics simulator class header file
 */

#ifndef VTOL_DYNAMICS_SIM_H
#define VTOL_DYNAMICS_SIM_H

#include <Eigen/Geometry>
#include <vector>
#include <array>
#include <random>
#include <geographiclib_conversions/geodetic_conv.hpp>
#include "uavDynamicsSimBase.hpp"



struct VtolParameters{
    double mass;                                    // kg
    double gravity;                                 // n/sec^2
    double atmoRho;                                 // air density (kg/m^3)
    double wingArea;                                // m^2
    double characteristicLength;                    // m
    Eigen::Matrix3d inertia;                        // kg*m^2

    std::array<Eigen::Vector3d, 5> propellersLocation;  // calculate

    double massUncertainty;                         // mass multiplier
    double inertiaUncertainty;                      // mass multiplier

    std::vector<double> actuatorMin;                // rad/sec
    std::vector<double> actuatorMax;                // rad/sec
    std::array<double, 8> deltaControlMax;          // rad/sec^2
    std::array<double, 8> timeConstant;             // sec
    std::array<double, 8> desiredControl;           // rad/sec

    double accVariance;
    double gyroVariance;
};

struct State{
    Eigen::Vector3d position;                       // meters
    Eigen::Vector3d linearVel;                      // m/sec, in body CS
    Eigen::Vector3d linearAccel;                    // m/sec^2, in body CS

    Eigen::Vector3d eulerAngles;                    // rad
    Eigen::Quaterniond attitude;
    Eigen::Vector3d angularVel;                     // rad/sec, in body CS
    Eigen::Vector3d angularAccel;                   // rad/sec^2, in body CS

    Eigen::Vector3d windVelocity;                   // m/sec^2
    Eigen::Vector3d gustVelocity;                   // m/sec^2
    Eigen::Vector3d accelBias;
    Eigen::Vector3d gyroBias;
    Eigen::Vector3d Faero;                          // N
    Eigen::Vector3d Maero;                          // N*m
    Eigen::Vector3d Msteer;                         // N*m
    Eigen::Vector3d Mairspeed;                      // N*m
    Eigen::Vector3d MmotorsTotal;                   // N*m

    Eigen::Vector3d Fspecific;                      // N
    Eigen::Vector3d Ftotal;                         // N
    Eigen::Vector3d Mtotal;                         // N*m

    double windVariance;
    double guVariance;

    std::vector<double> prevActuators;              // rad/sec
    std::vector<double> crntActuators;              // rad/sec
};

struct TablesWithCoeffs{
    Eigen::MatrixXd CS_rudder;
    Eigen::MatrixXd CS_beta;

    Eigen::MatrixXd AoA;
    Eigen::MatrixXd AoS;

    Eigen::MatrixXd actuator;
    Eigen::MatrixXd airspeed;

    Eigen::MatrixXd CLPolynomial;
    Eigen::MatrixXd CSPolynomial;
    Eigen::MatrixXd CDPolynomial;
    Eigen::MatrixXd CmxPolynomial;
    Eigen::MatrixXd CmyPolynomial;
    Eigen::MatrixXd CmzPolynomial;

    Eigen::MatrixXd CmxAileron;
    Eigen::MatrixXd CmyElevator;
    Eigen::MatrixXd CmzRudder;

    Eigen::MatrixXd prop;
};

/**
 * @brief Vtol dynamics simulator class
 */
class InnoVtolDynamicsSim : public UavDynamicsSimBase{
    public:
        InnoVtolDynamicsSim();
        virtual int8_t init();
        virtual void initStaticMotorTransform() override;
        virtual void setReferencePosition(double latRef, double lonRef, double altRef) override;
        virtual void setInitialPosition(const Eigen::Vector3d & position,
                                        const Eigen::Quaterniond& attitude) override;
        virtual void process(double dt_secs,
                             const std::vector<double>& motorSpeedCommandIn,
                             bool isCmdPercent) override;

        virtual Eigen::Vector3d getVehiclePosition() const override;
        virtual Eigen::Quaterniond getVehicleAttitude() const override;
        virtual Eigen::Vector3d getVehicleVelocity() const override;
        virtual Eigen::Vector3d getVehicleAngularVelocity() const override;
        virtual void getIMUMeasurement(Eigen::Vector3d& accOut, Eigen::Vector3d& gyroOut) override;
        virtual void enu2Geodetic(double east, double north, double up,
                                  double *latitude, double *longitude, double *altitude) override;

        /**
         * @note These methods should be public for debug only (publish to ros topic)
         */
        virtual Eigen::Vector3d getFaero() const;
        virtual Eigen::Vector3d getFtotal() const;
        virtual Eigen::Vector3d getMaero() const;
        virtual Eigen::Vector3d getMtotal() const;

        /**
         * @note The methods below are should be public for test only
         */
        Eigen::Vector3d calculateWind();
        Eigen::Matrix3d calculateRotationMatrix() const;
        double calculateDynamicPressure(double airSpeedMod);
        double calculateAnglesOfAtack(const Eigen::Vector3d& airSpeed) const;
        double calculateAnglesOfSideslip(const Eigen::Vector3d& airSpeed) const;
        void thruster(double actuator, double& thrust, double& torque) const;
        void calculateNewState(const Eigen::Vector3d& Maero,
                               const Eigen::Vector3d& Faero,
                               const std::vector<double>& actuator,
                               double dt_sec);

        void calculateAerodynamics(const Eigen::Vector3d& airspeed,
                                   double AoA,
                                   double AoS,
                                   double aileron_pos,
                                   double elevator_pos,
                                   double rudder_pos,
                                   Eigen::Vector3d& Faero,
                                   Eigen::Vector3d& Maero,
                                   double& Cmx_a,
                                   double& Cmy_e,
                                   double& Cmz_r);

        void calculateCLPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs);
        void calculateCSPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs);
        void calculateCDPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs);
        void calculateCmxPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs);
        void calculateCmyPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs);
        void calculateCmzPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs);
        void calculatePolynomialUsingTable(const Eigen::MatrixXd& table,
                                           double airSpeedMod,
                                           Eigen::VectorXd& polynomialCoeffs);

        double calculateCSRudder(double rudder_pos, double airspeed) const;
        double calculateCSBeta(double AoS_deg, double airspeed) const;
        double calculateCmxAileron(double aileron_pos, double airspeed) const;
        double calculateCmyElevator(double elevator_pos, double airspeed) const;
        double calculateCmzRudder(double rudder_pos, double airspeed) const;

        size_t findRow(const Eigen::MatrixXd& table, double value) const;
        double lerp(double a, double b, double f) const;
        /**
         * @note Similar to https://www.mathworks.com/help/matlab/ref/griddata.html
         * Implementation from https://en.wikipedia.org/wiki/Bilinear_interpolation
         */
        double griddata(const Eigen::MatrixXd& x,
                        const Eigen::MatrixXd& y,
                        const Eigen::MatrixXd& z,
                        double xi,
                        double yi) const;
        double polyval(const Eigen::VectorXd& poly, double val) const;
        size_t search(const Eigen::MatrixXd& matrix, double key) const;


        void setWindParameter(Eigen::Vector3d windMeanVelocity, double wind_velocityVariance);
        void setEulerAngles(Eigen::Vector3d eulerAngles);
        void setInitialVelocity(const Eigen::Vector3d & linearVelocity,
                                const Eigen::Vector3d& angularVelocity);

        Eigen::Vector3d getAngularAcceleration() const;
        Eigen::Vector3d getLinearAcceleration() const;
        Eigen::Vector3d getMsteer() const;
        Eigen::Vector3d getMairspeed() const;
        Eigen::Vector3d getMmotorsTotal() const;

    private:
        void loadTables(const std::string& path);
        void loadParams(const std::string& path);
        std::vector<double> mapCmdToActuatorStandardVTOL(const std::vector<double>& cmd) const;
        std::vector<double> mapCmdToActuatorInnoVTOL(const std::vector<double>& cmd) const;
        Eigen::Vector3d calculateAirSpeed(const Eigen::Matrix3d& rotationMatrix,
                                    const Eigen::Vector3d& estimatedVelocity,
                                    const Eigen::Vector3d& windSpeed);

        VtolParameters params_;
        State state_;
        TablesWithCoeffs tables_;

        std::default_random_engine generator_;
        std::normal_distribution<double> distribution_;
        geodetic_converter::GeodeticConverter geodetic_converter_;
};

#endif  // VTOL_DYNAMICS_SIM_H
