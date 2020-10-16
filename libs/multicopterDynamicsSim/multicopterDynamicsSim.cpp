/**
 * @file multicopterDynamicsSim.cpp
 * @author Ezra Tal
 * @brief Multicopter dynamics simulator class implementation
 * 
 */
#include "multicopterDynamicsSim.hpp"
#include <iostream>
#include <chrono>

/**
 * @brief Construct a new Multicopter Dynamics Sim object
 * 
 * @param numCopter Number of motors (e.g. 4 for a quadcopter)
 * @param thrustCoefficient Motors thrust coefficient
 * @param torqueCoefficient Motors torque coefficient
 * @param minMotorSpeed Motors minimum rotation speed
 * @param maxMotorSpeed Motors maximum rotation speed
 * @param motorTimeConstant Motors time constant
 * @param motorRotationalInertia Motors rotational mass moment of inertia (including propeller)
 * @param vehicleMass Vehicle mass
 * @param vehicleInertia Vehicle inertia matrix
 * @param aeroMomentCoefficient Vehicle aerodynamic moment coefficient matrix
 * @param dragCoefficient Vehicle drag coefficient
 * @param momentProcessNoiseAutoCorrelation Vehicle dynamics stochastic moment process noise auto correlation
 * @param forceProcessNoiseAutoCorrelation Vehicle dynamics stochastic force process noise auto correlation
 * @param gravity Gravity vector in world-fixed reference frame
 */
MulticopterDynamicsSim::MulticopterDynamicsSim(int numCopter,
double thrustCoefficient, double torqueCoefficient,
double minMotorSpeed, double maxMotorSpeed,
double motorTimeConstant, double motorRotationalInertia,
double vehicleMass,
const Eigen::Matrix3d & vehicleInertia, 
const Eigen::Matrix3d & aeroMomentCoefficient,
double dragCoefficient,
double momentProcessNoiseAutoCorrelation,
double forceProcessNoiseAutoCorrelation,
const Eigen::Vector3d & gravity
)
: numCopter_(numCopter)
, motorFrame_(numCopter)
, motorDirection_(numCopter)
, motorRotationalInertia_(numCopter)
, thrustCoefficient_(numCopter)
, torqueCoefficient_(numCopter)
, motorSpeed_(numCopter)
, motorTimeConstant_(numCopter)
, maxMotorSpeed_(numCopter)
, minMotorSpeed_(numCopter)
{
    randomNumberGenerator_.seed(std::chrono::system_clock::now().time_since_epoch().count());

    for (int indx = 0; indx < numCopter; indx++){
        motorFrame_.at(indx).setIdentity();
        thrustCoefficient_.at(indx) = thrustCoefficient;
        torqueCoefficient_.at(indx) = torqueCoefficient;
        motorTimeConstant_.at(indx) = motorTimeConstant;
        motorRotationalInertia_.at(indx) = motorRotationalInertia;
        maxMotorSpeed_.at(indx) = maxMotorSpeed;
        minMotorSpeed_.at(indx) = minMotorSpeed;
        motorDirection_.at(indx) = 1;
        motorSpeed_.at(indx) = 0.;
    }

    vehicleMass_ = vehicleMass;
    vehicleInertia_ = vehicleInertia;
    aeroMomentCoefficient_ = aeroMomentCoefficient;
    dragCoefficient_ = dragCoefficient;
    momentProcessNoiseAutoCorrelation_ = momentProcessNoiseAutoCorrelation;
    forceProcessNoiseAutoCorrelation_ = forceProcessNoiseAutoCorrelation;

    gravity_ = gravity;
}

/**
 * @brief Brief constructor for a new Multicopter Dynamics Sim object;
 * vehicle properties must still be set seperately
 * 
 * @param numCopter Number of motors (e.g. 4 for a quadcopter)
 */
MulticopterDynamicsSim::MulticopterDynamicsSim(int numCopter)
: numCopter_(numCopter)
, motorFrame_(numCopter)
, motorDirection_(numCopter)
, motorRotationalInertia_(numCopter)
, thrustCoefficient_(numCopter)
, torqueCoefficient_(numCopter)
, motorSpeed_(numCopter)
, motorTimeConstant_(numCopter)
, maxMotorSpeed_(numCopter)
, minMotorSpeed_(numCopter)
{
    randomNumberGenerator_.seed(std::chrono::system_clock::now().time_since_epoch().count());

    for (int indx = 0; indx < numCopter; indx++){
        motorFrame_.at(indx).setIdentity();
        thrustCoefficient_.at(indx) = 0.;
        torqueCoefficient_.at(indx) = 0.;
        motorTimeConstant_.at(indx) = 0.;
        motorRotationalInertia_.at(indx) = 0.;
        maxMotorSpeed_.at(indx) = 0.;
        minMotorSpeed_.at(indx) = 0.;
        motorDirection_.at(indx) = 1;
        motorSpeed_.at(indx) = 0.;
    }

    // Default is NED, but can be set by changing gravity direction
    gravity_ << 0.,0.,9.81;
}

/**
 * @brief Set vehicle properties
 * 
 * @param vehicleMass Vehicle mass
 * @param vehicleInertia Vehicle inertia matrix
 * @param aeroMomentCoefficient Vehicle aerodynamic moment coefficient matrix
 * @param dragCoefficient Vehicle drag coefficient
 * @param momentProcessNoiseAutoCorrelation Vehicle dynamics stochastic moment process noise auto correlation
 * @param forceProcessNoiseAutoCorrelation Vehicle dynamics stochastic force process noise auto correlation
 */
void MulticopterDynamicsSim::setVehicleProperties(double vehicleMass,
                                            const Eigen::Matrix3d & vehicleInertia, 
                                            const Eigen::Matrix3d & aeroMomentCoefficient,
                                            double dragCoefficient,
                                            double momentProcessNoiseAutoCorrelation,
                                            double forceProcessNoiseAutoCorrelation){
    vehicleMass_ = vehicleMass;
    vehicleInertia_ = vehicleInertia;
    aeroMomentCoefficient_ = aeroMomentCoefficient;
    dragCoefficient_ = dragCoefficient;
    momentProcessNoiseAutoCorrelation_ = momentProcessNoiseAutoCorrelation;
    forceProcessNoiseAutoCorrelation_ = forceProcessNoiseAutoCorrelation;
}

/**
 * @brief Set orientation of world-fixed reference frame using gravity vector
 * 
 * @param gravity Gravity vector in world-fixed reference frame
 */
void MulticopterDynamicsSim::setGravityVector(const Eigen::Vector3d & gravity){
    gravity_ = gravity;
}

/**
 * @brief Set orientation and position for individual motor
 * 
 * @param motorFrame Motor orientation and position with regard to body-fixed reference frame
 * @param motorDirection Motor rotation direction
 *         +1 if positive motor speed corresponds to positive moment around the motor frame z-axis
           -1 if positive motor speed corresponds to negative moment around the motor frame z-axis
           i.e. -1 indicates a positive motor speed corresponds to a positive rotation rate around the motor z-axis
 * @param motorIndex Motor index number
 */
void MulticopterDynamicsSim::setMotorFrame(const Eigen::Isometry3d & motorFrame, int motorDirection, int motorIndex){
    motorFrame_.at(motorIndex) = motorFrame;
    motorDirection_.at(motorIndex) = motorDirection;
}

/**
 * @brief Set properties for individual motor
 * 
 * @param thrustCoefficient Motor thrust coefficient
 * @param torqueCoefficient Motor torque coefficient
 * @param motorTimeConstant Motor time constant
 * @param minMotorSpeed Minimum motor rotation speed
 * @param maxMotorSpeed Maximum motor rotation speed
 * @param rotationalInertia Motor moment of inertia
 * @param motorIndex Motor index number
 */
void MulticopterDynamicsSim::setMotorProperties(double thrustCoefficient, double torqueCoefficient, double motorTimeConstant,
                                                double minMotorSpeed, double maxMotorSpeed, double rotationalInertia, int motorIndex){
    thrustCoefficient_.at(motorIndex) = thrustCoefficient;
    torqueCoefficient_.at(motorIndex) = torqueCoefficient;
    motorTimeConstant_.at(motorIndex) = motorTimeConstant;
    maxMotorSpeed_.at(motorIndex) = maxMotorSpeed;
    minMotorSpeed_.at(motorIndex) = minMotorSpeed;
    motorRotationalInertia_.at(motorIndex) = rotationalInertia;
}

/**
 * @brief Set properties for all motors
 * 
 * @param thrustCoefficient Motor thrust coefficient
 * @param torqueCoefficient Motor torque coefficient
 * @param motorTimeConstant Motor time constant
 * @param minMotorSpeed Minimum motor rotation speed
 * @param maxMotorSpeed Maximum motor rotation speed
 * @param rotationalInertia Motor moment of inertia
 */
void MulticopterDynamicsSim::setMotorProperties(double thrustCoefficient, double torqueCoefficient, double motorTimeConstant,
                                                double minMotorSpeed, double maxMotorSpeed, double rotationalInertia){
    for (int motorIndex = 0; motorIndex < numCopter_; motorIndex++){
        setMotorProperties(thrustCoefficient, torqueCoefficient, motorTimeConstant,
                                             minMotorSpeed, maxMotorSpeed, rotationalInertia, motorIndex);
    }
}

/**
 * @brief Set motor speed for individual motor
 * 
 * @param motorSpeed Motor speed value
 * @param motorIndex Motor index number
 */
void MulticopterDynamicsSim::setMotorSpeed(double motorSpeed, int motorIndex){
    motorSpeed_.at(motorIndex) = motorSpeed;
}


/**
 * @brief Set motor speed for all motors
 * 
 * @param motorSpeed Motor speed value
 */
void MulticopterDynamicsSim::setMotorSpeed(double motorSpeed){
    for (int motorIndex = 0; motorIndex < numCopter_; motorIndex++){
        setMotorSpeed(motorSpeed, motorIndex);
    }
}

/**
 * @brief Set motor speed to zero for all motors
 * 
 */
void MulticopterDynamicsSim::resetMotorSpeeds(void){
    for (int indx = 0; indx < numCopter_; indx++){
        motorSpeed_.at(indx) = 0.;
    }
}

/**
 * @brief Set vehicle position and attitude
 * 
 * @param position Position in world-fixed reference frame
 * @param attitude Vehilce attitude quaternion
 */
void MulticopterDynamicsSim::setVehiclePosition(const Eigen::Vector3d & position,
                                                const Eigen::Quaterniond & attitude){
    position_ = position;
    attitude_ = attitude;

    angularVelocity_.setZero();
    velocity_.setZero();

    resetMotorSpeeds();
}

void MulticopterDynamicsSim::setVehicleInitialAttitude(const Eigen::Quaterniond & attitude){
    default_attitude_ = attitude;
}

/**
 * @brief Set vehicle state
 * 
 * @param position Position in world-fixed reference frame
 * @param velocity Velocity in world-fixed reference frame
 * @param angularVelocity Angular velocity in vehicle-fixed reference frame
 * @param attitude Vehilce attitude quaternion
 * @param motorSpeed Vector with motor speeds for all motors
 */
void MulticopterDynamicsSim::setVehicleState(const Eigen::Vector3d & position,
                                             const Eigen::Vector3d & velocity,
                                             const Eigen::Vector3d & angularVelocity,
                                             const Eigen::Quaterniond & attitude,
                                             const std::vector<double> & motorSpeed){
    position_ = position;
    velocity_ = velocity;
    angularVelocity_ = angularVelocity;
    attitude_ = attitude;
    for (int indx = 0; indx < numCopter_; indx++){
        motorSpeed_.at(indx) = motorSpeed.at(indx);
    }
}

/**
 * @brief Get vehicle state
 * 
 * @param position Position in world-fixed reference frame output
 * @param velocity Velocity in world-fixed reference frame output
 * @param angularVelocity Angular velocity in vehicle-fixed reference frame output
 * @param attitude Vehilce attitude quaternion output
 * @param motorSpeed Vector with motor speeds for all motors output
 */
void MulticopterDynamicsSim::getVehicleState(Eigen::Vector3d & position,
                                             Eigen::Vector3d & velocity,
                                             Eigen::Vector3d & angularVelocity,
                                             Eigen::Quaterniond & attitude,
                                             std::vector<double> & motorSpeed){
    position = position_;
    velocity = velocity_;
    angularVelocity = angularVelocity_;
    attitude = attitude_;
    motorSpeed = motorSpeed_;
}

/**
 * @brief Get vehicle position
 * 
 * @return Eigen::Vector3d Position in world-fixed reference frame
 */
Eigen::Vector3d MulticopterDynamicsSim::getVehiclePosition(void){
    return position_;
}

/**
 * @brief Get vehicle attitude 
 * 
 * @return Eigen::Quaterniond Vehicle attitude quaternion
 */
Eigen::Quaterniond MulticopterDynamicsSim::getVehicleAttitude(void){
    return attitude_;
}

/**
 * @brief Get vehicle velocity
 * 
 * @return Eigen::Vector3d Velocity in world-fixed reference frame
 */
Eigen::Vector3d MulticopterDynamicsSim::getVehicleVelocity(void){
    return velocity_;
}

/**
 * @brief Get vehicle angular velocity
 * 
 * @return Eigen::Vector3d Angular velocity in vehicle-fixed reference frame
 */
Eigen::Vector3d MulticopterDynamicsSim::getVehicleAngularVelocity(void){
    return angularVelocity_;
}

/**
 * @brief Get total specific force acting on vehicle, excluding gravity force
 * 
 * @return Eigen::Vector3d Specific force in vehicle-fixed reference frame
 */
Eigen::Vector3d MulticopterDynamicsSim::getVehicleSpecificForce(void){
    Eigen::Vector3d specificForce = (getThrust(motorSpeed_) + 
                                     attitude_.inverse()*(getDragForce(velocity_) + stochForce_))
                                     / vehicleMass_;
    
    return specificForce;
}


/**
 * @brief Get total force acting on vehicle, including gravity force
 * 
 * @return Eigen::Vector3d Total force in vehicle-fixed reference frame
 */
Eigen::Vector3d MulticopterDynamicsSim::getTotalForce(void){
    return (getThrust(motorSpeed_) + attitude_.inverse()*(gravity_*vehicleMass_ + getDragForce(velocity_) + stochForce_))/vehicleMass_;
}


/**
 * @brief Get thrust in vehicle-fixed reference frame
 * 
 * @param motorSpeed Vector containing motor speeds
 * @return Eigen::Vector3d Thrust vector
 */
Eigen::Vector3d MulticopterDynamicsSim::getThrust(const std::vector<double> & motorSpeed){
    Eigen::Vector3d thrust = Eigen::Vector3d::Zero();
    for (int indx = 0; indx < numCopter_; indx++){

        Eigen::Vector3d motorThrust(0.,0.,fabs(motorSpeed.at(indx))*motorSpeed.at(indx)*thrustCoefficient_.at(indx));
        thrust += motorFrame_.at(indx).linear()*motorThrust;
    }
    return thrust;
}

/**
 * @brief Get control moment in vehicle-fixed reference frame
 * 
 * @param motorSpeed Vector of motor speeds
 * @param motorAcceleration Vector of motor accelerations
 * @return Eigen::Vector3d Moment vector
 */
Eigen::Vector3d MulticopterDynamicsSim::getControlMoment(const std::vector<double> & motorSpeed, const std::vector<double> & motorAcceleration){
    Eigen::Vector3d controlMoment = Eigen::Vector3d::Zero();

    for (int indx = 0; indx < numCopter_; indx++){
        // Moment due to thrust
        Eigen::Vector3d motorThrust(0.,0.,fabs(motorSpeed.at(indx))*motorSpeed.at(indx)*thrustCoefficient_.at(indx));
        controlMoment += motorFrame_.at(indx).translation().cross(motorFrame_.at(indx).linear()*motorThrust);

        // Moment due to torque
        Eigen::Vector3d motorTorque(0.,0.,motorDirection_.at(indx)*fabs(motorSpeed.at(indx))*motorSpeed.at(indx)*torqueCoefficient_.at(indx));
        motorTorque(2) += motorDirection_.at(indx)*motorRotationalInertia_.at(indx)*motorAcceleration.at(indx);
        controlMoment += motorFrame_.at(indx).linear()*motorTorque;
    }

    return controlMoment;
}

/**
 * @brief Get aerodynamic moment in vehicle-fixed reference frame
 * 
 * @param angularVelocity Vehicle angular velocity
 * @return Eigen::Vector3d Aerodynamic moment vector
 */
Eigen::Vector3d MulticopterDynamicsSim::getAeroMoment(const Eigen::Vector3d & angularVelocity){
    return (-angularVelocity.norm()*aeroMomentCoefficient_*angularVelocity);
}

/**
 * @brief Get drag force in world-fixed reference frame
 * 
 * @param velocity Vehicle velocity in world-fixed reference frame
 * @return Eigen::Vector3d Drag force vector
 */
Eigen::Vector3d MulticopterDynamicsSim::getDragForce(const Eigen::Vector3d & velocity){
    return (-dragCoefficient_*velocity.norm()*velocity);
}

/**
 * @brief Get IMU measurement
 * 
 * @param accOutput Ouput accelerometer measurement
 * @param gyroOutput Ouput gyroscope measurement
 */
void MulticopterDynamicsSim::getIMUMeasurement(Eigen::Vector3d & accOutput, Eigen::Vector3d & gyroOutput){
    if( position_.z() < 0.1){
        Eigen::Vector3d zero_force;
        zero_force.setZero();
        imu_.getMeasurement(accOutput, gyroOutput, zero_force, angularVelocity_);
        accOutput = accOutput - attitude_.inverse()*gravity_;
    }else{
        imu_.getMeasurement(accOutput, gyroOutput, getVehicleSpecificForce(), angularVelocity_);
    }
}

/**
 * @brief Proceed vehicle dynamics using Explicit Euler integration
 * 
 * @param dt_secs Time step
 * @param motorSpeedCommand Motor speed commands 
 */
void MulticopterDynamicsSim::proceedState_ExplicitEuler(double dt_secs, const std::vector<double> & motorSpeedCommandIn, bool isCmdPercent){
    std::vector<double> motorSpeedCommand = motorSpeedCommandIn;
    if(isCmdPercent)
    {
        for (size_t i = 0; i < motorSpeedCommand.size(); i++)
        {
            motorSpeedCommand[i] *= maxMotorSpeed_[i];
        }
    }

    std::vector<double> motorSpeedCommandBounded(numCopter_);
    vectorBoundOp(motorSpeedCommand,motorSpeedCommandBounded,minMotorSpeed_,maxMotorSpeed_);

    std::vector<double> motorSpeed(motorSpeed_);
    Eigen::Vector3d position = position_;
    Eigen::Vector3d velocity = velocity_;
    Eigen::Vector3d angularVelocity = angularVelocity_;
    Eigen::Quaterniond attitude = attitude_;

    stochForce_ /= sqrt(dt_secs);
    Eigen::Vector3d stochMoment;
    stochMoment << sqrt(momentProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
                   sqrt(momentProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
                   sqrt(momentProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_);

    std::vector<double> motorSpeedDer(numCopter_);
    getMotorSpeedDerivative(motorSpeedDer,motorSpeed,motorSpeedCommandBounded);
    Eigen::Vector3d positionDer = velocity;
    Eigen::Vector3d velocityDer = getVelocityDerivative(attitude,stochForce_,velocity,motorSpeed);
    Eigen::Vector4d attitudeDer = getAttitudeDerivative(attitude,angularVelocity);
    Eigen::Vector3d angularVelocityDer = getAngularVelocityDerivative(motorSpeed,motorSpeedDer,angularVelocity,stochMoment);

    vectorAffineOp(motorSpeed,motorSpeedDer,motorSpeed_,dt_secs);
    vectorBoundOp(motorSpeed_,motorSpeed_,minMotorSpeed_,maxMotorSpeed_);
    position_ = position + positionDer*dt_secs;
    velocity_ = velocity + velocityDer*dt_secs;
    angularVelocity_ = angularVelocity + angularVelocityDer*dt_secs;
    attitude_.coeffs() = attitude.coeffs() + attitudeDer*dt_secs;

    attitude_.normalize();

    if( position_.z() < 0){
        position_[2] = 0.00;
        velocity_ << 0.0, 0.0, 0.0;
        angularVelocity_ << 0.0, 0.0, 0.0;
        attitude_ = default_attitude_;
    }

    stochForce_ << sqrt(forceProcessNoiseAutoCorrelation_)*standardNormalDistribution_(randomNumberGenerator_),
                   sqrt(forceProcessNoiseAutoCorrelation_)*standardNormalDistribution_(randomNumberGenerator_),
                   sqrt(forceProcessNoiseAutoCorrelation_)*standardNormalDistribution_(randomNumberGenerator_);

    imu_.proceedBiasDynamics(dt_secs);
}

/**
 * @brief Proceed vehicle dynamics using 4th order Runge-Kutta integration
 * 
 * @param dt_secs Time step
 * @param motorSpeedCommand Motor speed commands 
 */
void MulticopterDynamicsSim::proceedState_RK4(double dt_secs, const std::vector<double> & motorSpeedCommandIn, bool isCmdPercent){
    std::vector<double> motorSpeedCommand = motorSpeedCommandIn;

    if(isCmdPercent)
    {
        for (size_t i = 0; i < motorSpeedCommand.size(); i++)
        {
            motorSpeedCommand[i] *= maxMotorSpeed_[i];
        }
    }
    std::vector<double> motorSpeedCommandBounded(numCopter_);
    vectorBoundOp(motorSpeedCommand,motorSpeedCommandBounded,minMotorSpeed_,maxMotorSpeed_);

    std::vector<double> motorSpeed(motorSpeed_);
    Eigen::Vector3d position = position_;
    Eigen::Vector3d velocity = velocity_;
    Eigen::Vector3d angularVelocity = angularVelocity_;
    Eigen::Quaterniond attitude = attitude_;

    stochForce_ /= sqrt(dt_secs);

    Eigen::Vector3d stochMoment;
    stochMoment << sqrt(momentProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
                   sqrt(momentProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
                   sqrt(momentProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_);

    // k1
    std::vector<double> motorSpeedDer(numCopter_);
    getMotorSpeedDerivative(motorSpeedDer,motorSpeed_,motorSpeedCommandBounded);
    Eigen::Vector3d positionDer = dt_secs*velocity_;
    Eigen::Vector3d velocityDer = dt_secs*getVelocityDerivative(attitude_,stochForce_,velocity_,motorSpeed_);
    Eigen::Vector4d attitudeDer = dt_secs*getAttitudeDerivative(attitude_,angularVelocity_);
    Eigen::Vector3d angularVelocityDer = dt_secs*getAngularVelocityDerivative(motorSpeed_,motorSpeedDer,angularVelocity_,stochMoment);
    vectorScalarProd(motorSpeedDer,motorSpeedDer,dt_secs);

    // x + 1/6*(k1)
    vectorAffineOp(motorSpeed,motorSpeedDer,motorSpeed,(1./6.));
    position += (1./6.)*positionDer;
    velocity += (1./6.)*velocityDer;
    attitude.coeffs() += (1./6.)*attitudeDer;
    attitude.normalize();
    angularVelocity += (1./6.)*angularVelocityDer;

    // k2
    std::vector<double> motorSpeedIntermediate(numCopter_);
    Eigen::Quaterniond attitudeIntermediate;

    vectorAffineOp(motorSpeed_,motorSpeedDer,motorSpeedIntermediate,0.5);  // x + 0.5*(k1)
    vectorBoundOp(motorSpeedIntermediate,motorSpeedIntermediate,minMotorSpeed_,maxMotorSpeed_);
    attitudeIntermediate.coeffs() = attitude_.coeffs() + attitudeDer*0.5;
    attitudeIntermediate.normalize();

    getMotorSpeedDerivative(motorSpeedDer, motorSpeedIntermediate, motorSpeedCommandBounded);
    positionDer = dt_secs*(velocity_ + 0.5*velocityDer);
    velocityDer = dt_secs*getVelocityDerivative(attitudeIntermediate,stochForce_,(velocity_ + 0.5*velocityDer), motorSpeedIntermediate);
    attitudeDer = dt_secs*getAttitudeDerivative(attitudeIntermediate,(angularVelocity_ + 0.5*angularVelocityDer));
    angularVelocityDer = dt_secs*getAngularVelocityDerivative(motorSpeedIntermediate,motorSpeedDer,(angularVelocity_ + 0.5*angularVelocityDer),stochMoment);
    vectorScalarProd(motorSpeedDer, motorSpeedDer, dt_secs);

    // x + 1/6*(k1 + 2*k2)
    vectorAffineOp(motorSpeed,motorSpeedDer,motorSpeed,(1./3.));
    position += (1./3.)*positionDer;
    velocity += (1./3.)*velocityDer;
    attitude.coeffs() += (1./3.)*attitudeDer;
    attitude.normalize();
    angularVelocity += (1./3.)*angularVelocityDer;

    // k3
    vectorAffineOp(motorSpeed_,motorSpeedDer,motorSpeedIntermediate,0.5);
    vectorBoundOp(motorSpeedIntermediate,motorSpeedIntermediate,minMotorSpeed_,maxMotorSpeed_);
    attitudeIntermediate.coeffs() = attitude_.coeffs() + attitudeDer*0.5;
    attitudeIntermediate.normalize();

    getMotorSpeedDerivative(motorSpeedDer, motorSpeedIntermediate, motorSpeedCommandBounded);
    positionDer = dt_secs*(velocity_ + 0.5*velocityDer);
    velocityDer = dt_secs*getVelocityDerivative(attitudeIntermediate,stochForce_,(velocity_ + 0.5*velocityDer), motorSpeedIntermediate);
    attitudeDer = dt_secs*getAttitudeDerivative(attitudeIntermediate,(angularVelocity_ + 0.5*angularVelocityDer));
    angularVelocityDer = dt_secs*getAngularVelocityDerivative(motorSpeedIntermediate,motorSpeedDer,(angularVelocity_ + 0.5*angularVelocityDer),stochMoment);
    vectorScalarProd(motorSpeedDer, motorSpeedDer, dt_secs);

    // x + 1/6*(k1 + 2*k2 + 2*k3)
    vectorAffineOp(motorSpeed,motorSpeedDer,motorSpeed,(1./3.));
    position += (1./3.)*positionDer;
    velocity += (1./3.)*velocityDer;
    attitude.coeffs() += (1./3.)*attitudeDer;
    attitude.normalize();
    angularVelocity += (1./3.)*angularVelocityDer;

    // k4
    vectorAffineOp(motorSpeed_,motorSpeedDer,motorSpeedIntermediate,1.);
    vectorBoundOp(motorSpeedIntermediate,motorSpeedIntermediate,minMotorSpeed_,maxMotorSpeed_);
    attitudeIntermediate.coeffs() = attitude_.coeffs() + attitudeDer;
    attitudeIntermediate.normalize();

    getMotorSpeedDerivative(motorSpeedDer, motorSpeedIntermediate, motorSpeedCommandBounded);
    positionDer = dt_secs*(velocity_ + velocityDer);
    velocityDer = dt_secs*getVelocityDerivative(attitudeIntermediate,stochForce_,(velocity_ + velocityDer), motorSpeedIntermediate);
    attitudeDer = dt_secs*getAttitudeDerivative(attitudeIntermediate,(angularVelocity_ + angularVelocityDer));
    angularVelocityDer = dt_secs*getAngularVelocityDerivative(motorSpeedIntermediate,motorSpeedDer,(angularVelocity_ + angularVelocityDer),stochMoment);
    vectorScalarProd(motorSpeedDer, motorSpeedDer, dt_secs);

    // x + 1/6*(k1 + 2*k2 + 2*k3 + k4)
    vectorAffineOp(motorSpeed,motorSpeedDer,motorSpeed_,(1./6.));
    vectorBoundOp(motorSpeed_,motorSpeed_,minMotorSpeed_,maxMotorSpeed_);
    position_ = position + positionDer*(1./6.);
    velocity_ = velocity + velocityDer*(1./6.);
    attitude_.coeffs() = attitude.coeffs() + attitudeDer*(1./6.);
    attitude_.normalize();
    angularVelocity_ = angularVelocity + angularVelocityDer*(1./6.);

    stochForce_ << sqrt(forceProcessNoiseAutoCorrelation_)*standardNormalDistribution_(randomNumberGenerator_),
                   sqrt(forceProcessNoiseAutoCorrelation_)*standardNormalDistribution_(randomNumberGenerator_),
                   sqrt(forceProcessNoiseAutoCorrelation_)*standardNormalDistribution_(randomNumberGenerator_);

    imu_.proceedBiasDynamics(dt_secs);
}

/**
 * @brief Element-wise affine vector calculus: vec3 = vec1 + val*vec2
 * 
 * @param vec1 Vector addition term
 * @param vec2 Vector multiplication factor
 * @param vec3 Output resulting vector
 * @param val Scalar multiplication factor
 */
void MulticopterDynamicsSim::vectorAffineOp(const std::vector<double> & vec1, const std::vector<double> & vec2,
                                         std::vector<double> & vec3, double val){
    std::transform(vec1.begin(), vec1.end(), vec2.begin(), vec3.begin(), [val](const double & vec1val, const double & vec2val)->double{return (vec1val + val*vec2val);});
}

/**
 * @brief Element-wise vector bound operation: vec2 = max(minvalue, min(maxvalue, vec1))
 * 
 * @param vec1 Input vector
 * @param vec2 Output vector
 * @param minvec Vector of lower bounds
 * @param maxvec Vector of upper bounds
 */
void MulticopterDynamicsSim::vectorBoundOp(const std::vector<double> & vec1, std::vector<double> & vec2,
                                         const std::vector<double> &  minvec, const std::vector<double> & maxvec){
    std::transform(vec1.begin(), vec1.end(), maxvec.begin(), vec2.begin(), [](const double & vec1val, const double & maxvalue)->double{return fmin(vec1val,maxvalue);});
    std::transform(vec2.begin(), vec2.end(), minvec.begin(), vec2.begin(), [](const double & vec2val, const double & minvalue)->double{return fmax(vec2val,minvalue);});
}

/**
 * @brief Vector-scalar product: vec2 = val*vec1
 * 
 * @param vec1 Input vector
 * @param vec2 Output vector
 * @param val Scalar multiplication
 */
void MulticopterDynamicsSim::vectorScalarProd(const std::vector<double> & vec1, std::vector<double> & vec2, double val){
    std::transform(vec1.begin(), vec1.end(), vec2.begin(), [val](const double & vec1val){return vec1val*val;});
}

/**
 * @brief Get motor acceleration
 * 
 * @param motorSpeedDer Output vector of accelerationa
 * @param motorSpeed Motor speeds vector
 * @param motorSpeedCommand Motor commanded speeds vector
 */
void MulticopterDynamicsSim::getMotorSpeedDerivative(std::vector<double> & motorSpeedDer,
                                                  const std::vector<double> & motorSpeed,
                                                  const std::vector<double> & motorSpeedCommand){
    for (int indx = 0; indx < numCopter_; indx++){
        motorSpeedDer.at(indx) = (motorSpeedCommand.at(indx) - motorSpeed.at(indx))/motorTimeConstant_.at(indx);
    }
}

/**
 * @brief Get vehicle accelertion in world-fixed reference frame
 * 
 * @param attitude Vehicle attitude
 * @param stochForce Stochastic force vecotr
 * @param velocity Vehicle velocity
 * @param motorSpeed Motor speeds vector
 * @return Eigen::Vector3d Acceleration vector
 */
Eigen::Vector3d MulticopterDynamicsSim::getVelocityDerivative(const Eigen::Quaterniond & attitude, const Eigen::Vector3d & stochForce,
                                        const Eigen::Vector3d & velocity, const std::vector<double> & motorSpeed){
    return (gravity_ + (attitude*getThrust(motorSpeed) + getDragForce(velocity) + stochForce)/vehicleMass_);
}

/**
 * @brief Get attitude quaternion time-derivative
 * 
 * @param attitude Vehicle attitude
 * @param angularVelocity Vehicle angular velocity in vehicle-fixed reference frame
 * @return Eigen::Vector4d Attitude derivative
 */
Eigen::Vector4d MulticopterDynamicsSim::getAttitudeDerivative(const Eigen::Quaterniond & attitude, const Eigen::Vector3d & angularVelocity){
    Eigen::Quaterniond angularVelocityQuad;
    angularVelocityQuad.w() = 0;
    angularVelocityQuad.vec() = angularVelocity;

    return (0.5*(attitude*angularVelocityQuad).coeffs());
}

/**
 * @brief Get vehicle angular acceleration in vehicle-fixed reference frame
 * 
 * @param motorSpeed Vector of motor speeds
 * @param motorAcceleration Vector of motor accelerations
 * @param angularVelocity Vehicle angular velocity
 * @param stochMoment Stochastic moment vector
 * @return Eigen::Vector3d Angular acceleration
 */
Eigen::Vector3d MulticopterDynamicsSim::getAngularVelocityDerivative(const std::vector<double> & motorSpeed,
    const std::vector<double>& motorAcceleration, const Eigen::Vector3d & angularVelocity, const Eigen::Vector3d & stochMoment){

    Eigen::Vector3d angularMomentum = vehicleInertia_*angularVelocity;

    for (int indx = 0; indx < numCopter_; indx++){
        Eigen::Vector3d motorAngularMomentum = Eigen::Vector3d::Zero();
        
        motorAngularMomentum(2) = -1*motorDirection_.at(indx)*motorRotationalInertia_.at(indx)*motorSpeed.at(indx);

        angularMomentum += motorFrame_.at(indx).linear()*motorAngularMomentum;
    }

    return (vehicleInertia_.inverse()*(getControlMoment(motorSpeed,motorAcceleration) + getAeroMoment(angularVelocity) + stochMoment 
                                                       - angularVelocity.cross(angularMomentum)));
}