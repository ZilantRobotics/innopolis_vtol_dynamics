/**
 * @file innopolis_vtol_dynamics_node.cpp
 * @author Roman Fedorenko
 * @author Ezra Tal
 * @author Winter Guerra
 * @author Varun Murali
 * @brief Implementation of UAV dynamics, IMU, and angular rate control simulation node
 * 
 */

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "innopolis_vtol_dynamics_node.hpp"

/**
 * @brief Global function designated start of UAV dynamics node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "innopolis_vtol_dynamics_node");
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  // Init class
  Uav_Dynamics uav_dynamics_node(n);

  // Spin
  ros::spin();

  return 0;
}

static const std::string INNO_DYNAMICS_NS = "/uav/innopolis_vtol_dynamics/";
void getParameter(std::string name, bool& parameter, bool default_value){
  if (!ros::param::get(INNO_DYNAMICS_NS + name, parameter)){
    std::cout << "Did not get bool "
              << name
              << " from the params, defaulting to "
              << default_value
              << std::endl;
    parameter = default_value;
  }
}
void getParameter(std::string name, double& parameter, double default_value, std::string unit=""){
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

/**
 * @brief Construct a new Uav_Dynamics::Uav_Dynamics object
 * 
 * @param nh ROS Nodehandle
 */
Uav_Dynamics::Uav_Dynamics(ros::NodeHandle nh):
// Node handle
node_(nh)
{
  getParameter("ignore_collisions",         ignoreCollisions_,          false);
  getParameter("use_ratethrust_controller", useRateThrustController_,   true);
  getParameter("use_rungekutta4integrator", useRungeKutta4Integrator_,  false);
  getParameter("/use_sim_time",             useSimTime_,                true);

  getParameter("ignore_collisions",         ignoreCollisions_,          false);
  getParameter("use_ratethrust_controller", useRateThrustController_,   true);
  getParameter("use_rungekutta4integrator", useRungeKutta4Integrator_,  false);
  getParameter("use_sim_time",              useSimTime_,                true);

  double vehicleMass, motorTimeconstant, motorRotationalInertia, momentArm,
         thrustCoeff, torqueCoeff, dragCoeff;
  getParameter("vehicle_mass",              vehicleMass,                1.,       "kg");
  getParameter("motor_time_constant",       motorTimeconstant,          0.02,     "sec");
  getParameter("motor_rotational_inertia",  motorRotationalInertia,     6.62e-6,  "kg m^2");
  getParameter("moment_arm",                momentArm,                  0.35,     "m");
  getParameter("thrust_coefficient",        thrustCoeff,                1.91e-6,  "N/(rad/s)^2");
  getParameter("torque_coefficient",        torqueCoeff,                2.6e-7,   "Nm/(rad/s)^2");
  getParameter("drag_coefficient",          dragCoeff,                  0.1,      "N/(m/s)");

  Eigen::Matrix3d aeroMomentCoefficient = Eigen::Matrix3d::Zero();
  getParameter("aeromoment_coefficient_xx", aeroMomentCoefficient(0,0),0.003,     "Nm/(rad/s)^2");
  getParameter("aeromoment_coefficient_yy", aeroMomentCoefficient(1,1),0.003,     "Nm/(rad/s)^2");
  getParameter("aeromoment_coefficient_zz", aeroMomentCoefficient(2,2),0.003,     "Nm/(rad/s)^2");

  Eigen::Matrix3d vehicleInertia = Eigen::Matrix3d::Zero();
  getParameter("vehicle_inertia_xx",        vehicleInertia(0,0),        0.0049,   "kg m^2");
  getParameter("vehicle_inertia_yy",        vehicleInertia(1,1),        0.0049,   "kg m^2");
  getParameter("vehicle_inertia_zz",        vehicleInertia(2,2),        0.0069,   "kg m^2");
  
  double maxPropSpeed, momentProcessNoiseAutoCorrelation, forceProcessNoiseAutoCorrelation;
  getParameter("max_prop_speed",            maxPropSpeed,               1500,     "rad/s");
  getParameter("force_process_noise", forceProcessNoiseAutoCorrelation, 0.0005,   "N^2 s");

  std::vector<double> initPose(7);
  if (!ros::param::get(INNO_DYNAMICS_NS + "init_pose", initPose)) {
    // Start a few meters above the ground.
    std::cout << "Did NOT find initial pose from param file" << std::endl;

    initPose.at(2) = 0.2;
    initPose.at(6) = 1.0;
  }

  // Set gravity vector according to ROS reference axis system, see header file
  Eigen::Vector3d gravity(0.,0.,-9.81);

  // Create quadcopter simulator
  multicopterSim_ = new MulticopterDynamicsSim(4, thrustCoeff, torqueCoeff,
                        0., maxPropSpeed, motorTimeconstant, motorRotationalInertia,
                        vehicleMass, vehicleInertia,
                        aeroMomentCoefficient, dragCoeff, momentProcessNoiseAutoCorrelation,
                        forceProcessNoiseAutoCorrelation, gravity);

  // Set and publish motor transforms for the four motors
  Eigen::Isometry3d motorFrame = Eigen::Isometry3d::Identity();
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

  // Set initial conditions
  initPosition_ << initPose.at(0), initPose.at(1), initPose.at(2);
  initAttitude_.coeffs() << initPose.at(3), initPose.at(4), initPose.at(5), initPose.at(6);
  initAttitude_.normalize();
  initPropSpeed_ = sqrt(vehicleMass/4.*9.81/thrustCoeff);

  multicopterSim_->setVehiclePosition(initPosition_,initAttitude_);
  multicopterSim_->setMotorSpeed(initPropSpeed_);

  // Get and set IMU parameters
  double accBiasProcessNoiseAutoCorrelation, gyroBiasProcessNoiseAutoCorrelation;
  getParameter("accelerometer_biasprocess", accBiasProcessNoiseAutoCorrelation, 1.0e-7, "m^2/s^5");
  getParameter("gyroscope_biasprocess",     accBiasProcessNoiseAutoCorrelation, 1.0e-7, "rad^2/s^3");
  getParameter("accelerometer_biasinitvar", accBiasInitVar_,                    0.00015,"(m/s^2)^2");
  getParameter("gyroscope_biasinitvar",     gyroBiasInitVar_,                   0.00013,"(rad/s)^2");
  getParameter("accelerometer_variance",    accMeasNoiseVariance_,              0.001,  "m^2/s^4");
  getParameter("gyroscope_variance",        gyroMeasNoiseVariance_,             0.001,  "rad^2/s^2");
  multicopterSim_->imu_.setBias(accBiasInitVar_, gyroBiasInitVar_,
                                accBiasProcessNoiseAutoCorrelation, gyroBiasProcessNoiseAutoCorrelation);
  multicopterSim_->imu_.setNoiseVariance(accMeasNoiseVariance_, gyroMeasNoiseVariance_);

  // Get home (reference) position
  getParameter("lat_ref",                   latRef_,                            47.3977420);
  getParameter("lon_ref",                   lonRef_,                            8.5455940);
  getParameter("alt_ref",                   altRef_,                            488.157);

  multicopterSim_->geodetic_converter_.initialiseReference(latRef_, lonRef_, altRef_);

  // Only enable clock scaling when simtime is enabled.
  if (useSimTime_) {
    if (!ros::param::get(INNO_DYNAMICS_NS + "clockscale", clockScale)) {
      std::cout << "Using sim_time and did not get a clock scaling value. Defaulting to automatic clock scaling." << std::endl;
      useAutomaticClockscale_ = true;
    }
  }

  // Print several parameters to terminal
  std::cout << "Ignore collisions: " << ignoreCollisions_ << std::endl;
  std::cout << "Initial position: " << initPosition_(0) << ", "
                                    << initPosition_(1) << ", "
                                    << initPosition_(2) << std::endl;
  std::cout << "Initial attitude: " << initAttitude_.coeffs()(0) << ", "
                                    << initAttitude_.coeffs()(1) << ", "
                                    << initAttitude_.coeffs()(2) << ", "
                                    << initAttitude_.coeffs()(3) << std::endl;

  // Init subscribers and publishers
  /*Allow for up to 100ms sim time buffer of outgoing IMU messages. 
    This should improve IMU integration methods on slow client nodes (see issue #63). */
  imuPub_ = node_.advertise<sensor_msgs::Imu>("/uav/sensors/imu", 96);  
  positionPub_ = node_.advertise<geometry_msgs::Pose>("/uav/position", 1);
  speedPub_ = node_.advertise<geometry_msgs::Twist>("/uav/speed", 1);
  inputCommandSub_ = node_.subscribe("/uav/input/rateThrust", 1, &Uav_Dynamics::inputCallback, this);
  inputMotorspeedCommandSub_ = node_.subscribe("/uav/input/motorspeed", 1, &Uav_Dynamics::inputMotorspeedCallback, this);
  collisionSub_ = node_.subscribe("/uav/collision", 1, &Uav_Dynamics::collisionCallback, this);
  armSub_ = node_.subscribe("/uav/input/arm", 1, &Uav_Dynamics::armCallback, this);
  resetSub_ = node_.subscribe("/uav/input/reset", 1, &Uav_Dynamics::resetCallback, this);
  frameRateSub_ = node_.subscribe("/uav/camera/debug/fps", 1, &Uav_Dynamics::fpsCallback, this);

  if (useSimTime_) {
    clockPub_ = node_.advertise<rosgraph_msgs::Clock>("/clock",1);
    clockPub_.publish(currentTime_);
  } else {
    // Get the current time if we are using wall time. Otherwise, use 0 as initial clock.
    currentTime_ = ros::Time::now();
  }

  px4 = new PX4Communicator(altRef_);
  int px4id = 0;
  if (px4->Init(px4id, multicopterSim_) != 0) {
		std::cerr << "Unable to Init PX4 Communication" << std::endl;
	}

  // Init main simulation loop
  simulationLoopTimer_ = node_.createWallTimer(ros::WallDuration(dt_secs/clockScale), &Uav_Dynamics::simulationLoopTimerCallback, this);
  simulationLoopTimer_.start();
}

/**
 * @brief Callback to handle the frame rate from unity
 * @param msg Float msg of frame rate in sim time from unity
 */
void Uav_Dynamics::fpsCallback(std_msgs::Float32::Ptr msg) { 
  actualFps = msg->data; 
}

/**
 * @brief Main Simulator loop
 * @param event Wall clock timer event
 */
void Uav_Dynamics::simulationLoopTimerCallback(const ros::WallTimerEvent& event){
  // Step the time forward
  if (useSimTime_){
    currentTime_ += ros::Duration(dt_secs);
    clockPub_.publish(currentTime_);
  } else {
      ros::Time loopStartTime = ros::Time::now();
      dt_secs = (loopStartTime - currentTime_).toSec();
      currentTime_ = loopStartTime;
  }

  // In case of collision reset state and disarm
  if(resetRequested_ || (hasCollided_ && !ignoreCollisions_)){
    lpf_.resetState();
    pid_.resetState();
    resetState();
    lastCommandMsg_.reset();
    lastMotorspeedCommandMsg_.reset();
    hasCollided_ = false;
    armed_= false;
    resetRequested_ = false;
    return;
  }

  std::vector<double> propSpeedCommand(4, 0.);

  // The sequence of steps for lockstep are:
  // The simulation sends a sensor message HIL_SENSOR including a timestamp time_usec to update the sensor state and time of PX4.
  // PX4 receives this and does one iteration of state estimation, controls, etc. and eventually sends an actuator message HIL_ACTUATOR_CONTROLS.
  // The simulation waits until it receives the actuator/motor message, then simulates the physics and calculates the next sensor message to send to PX4 again.
  // The system starts with a "freewheeling" period where the simulation sends sensor messages including time and therefore runs PX4 until it has initialized and responds with an actautor message.

	bool px4Recved = false;
  // std::vector<double> commandPercent(4, 0.0);
  
  px4->Send(currentTime_.toNSec() / 1000);
  if(receivedPX4Actuator)
  {
    for (size_t i = 0; i < 1; i++)
    {
      px4Recved = (px4->Receive(false, armed_, propSpeedCommand) == 1);
      // usleep(500);
      if(px4Recved)
        break;
    }
  }
  else
  {
    px4Recved = (px4->Receive(false, armed_, propSpeedCommand) == 1);
    if(px4Recved)
      receivedPX4Actuator = true;
  }

  if(!px4Recved)
    return;


  // Only propagate simulation if armed
  if (true /*armed_*/) {  

    

    // if(useRateThrustController_){
    //   // Proceed LPF state based on gyro measurement
    //   lpf_.proceedState(imuGyroOutput_, dt_secs);

    //   // PID compute motor speed commands
    //   if(lastCommandMsg_){
    //     pid_.controlUpdate(lastCommandMsg_->angular_rates, lastCommandMsg_->thrust.z,
    //                       lpf_.filterState_, lpf_.filterStateDer_,
    //                       propSpeedCommand, dt_secs);
    //   }
    // }
    // else{
    //   if(lastMotorspeedCommandMsg_){
    //     for (size_t motorIndx = 0; motorIndx < 4; motorIndx++){
    //       propSpeedCommand.at(motorIndx) = lastMotorspeedCommandMsg_->angular_velocities[motorIndx];
    //     }
    //   }
    // }

    // Proceed quadcopter dynamics
    if(useRungeKutta4Integrator_){
      multicopterSim_->proceedState_RK4(dt_secs, propSpeedCommand, true); 
    }
    else{
      multicopterSim_->proceedState_ExplicitEuler(dt_secs, propSpeedCommand, true);
    }

    // Get IMU measurements
    multicopterSim_->getIMUMeasurement(imuAccOutput_, imuGyroOutput_);

    // Publish IMU measurements
    publishIMUMeasurement();
  }
  publishUavPosition();
  publishUavSpeed();

  // Publish quadcopter state
  publishState();
  // std::cout << "[" << currentTime_ <<"]: position " << multicopterSim_->getVehiclePosition().transpose() << std::endl;

  // Update clockscale if necessary
  if (actualFps != -1 && actualFps < 1e3 && useSimTime_ && useAutomaticClockscale_) {
     clockScale =  (actualFps / 55.0);
     simulationLoopTimer_.stop();
     simulationLoopTimer_.setPeriod(ros::WallDuration(dt_secs / clockScale));
     simulationLoopTimer_.start();
  }
}

/**
 * @brief Handle incoming rate thrust message
 * @param msg rateThrust message from keyboard/ joystick controller
 */
void Uav_Dynamics::inputCallback(mav_msgs::RateThrust::Ptr msg){
	lastCommandMsg_ = msg;
}

/**
 * @brief Handle arming message
 * @param msg Empty message, this will be received when drone is to be armed
 */
void Uav_Dynamics::armCallback(std_msgs::Empty::Ptr msg){
	armed_ = true;
}

/**
 * @brief Handle reset message
 * @param msg Empty message, this will be received when drone is to be reset
 */
void Uav_Dynamics::resetCallback(std_msgs::Empty::Ptr msg){
	resetRequested_ = true;
}

/**
 * @brief Handle incoming motor speed command message
 * @param msg Actuators message containing the motor speed commands
 */
void Uav_Dynamics::inputMotorspeedCallback(mav_msgs::Actuators::Ptr msg){
	lastMotorspeedCommandMsg_ = msg;
}

/**
 * @brief Handle the checking of collisions
 * @param msg Empty message, this will be received when a collision is detected
 */
void Uav_Dynamics::collisionCallback(std_msgs::Empty::Ptr msg){
  hasCollided_ = true;
}

/**
 * @brief Reset state to initial
 */
void Uav_Dynamics::resetState(void){
  multicopterSim_->setVehiclePosition(initPosition_,initAttitude_);
  multicopterSim_->setMotorSpeed(initPropSpeed_);
  multicopterSim_->imu_.setBias(accBiasInitVar_, gyroBiasInitVar_);
}

/**
 * @brief Publish UAV state transform message
 */
void Uav_Dynamics::publishState(void){
  geometry_msgs::TransformStamped transform;

  transform.header.stamp = currentTime_;
  transform.header.frame_id = "world";

  Eigen::Vector3d position = multicopterSim_->getVehiclePosition();
  Eigen::Quaterniond attitude = multicopterSim_->getVehicleAttitude();

  transform.transform.translation.x = position(0);
  transform.transform.translation.y = position(1);
  transform.transform.translation.z = position(2);

  transform.transform.rotation.x = attitude.x();
  transform.transform.rotation.y = attitude.y();
  transform.transform.rotation.z = attitude.z();
  transform.transform.rotation.w = attitude.w();

  transform.child_frame_id = "uav/imu";

  tfPub_.sendTransform(transform);
}

/**
 * @brief Publish IMU measurement message
 */
void Uav_Dynamics::publishIMUMeasurement(void){

  sensor_msgs::Imu meas;

  meas.header.stamp = currentTime_;

  // Per message spec: set to -1 since orientation is not populated
  meas.orientation_covariance[0] = -1;

  meas.angular_velocity.x = imuGyroOutput_(0);
  meas.linear_acceleration.x = imuAccOutput_(0);

  meas.angular_velocity.y = imuGyroOutput_(1);
  meas.linear_acceleration.y = imuAccOutput_(1);

  meas.angular_velocity.z = imuGyroOutput_(2);
  meas.linear_acceleration.z = imuAccOutput_(2);

  meas.angular_velocity_covariance[0] = gyroMeasNoiseVariance_;
  meas.linear_acceleration_covariance[0] = accMeasNoiseVariance_;
  for (size_t i = 1; i < 8; i++){
    if (i == 4){
      meas.angular_velocity_covariance[i] = gyroMeasNoiseVariance_;
      meas.linear_acceleration_covariance[i] = accMeasNoiseVariance_;
    }
    else{
      meas.angular_velocity_covariance[i] = 0.;
      meas.linear_acceleration_covariance[i] = 0.;
    }
  }

  meas.angular_velocity_covariance[8] = gyroMeasNoiseVariance_;
  meas.linear_acceleration_covariance[8] = accMeasNoiseVariance_;

  imuPub_.publish(meas);
}

/**
 * @brief Publish position message
 */
void Uav_Dynamics::publishUavPosition(void){
    auto position = multicopterSim_->getVehiclePosition().transpose();
    auto euler_angles = multicopterSim_->getVehicleAttitude().toRotationMatrix().eulerAngles(0, 1, 2);
  
    geometry_msgs::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
    pose.orientation.x = euler_angles[0];
    pose.orientation.y = euler_angles[1];
    pose.orientation.z = euler_angles[2];
    positionPub_.publish(pose);
}

/**
 * @brief Publish position message
 */
void Uav_Dynamics::publishUavSpeed(void){
    auto velocity = multicopterSim_->getVehicleVelocity();
    auto angular_velocity = multicopterSim_->getVehicleAngularVelocity();

    geometry_msgs::Twist speed;
    speed.linear.x = velocity[0];
    speed.linear.y = velocity[1];
    speed.linear.z = velocity[2];
    speed.angular.x = angular_velocity[0];
    speed.angular.y = angular_velocity[1];
    speed.angular.z = angular_velocity[2];
    speedPub_.publish(speed);
}

/**
 * @brief Publish static transform from UAV centroid to motor
 * 
 * @param timeStamp Tf timestamp
 * @param frame_id Parent (UAV) frame ID
 * @param child_frame_id Child (motor) frame ID
 * @param motorFrame Transformation
 */
void Uav_Dynamics::publishStaticMotorTransform(
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

/**
 * @brief Construct a new Uav_LowPassFilter::Uav_LowPassFilter object
 * 
 */
Uav_LowPassFilter::Uav_LowPassFilter(){

  double temp1;
  double temp2;

  // Get filter gains
  if ((!ros::param::get("/uav/flightgoggles_lpf/gain_p", temp1)) ||
      (!ros::param::get("/uav/flightgoggles_lpf/gain_q", temp2)))
  { 
      std::cout << "Did not get the LPF gain_p and/or gain_q from the params, defaulting to 30 Hz cutoff freq." << std::endl;
  }
  else
  {
    gainP_ = temp1;
    gainQ_ = temp2;
  }
}

/**
 * @brief Propagates the state using trapezoidal integration
 * @param input Filter input
 * @param dt Time step
 */
void Uav_LowPassFilter::proceedState(Eigen::Vector3d & input, double dt){

  double det = gainP_ * dt * dt + gainQ_ * dt + 1.;
  double stateDer;
  for (size_t ind = 0; ind < 3; ind++) {
    stateDer = (filterStateDer_[ind] + gainP_ * dt * input(ind)) / det -
               (dt * gainP_ * filterState_[ind]) / det;
    filterState_[ind] =
        (dt * (filterStateDer_[ind] + gainP_ * dt * input(ind))) / det +
        ((dt * gainQ_ + 1.) * filterState_[ind]) / det;
    filterStateDer_[ind] = stateDer;
  }
}

/**
 * @brief Reset the LPF state to zeros
 */
void Uav_LowPassFilter::resetState(void){
  for (size_t ind = 0; ind < 3; ind++) {
    filterState_[ind] = 0.;
    filterStateDer_[ind] = 0.;
  }
}

/**
 * @brief Construct a new Uav_Pid::Uav_Pid object
 * 
 */
Uav_Pid::Uav_Pid(){
  getParameter("gain_p_roll",         propGain_[0],       propGain_[0]);
  getParameter("gain_i_roll",         intGain_[0],        intGain_[0]);
  getParameter("gain_d_roll",         derGain_[0],        derGain_[0]);

  getParameter("gain_p_pitch",        propGain_[1],       propGain_[1]);
  getParameter("gain_i_pitch",        intGain_[1],        intGain_[1]);
  getParameter("gain_d_pitch",        derGain_[1],        derGain_[1]);

  getParameter("gain_p_yaw",          propGain_[2],       propGain_[2]);
  getParameter("gain_i_yaw",          intGain_[2],        intGain_[2]);
  getParameter("gain_d_yaw",          derGain_[2],        derGain_[2]);

  getParameter("int_bound_roll",      intBound_[0],       intBound_[0]);
  getParameter("int_bound_pitch",     intBound_[1],       intBound_[1]);
  getParameter("int_bound_yaw",       intBound_[2],       intBound_[2]);

  getParameter("vehicle_inertia_xx",  vehicleInertia_[0], vehicleInertia_[0]);
  getParameter("vehicle_inertia_yy",  vehicleInertia_[1], vehicleInertia_[1]);
  getParameter("vehicle_inertia_zz",  vehicleInertia_[2], vehicleInertia_[2]);

  getParameter("moment_arm",          momentArm_,         momentArm_);
  getParameter("thrust_coefficient",  thrustCoeff_,       thrustCoeff_);
  getParameter("torque_coefficient",  torqueCoeff_,       torqueCoeff_);
}

/**
 * @brief Compute motor speed commands based on angular rate and thrust inputs
 * 
 * @param command Angular rate command
 * @param thrustCommand Thrust command
 * @param curval Current vehicle angular rate
 * @param curder Current vehicle angular acceleration
 * @param propSpeedCommand Motor speed command vector output
 * @param dt Time step used for PID integrators
 */
void Uav_Pid::controlUpdate(geometry_msgs::Vector3 & command, double thrustCommand,
                      double * curval, double * curder,
                      std::vector<double> & propSpeedCommand, double dt){

  double angAccCommand[3];

  double stateDev[] = {command.x-curval[0], command.y-curval[1], command.z-curval[2]};

  for (size_t ind = 0; ind < 3; ind++) {
    intState_[ind] += dt * stateDev[ind];
    intState_[ind] = fmin(fmax(-intBound_[ind],intState_[ind]),intBound_[ind]);
    angAccCommand[ind] = propGain_[ind] * stateDev[ind] +
                         intGain_[ind] * intState_[ind] + derGain_[ind] * -curder[ind];
  }

  thrustMixing(propSpeedCommand,angAccCommand,thrustCommand);
}

/**
 * @brief Compute motor speed commands based on angular acceleration and thrust commands
 * 
 * @param propSpeedCommand Motor speed command vector output
 * @param angAccCommand Angular acceleration command
 * @param thrustCommand Thurst command
 */
void Uav_Pid::thrustMixing(std::vector<double> & propSpeedCommand, double * angAccCommand, double thrustCommand){

  // Compute torqu and thrust vector
  double momentThrust[4] = {
    vehicleInertia_[0]*angAccCommand[0],
    vehicleInertia_[1]*angAccCommand[1],
    vehicleInertia_[2]*angAccCommand[2],
    thrustCommand
  };

  // Compute signed, squared motor speed values
  double motorSpeedsSquared[4] = {
    momentThrust[0]/(4*momentArm_*thrustCoeff_)+ -momentThrust[1]/(4*momentArm_*thrustCoeff_)+ -momentThrust[2]/(4*torqueCoeff_)+ momentThrust[3]/(4*thrustCoeff_),
    momentThrust[0]/(4*momentArm_*thrustCoeff_)+  momentThrust[1]/(4*momentArm_*thrustCoeff_)+  momentThrust[2]/(4*torqueCoeff_)+ momentThrust[3]/(4*thrustCoeff_),
   -momentThrust[0]/(4*momentArm_*thrustCoeff_)+  momentThrust[1]/(4*momentArm_*thrustCoeff_)+ -momentThrust[2]/(4*torqueCoeff_)+ momentThrust[3]/(4*thrustCoeff_),
   -momentThrust[0]/(4*momentArm_*thrustCoeff_)+ -momentThrust[1]/(4*momentArm_*thrustCoeff_)+  momentThrust[2]/(4*torqueCoeff_)+ momentThrust[3]/(4*thrustCoeff_)
  };

  // Compute signed motor speed values
  for(size_t i = 0; i < 4; i++)
    propSpeedCommand.at(i) = copysign(sqrt(fabs(motorSpeedsSquared[i])),motorSpeedsSquared[i]);
}

/**
 * @brief Reset PID controller integrator state
 * 
 */
void Uav_Pid::resetState(void){
  for (size_t ind = 0; ind < 3; ind++) {
    intState_[ind] = 0.;
  }
}
