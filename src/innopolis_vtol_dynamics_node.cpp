/**
 * @file innopolis_vtol_dynamics_node.cpp
 * @author Roman Fedorenko
 * @author Ezra Tal
 * @author Winter Guerra
 * @author Varun Murali
 * @brief Implementation of UAV dynamics, IMU, and angular rate control simulation node
 */

#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include "innopolis_vtol_dynamics_node.hpp"
#include "vtolDynamicsSim.hpp"


int main(int argc, char **argv){
    ros::init(argc, argv, "innopolis_vtol_dynamics_node");
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::NodeHandle node_handler;
    Uav_Dynamics uav_dynamics_node(node_handler);
    if(uav_dynamics_node.init() == -1){
        ros::shutdown();
        return 0;
    }
    ros::spin();
    return 0;
}


Uav_Dynamics::Uav_Dynamics(ros::NodeHandle nh): node_(nh), propSpeedCommand_(4, 0.){

}


int8_t Uav_Dynamics::init(){
    // Get Simulator parameters
    std::vector<double> initPose(7);
    std::string dynamics_type;
    const std::string SIM_PARAMS_PATH = "/uav/sim_params/";
    if(!ros::param::get(SIM_PARAMS_PATH + "ignore_collisions",  ignoreCollisions_)  ||
       !ros::param::get(SIM_PARAMS_PATH + "use_sim_time",       useSimTime_ ) ||
       !ros::param::get(SIM_PARAMS_PATH + "lat_ref",            latRef_) ||
       !ros::param::get(SIM_PARAMS_PATH + "lon_ref",            lonRef_) ||
       !ros::param::get(SIM_PARAMS_PATH + "alt_ref",            altRef_) ||
       !ros::param::get(SIM_PARAMS_PATH + "dynamics_type",      dynamics_type) ||
       !ros::param::get(SIM_PARAMS_PATH + "init_pose",          initPose)){
        ROS_ERROR("There is no at least one of required simulator parameters.");
        return -1;
    }
    /**
     * @brief Init dynamics simulator
     * @todo it's better to use some build method instead of manually call new
     */
    if(dynamics_type == "flightgoggles_multicopter"){
        uavDynamicsSim_ = new MulticopterDynamicsWrapper;
    }else if(dynamics_type == "inno_vtol"){
        uavDynamicsSim_ = new VtolDynamicsSim;
    }else{
        ROS_ERROR("Dynamics type with name \"%s\" is not exist.", dynamics_type.c_str());
        return -1;
    }
    if(uavDynamicsSim_ == nullptr || uavDynamicsSim_->init() == -1){
        ROS_ERROR("Can't init uav dynamics sim. Shutdown.");
        return -1;
    }
    uavDynamicsSim_->initStaticMotorTransform();
    uavDynamicsSim_->setReferencePosition(latRef_, lonRef_, altRef_);

    Eigen::Vector3d initPosition(initPose.at(0), initPose.at(1), initPose.at(2));
    Eigen::Quaterniond initAttitude(initPose.at(6), initPose.at(3), initPose.at(4), initPose.at(5));
    initAttitude.normalize();
    uavDynamicsSim_->setInitialPosition(initPosition, initAttitude);

    if(useSimTime_){
        if (!ros::param::get(SIM_PARAMS_PATH + "clockscale", clockScale_)) {
            std::cout << "Using sim_time and did not get a clock scaling value." << 
                         "Defaulting to automatic clock scaling." << std::endl;
            useAutomaticClockscale_ = true;
        }
    }

    // Init subscribers and publishers
    // Allow for up to 100ms sim time buffer of outgoing IMU messages
    // This should improve IMU integration methods on slow client nodes (see issue #63)
    imuPub_ = node_.advertise<sensor_msgs::Imu>("/uav/sensors/imu", 96);
    positionPub_ = node_.advertise<geometry_msgs::Pose>("/uav/position", 1);
    speedPub_ = node_.advertise<geometry_msgs::Twist>("/uav/speed", 1);
    controlPub_ = node_.advertise<std_msgs::Float64MultiArray>("/uav/control", 1);
    threadPub_ = node_.advertise<std_msgs::Float64MultiArray>("/uav/threads_info", 1);
    inputCommandSub_ = node_.subscribe("/uav/input/rateThrust", 1, &Uav_Dynamics::inputCallback, this);
    inputMotorspeedCommandSub_ = node_.subscribe("/uav/input/motorspeed", 1, &Uav_Dynamics::inputMotorspeedCallback, this);
    collisionSub_ = node_.subscribe("/uav/collision", 1, &Uav_Dynamics::collisionCallback, this);
    armSub_ = node_.subscribe("/uav/input/arm", 1, &Uav_Dynamics::armCallback, this);
    resetSub_ = node_.subscribe("/uav/input/reset", 1, &Uav_Dynamics::resetCallback, this);
    frameRateSub_ = node_.subscribe("/uav/camera/debug/fps", 1, &Uav_Dynamics::fpsCallback, this);

    if(useSimTime_){
        clockPub_ = node_.advertise<rosgraph_msgs::Clock>("/clock", 1);
        clockPub_.publish(currentTime_);
    }else{
        // Get the current time if we are using wall time. Otherwise, use 0 as initial clock.
        currentTime_ = ros::Time::now();
    }
  
    px4 = new PX4Communicator(altRef_);
    int px4id = 0;
    if(px4->Init(px4id, uavDynamicsSim_) != 0) {
        std::cerr << "Unable to Init PX4 Communication" << std::endl;
        return -1;
    }

    simulationLoopTimer_ = node_.createWallTimer(ros::WallDuration(dt_secs/clockScale_), &Uav_Dynamics::simulationLoopTimerCallback, this);
    simulationLoopTimer_.start();

    proceedDynamicsTask = std::thread(&Uav_Dynamics::proceedQuadcopterDynamics, this, dt_secs);
    proceedDynamicsTask.detach();

    sendHilGpsTask = std::thread(&Uav_Dynamics::sendHilGps, this, 0.1);
    sendHilGpsTask.detach();

    sendHilSensorTask = std::thread(&Uav_Dynamics::sendHilSensor, this, 0.001);
    sendHilSensorTask.detach();

    receiveTask = std::thread(&Uav_Dynamics::receive, this, 0.001);
    receiveTask.detach();

    publishToRosTask = std::thread(&Uav_Dynamics::publishToRos, this, 0.05);
    publishToRosTask.detach();

    return 0;
}

/**
 * @brief Callback to handle the frame rate from unity
 * @param msg Float msg of frame rate in sim time from unity
 */
void Uav_Dynamics::fpsCallback(std_msgs::Float32::Ptr msg) { 
    actualFps_ = msg->data;
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
        resetState();
        lastCommandMsg_.reset();
        lastMotorspeedCommandMsg_.reset();
        hasCollided_ = false;
        armed_= false;
        resetRequested_ = false;
        return;
    }

    // Update clockscale if necessary
    if (actualFps_ != -1 && actualFps_ < 1e3 && useSimTime_ && useAutomaticClockscale_) {
        clockScale_ =  (actualFps_ / 55.0);
        simulationLoopTimer_.stop();
        simulationLoopTimer_.setPeriod(ros::WallDuration(dt_secs / clockScale_));
        simulationLoopTimer_.start();
    }
}

void Uav_Dynamics::sendHilSensor(double period){
    while(ros::ok()){
        threadCounter[2]++;

        auto crnt_time = std::chrono::system_clock::now();
        auto sleed_period = std::chrono::microseconds(int(1000000 * period * clockScale_));
        auto time_point = crnt_time + sleed_period;
        int send_status = px4->SendHilSensor(currentTime_.toNSec() / 1000);
        if(send_status == -1){
            ROS_ERROR_STREAM_THROTTLE(1, "PX4 Communicator: Sent to PX4 failed" << strerror(errno));
        }
        std::this_thread::sleep_until(time_point);
    }
}

void Uav_Dynamics::sendHilGps(double period){
    while(ros::ok()){
        threadCounter[1]++;

        auto crnt_time = std::chrono::system_clock::now();
        auto sleed_period = std::chrono::microseconds(int(1000000 * period * clockScale_));
        auto time_point = crnt_time + sleed_period;
        int send_status = px4->SendHilGps(currentTime_.toNSec() / 1000);
        if(send_status == -1){
            ROS_ERROR_STREAM_THROTTLE(1, "PX4 Communicator: Send to PX4 failed" << strerror(errno));
        }
        std::this_thread::sleep_until(time_point);
    }
}

void Uav_Dynamics::publishToRos(double period){
    while(ros::ok()){
        threadCounter[4]++;

        auto crnt_time = std::chrono::system_clock::now();
        auto sleed_period = std::chrono::microseconds(int(1000000 * period * clockScale_));
        auto time_point = crnt_time + sleed_period;
        std::this_thread::sleep_until(time_point);
        publishState();
        publishIMUMeasurement();
        publishUavPosition();
        publishUavSpeed();
        publishControl();

        static auto next_time = std::chrono::system_clock::now();
        if(crnt_time > next_time){
            publishThreadsInfo();
            next_time += std::chrono::milliseconds(int(1000));
        }
    }
}

// The sequence of steps for lockstep are:
// The simulation sends a sensor message HIL_SENSOR including a timestamp time_usec to update
// the sensor state and time of PX4.
// PX4 receives this and does one iteration of state estimation, controls, etc. and eventually
// sends an actuator message HIL_ACTUATOR_CONTROLS.
// The simulation waits until it receives the actuator/motor message, then simulates the physics
// and calculates the next sensor message to send to PX4 again.
// The system starts with a "freewheeling" period where the simulation sends sensor messages
// including time and therefore runs PX4 until it has initialized and responds with an actautor message.
void Uav_Dynamics::receive(double period){
    while(ros::ok()){
        threadCounter[3]++;

        auto crnt_time = std::chrono::system_clock::now();
        auto sleed_period = std::chrono::microseconds(int(1000000 * period * clockScale_));
        auto time_point = crnt_time + sleed_period;

        if(receivedPX4Actuator_){
            for (size_t i = 0; i < 1; i++){
                if(px4->Receive(false, armed_, propSpeedCommand_) == 1){
                    break;
                }
            }
        }else if(px4->Receive(false, armed_, propSpeedCommand_) == 1){
            receivedPX4Actuator_ = true;
        }

        std::this_thread::sleep_until(time_point);
    }
}

void Uav_Dynamics::proceedQuadcopterDynamics(double period){
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(int(10000)));
    while(ros::ok()){
        threadCounter[0]++;

        auto crnt_time = std::chrono::system_clock::now();
        auto sleed_period = std::chrono::microseconds(int(1000000 * period * clockScale_));
        auto time_point = crnt_time + sleed_period;

        if(receivedPX4Actuator_ && armed_){
            static auto prev_time = std::chrono::system_clock::now();
            static uint8_t counter = 0;
            counter++;
            auto time_now = std::chrono::system_clock::now();
            if(counter > 50){
                std::chrono::duration<double> delta_time = time_now - prev_time;
                counter = 0;
            }
            prev_time = std::chrono::system_clock::now();

            uavDynamicsSim_->process(dt_secs, propSpeedCommand_, true);
        }

        // test pitch
        // propSpeedCommand_[0] = 0.66;
        // propSpeedCommand_[1] = 0.64;
        // propSpeedCommand_[2] = 0.64;
        // propSpeedCommand_[3] = 0.66;
        // uavDynamicsSim_->process(dt_secs, propSpeedCommand_, true);

        // test roll
        // propSpeedCommand_[0] = 0.64;
        // propSpeedCommand_[1] = 0.64;
        // propSpeedCommand_[2] = 0.66;
        // propSpeedCommand_[3] = 0.66;
        // uavDynamicsSim_->process(dt_secs, propSpeedCommand_, true);

        // test yaw - should be clockwise
        // propSpeedCommand_[0] = 0.64;
        // propSpeedCommand_[1] = 0.66;
        // propSpeedCommand_[2] = 0.64;
        // propSpeedCommand_[3] = 0.66;
        // uavDynamicsSim_->process(dt_secs, propSpeedCommand_, true);

        // test yaw - should be counterclockwise
        // propSpeedCommand_[0] = 0.66;
        // propSpeedCommand_[1] = 0.64;
        // propSpeedCommand_[2] = 0.66;
        // propSpeedCommand_[3] = 0.64;
        // uavDynamicsSim_->process(dt_secs, propSpeedCommand_, true);

        std::this_thread::sleep_until(time_point);
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
    // do nothing now
}

/**
 * @brief Publish UAV state transform message
 */
void Uav_Dynamics::publishState(void){
    geometry_msgs::TransformStamped transform;

    transform.header.stamp = currentTime_;
    transform.header.frame_id = "world";

    Eigen::Vector3d position = uavDynamicsSim_->getVehiclePosition();
    Eigen::Quaterniond attitude = uavDynamicsSim_->getVehicleAttitude();

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

    uavDynamicsSim_->getIMUMeasurement(imuAccOutput_, imuGyroOutput_);

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
    geometry_msgs::Pose pose;

    auto position = uavDynamicsSim_->getVehiclePosition().transpose();
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];

    auto quant = uavDynamicsSim_->getVehicleAttitude();
    auto euler_angles = quant.toRotationMatrix().eulerAngles(1, 0, 2);
    for(int idx = 0; idx < 3; idx++){
        euler_angles[idx] = euler_angles[idx] * 180 / 3.14;
        if(euler_angles[idx] > 360){
            euler_angles[idx] -= 360;
        }else if(euler_angles[idx] < 0){
            euler_angles[idx] += 360;
        }
    }
    pose.orientation.x = euler_angles[0];
    pose.orientation.y = euler_angles[1];
    pose.orientation.z = euler_angles[2];

    positionPub_.publish(pose);
}

/**
 * @brief Publish position message
 */
void Uav_Dynamics::publishUavSpeed(void){
    auto velocity = uavDynamicsSim_->getVehicleVelocity();
    auto angular_velocity = uavDynamicsSim_->getVehicleAngularVelocity();

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
 * @brief Publish control
 */
void Uav_Dynamics::publishControl(void){
    std_msgs::Float64MultiArray cmd;
    cmd.data.resize(8);
    for(size_t idx = 0; idx < propSpeedCommand_.size(); idx++){
        cmd.data[idx] = propSpeedCommand_[idx];
    }
    controlPub_.publish(cmd);
}

/**
 * @brief Publish thread counters
 */
void Uav_Dynamics::publishThreadsInfo(void){
    std_msgs::Float64MultiArray times;
    times.data.resize(5);
    for(size_t idx = 0; idx < 5; idx++){
        times.data[idx] = 1.0 / threadCounter[idx];
        threadCounter[idx] = 0;
    }
    threadPub_.publish(times);
}