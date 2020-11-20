/**
 * @file flightgoggles_uav_dynamics_node.hpp
 * @author Ezra Tal
 * @author Winter Guerra
 * @author Varun Murali
 * @brief Header file for UAV dynamics, IMU, and angular rate control simulation node
 * 
 */

#ifndef UAV_DYNAMICS_HPP
#define UAV_DYNAMICS_HPP

#include <thread>
#include <sstream>
#include <random>

// ROS includes
#include <ros/ros.h>
#include <ros/time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

// Messages
#include <mav_msgs/RateThrust.h>
#include <mav_msgs/Actuators.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <rosgraph_msgs/Clock.h>

#include "px4_communicator.h"
#include "uavDynamicsSimBase.hpp"
#include "multicopterDynamicsSimWrapper.hpp"


/**
 * @brief UAV Dynamics class used for dynamics, IMU, and angular rate control simulation node
 */
class Uav_Dynamics {
    public:
        Uav_Dynamics(ros::NodeHandle nh);
        /**
         * @return -1 if error occured, else 0
         */
        int8_t init();

        ros::NodeHandle node_;

        tf2_ros::TransformBroadcaster tfPub_;

        ros::Publisher imuPub_;
        ros::Publisher positionPub_;
        ros::Publisher speedPub_;
        ros::Publisher controlPub_;
        ros::Publisher clockPub_;
        ros::Publisher threadPub_;


        void publishState();
        void publishIMUMeasurement();
        void publishUavPosition();
        void publishUavSpeed();
        void publishControl();
        void publishThreadsInfo();
        void publishStaticMotorTransform(const ros::Time & timeStamp,
                                         const char * frame_id,
                                         const char * child_frame_id,
                                         const Eigen::Isometry3d & motorFrame);

        /// @name Subscribers
        //@{
        ros::Subscriber inputCommandSub_;
        ros::Subscriber inputMotorspeedCommandSub_;
        ros::Subscriber collisionSub_;
        ros::Subscriber armSub_;
        ros::Subscriber resetSub_;
	    ros::Subscriber frameRateSub_;
        //@}

        /// @name Timers
        //@{
        ros::WallTimer simulationLoopTimer_;
        //@}

        /// @name Callbacks
        //@{
        void simulationLoopTimerCallback(const ros::WallTimerEvent& event);
        void inputCallback(mav_msgs::RateThrust::Ptr msg);
        void inputMotorspeedCallback(mav_msgs::Actuators::Ptr msg);
        void armCallback(std_msgs::Empty::Ptr msg);
        void resetCallback(std_msgs::Empty::Ptr msg);
        void collisionCallback(std_msgs::Empty::Ptr msg);
	    void fpsCallback(std_msgs::Float32::Ptr msg);
        //@}

        /// @name Control inputs
        //@{
        mav_msgs::RateThrust::Ptr lastCommandMsg_;
        mav_msgs::Actuators::Ptr lastMotorspeedCommandMsg_;
        //@}

        /// @name Time keeping variables
        //@{
        ros::Time currentTime_;
        double dt_secs = 1.0f/960.;
        bool useAutomaticClockscale_ = false;
        double clockScale_ = 1.0;
        double actualFps_  = -1;
        bool useSimTime_ = false;
        //@}

        /// @name Simulation state control parameters and flags
        //@{
        bool hasCollided_ = false;
        bool ignoreCollisions_ = false;
	    bool armed_ = false;
        bool resetRequested_ = false;
        bool useRateThrustController_ = true;
        bool useRungeKutta4Integrator_ = false;
        //@}

        bool receivedPX4Actuator_ = false;

        void resetState(void);

        /// @name Vehicle dynamics simulator
        //@{
        /* As standard in ROS:
        x : forward
        y : leftward
        z : upward */

        /* Motors are numbered counterclockwise (look at quadcopter from above) with
           motor 1 in the positive quadrant of the X-Y plane (i.e. front left). */

        UavDynamicsSimBase* uavDynamicsSim_;
        PX4Communicator *px4;
        //@}

        /// @name IMU measurements and variances
        //@{
        Eigen::Vector3d imuAccOutput_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d imuGyroOutput_ = Eigen::Vector3d::Zero();
        double gyroMeasNoiseVariance_;
        double accMeasNoiseVariance_;
        double accBiasInitVar_;
        double gyroBiasInitVar_;
        //@}

        double latRef_;
        double lonRef_;
        double altRef_;

        std::vector<double> propSpeedCommand_;

    private:
        void proceedQuadcopterDynamics(double period);
        void sendHilGps(double period);
        void sendHilSensor(double period);
        void receive(double period);
        void publishToRos(double period);
        std::thread proceedDynamicsTask;
        std::thread sendHilGpsTask;
        std::thread sendHilSensorTask;
        std::thread receiveTask;
        std::thread publishToRosTask;

        std::array<int64_t, 6> threadCounter;
};

#endif
