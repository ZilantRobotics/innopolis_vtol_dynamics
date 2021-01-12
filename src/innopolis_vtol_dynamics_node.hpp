/**
 * @file innopolis_vtol_dynamics_node.hpp
 * @author Dmitry Ponomarev
 * @author Roman Fedorenko
 * @author Ezra Tal
 * @author Winter Guerra
 * @author Varun Murali
 * @brief Header file for UAV dynamics, IMU, and angular rate control simulation node
 */

#ifndef UAV_DYNAMICS_HPP
#define UAV_DYNAMICS_HPP

#include <thread>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mav_msgs/RateThrust.h>
#include <mav_msgs/Actuators.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

#include "mavlink_communicator.h"
#include "uavDynamicsSimBase.hpp"


/**
 * @brief UAV Dynamics class used for dynamics, IMU, and angular rate control simulation node
 */
class Uav_Dynamics {
    public:
        explicit Uav_Dynamics(ros::NodeHandle nh);
        int8_t init();

    private:
        /// @name Simulator
        //@{
        ros::NodeHandle node_;
        UavDynamicsSimBase* uavDynamicsSim_;
        MavlinkCommunicator* px4;
        ros::Publisher clockPub_;

        ros::Time currentTime_;
        double dt_secs_ = 1.0f/960.;
        bool useAutomaticClockscale_ = false;
        double clockScale_ = 1.0;
        double actualFps_  = -1;
        bool useSimTime_;

        bool armed_ = false;
        bool receivedPX4Actuator_ = false;

        double latRef_;
        double lonRef_;
        double altRef_;

        enum DynamicsType{
            FLIGHTGOGGLES_MULTICOPTER = 0,
            INNO_VTOL,
        };
        enum AirframeType{
            IRIS = 0,
            STANDARD_VTOL,
        };

        DynamicsType dynamicsType_;
        AirframeType airframeType_;
        //@}


        /// @name Communication with PX4
        //@{
        ros::Publisher positionPub_;
        ros::Publisher speedPub_;
        ros::Publisher controlPub_;
        ros::Publisher imuPub_;
        ros::Subscriber inputMotorspeedCommandSub_;

        std::vector<double> propSpeedCommand_;
        mav_msgs::Actuators::Ptr lastMotorspeedCommandMsg_;

        void publishUavPosition();
        void publishIMUMeasurement();
        void publishUavSpeed();
        //@}


        /// @name Visualization (Markers and tf)
        //@{
        tf2_ros::TransformBroadcaster tfPub_;

        ros::Publisher totalForcePub_;
        ros::Publisher aeroForcePub_;
        ros::Publisher motorsForcesPub_[5];
        ros::Publisher liftForcePub_;
        ros::Publisher drugForcePub_;
        ros::Publisher sideForcePub_;

        ros::Publisher totalMomentPub_;
        ros::Publisher aeroMomentPub_;
        ros::Publisher motorsMomentsPub_[5];

        ros::Publisher velocityPub_;

        ros::Publisher forcesPub_;

        void publishState();
        void publishControl();
        void publishForcesInfo();
        void publishStaticMotorTransform(const ros::Time & timeStamp,
                                         const char * frame_id,
                                         const char * child_frame_id,
                                         const Eigen::Isometry3d & motorFrame);
        //@}


        /// @name Timer and threads
        //@{
        ros::WallTimer simulationLoopTimer_;

        std::thread proceedDynamicsTask;
        std::thread sendHilGpsTask;
        std::thread sendHilSensorTask;
        std::thread receiveTask;
        std::thread publishToRosTask;

        const float HIL_GPS_PERIOD_SEC = 0.1;
        const float HIL_SENSOR_PERIOD_SEC = 0.0012;
        const float RECEIVE_PERIOD_SEC = 0.0051;
        const float ROS_PUB_PERIOD_SEC = 0.05;

        std::array<int64_t, 6> threadCounter_;

        void simulationLoopTimerCallback(const ros::WallTimerEvent& event);
        void proceedQuadcopterDynamics(double period);
        void sendHilGps(double period);
        void sendHilSensor(double period);
        void receive(double period);
        void publishToRos(double period);
        //@}


        /// @name Not implemented yet stuff
        //@{
        ros::Subscriber inputCommandSub_;
        ros::Subscriber collisionSub_;
        ros::Subscriber armSub_;
        ros::Subscriber resetSub_;
        ros::Subscriber frameRateSub_;

        bool hasCollided_ = false;
        bool ignoreCollisions_;
        bool resetRequested_ = false;

        mav_msgs::RateThrust::Ptr lastCommandMsg_;

        void inputCallback(mav_msgs::RateThrust::Ptr msg);
        void inputMotorspeedCallback(mav_msgs::Actuators::Ptr msg);
        void armCallback(std_msgs::Empty::Ptr msg);
        void resetCallback(std_msgs::Empty::Ptr msg);
        void collisionCallback(std_msgs::Empty::Ptr msg);
        void fpsCallback(std_msgs::Float32::Ptr msg);
        void resetState(void);
        //@}
};

#endif
