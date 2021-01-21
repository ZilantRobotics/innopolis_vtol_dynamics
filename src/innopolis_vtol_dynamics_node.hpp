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
#include <geographiclib_conversions/geodetic_conv.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mav_msgs/RateThrust.h>
#include <mav_msgs/Actuators.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>

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
        ros::Publisher clockPub_;

        ros::Time currentTime_;
        double dt_secs_ = 1.0f/960.;
        bool useAutomaticClockscale_ = false;
        double clockScale_ = 1.0;
        double actualFps_  = -1;
        bool useSimTime_;

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

        geodetic_converter::GeodeticConverter geodeticConverter_;
        //@}


        /// @name Communication with PX4
        //@{
        ros::Subscriber actuatorsSub_;
        ros::Subscriber armSub_;

        ros::Publisher attitudePub_;
        ros::Publisher imuPub_;
        ros::Publisher gpsPositionPub_;
        ros::Publisher speedPub_;
        ros::Publisher magPub_;

        std::vector<double> actuators_;
        uint64_t actuatorsTimestampSec_;
        bool armed_ = false;

        void armCallback(std_msgs::Bool msg);
        void actuatorsCallback(sensor_msgs::Joy::Ptr msg);

        void publishUavGpsPosition(Eigen::Vector3d geoPosition);
        void publishUavAttitude(Eigen::Quaterniond attitude_frd_to_ned);
        void publishUavVelocity(Eigen::Vector3d linVelNed, Eigen::Vector3d angVelFrd);
        void publishIMUMeasurement(Eigen::Vector3d accFrd, Eigen::Vector3d gyroFrd);
        void publishUavMag(Eigen::Vector3d geoPosition, Eigen::Quaterniond attitudeFluToNed);

        const double GPS_POSITION_PERIOD = 0.0005;
        const double ATTITUDE_PERIOD = 0.0005;
        const double VELOCITY_PERIOD = 0.0005;
        const double IMU_PERIOD = 0.0005;
        const double MAG_PERIOD = 0.01;

        double gpsLastPubTimeSec_ = 0;
        double attitudeLastPubTimeSec_ = 0;
        double velocityLastPubTimeSec_ = 0;
        double imuLastPubTimeSec_ = 0;
        double magLastPubTimeSec_ = 0;

        void publishStateToCommunicator();
        //@}

        /// @name Calibration
        //@{
        ros::Subscriber calibrationSub_;
        uint8_t calibrationType_ = 0;
        void calibrationCallback(std_msgs::UInt8 msg);
        //@}

        /// @name Diagnostic
        //@{
        uint64_t actuatorsMsgCounter_;
        uint64_t dynamicsCounter_;
        uint64_t rosPubCounter_;
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
        void publishForcesAndMomentsInfo();
        void publishStaticMotorTransform(const ros::Time & timeStamp,
                                         const char * frame_id,
                                         const char * child_frame_id,
                                         const Eigen::Isometry3d & motorFrame);
        //@}


        /// @name Timer and threads
        //@{
        ros::WallTimer simulationLoopTimer_;
        std::thread proceedDynamicsTask;
        std::thread publishToRosTask;
        std::thread diagnosticTask;

        void simulationLoopTimerCallback(const ros::WallTimerEvent& event);
        void proceedQuadcopterDynamics(double period);
        void publishToRos(double period);
        void performDiagnostic(double period);

        const float ROS_PUB_PERIOD_SEC = 0.05;
        //@}
};

#endif
