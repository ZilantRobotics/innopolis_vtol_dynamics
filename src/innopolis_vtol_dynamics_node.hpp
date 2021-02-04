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

        std::string dynamicsTypeName_;

        geodetic_converter::GeodeticConverter geodeticConverter_;
        //@}


        /// @name Communication with PX4
        //@{
        ros::Subscriber actuatorsSub_;
        std::vector<double> actuators_;
        uint64_t lastActuatorsTimestampUsec_;
        uint64_t prevActuatorsTimestampUsec_;
        uint64_t maxDelayUsec_;
        void actuatorsCallback(sensor_msgs::Joy::Ptr msg);

        ros::Subscriber armSub_;
        bool armed_ = false;
        void armCallback(std_msgs::Bool msg);

        ros::Publisher attitudePub_;
        double attitudeLastPubTimeSec_ = 0;
        const double ATTITUDE_PERIOD = 0.005;
        void publishUavAttitude(Eigen::Quaterniond attitude_frd_to_ned);

        ros::Publisher imuPub_;
        double imuLastPubTimeSec_ = 0;
        const double IMU_PERIOD = 0.005;
        void publishIMUMeasurement(Eigen::Vector3d accFrd, Eigen::Vector3d gyroFrd);

        ros::Publisher gpsPositionPub_;
        double gpsLastPubTimeSec_ = 0;
        const double GPS_POSITION_PERIOD = 0.1;
        void publishUavGpsPosition(Eigen::Vector3d geoPosition);

        ros::Publisher speedPub_;
        double velocityLastPubTimeSec_ = 0;
        const double VELOCITY_PERIOD = 0.05;
        void publishUavVelocity(Eigen::Vector3d linVelNed, Eigen::Vector3d angVelFrd);

        ros::Publisher magPub_;
        double magLastPubTimeSec_ = 0;
        const double MAG_PERIOD = 0.03;
        void publishUavMag(Eigen::Vector3d geoPosition, Eigen::Quaterniond attitudeFluToEnu);

        ros::Publisher rawAirDataPub_;
        double rawAirDataLastPubTimeSec_ = 0;
        const double RAW_AIR_DATA_PERIOD = 0.05;
        void publishUavAirData(float absPressure, float diffPressure, float staticTemperature);

        ros::Publisher staticTemperaturePub_;
        double staticTemperatureLastPubTimeSec_ = 0;
        const double STATIC_TEMPERATURE_PERIOD = 0.05;
        void publishUavStaticTemperature(float staticTemperature);

        ros::Publisher staticPressurePub_;
        double staticPressureLastPubTimeSec_ = 0;
        const double STATIC_PRESSURE_PERIOD = 0.05;
        void publishUavStaticPressure(float staticPressure);

        void publishStateToCommunicator();
        //@}

        /// @name Calibration
        //@{
        ros::Subscriber calibrationSub_;
        UavDynamicsSimBase::CalibrationType_t calibrationType_ = UavDynamicsSimBase::WORK_MODE;
        void calibrationCallback(std_msgs::UInt8 msg);
        //@}

        /// @name Diagnostic
        //@{
        uint64_t actuatorsMsgCounter_ = 0;
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
        ros::Publisher controlSurfacesMomentPub_;
        ros::Publisher aoaMomentPub_;
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

        enum DynamicsNotation_t{
            PX4_NED_FRD = 0,
            ROS_ENU_FLU = 1,
        };
        DynamicsNotation_t dynamicsNotation_;
};

#endif
