/**
 * @file uavDynamicsSimBase.hpp
 * @author ponomarevda96@gmail.com
 * @brief Header file for UAV dynamics
 */

#ifndef UAV_DYNAMICS_SIM_BASE_HPP
#define UAV_DYNAMICS_SIM_BASE_HPP

#include <Eigen/Geometry>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>


class UavDynamicsSimBase{
public:
    UavDynamicsSimBase() {};

    /**
     * @brief Use rosparam here to initialize sim 
     * @return -1 if error occures and simulation can't start
     */
    virtual int8_t init() = 0;
    virtual void initStaticMotorTransform() = 0;

    virtual void setReferencePosition(double latRef, double lonRef, double altRef) = 0;
    virtual void setInitialPosition(const Eigen::Vector3d & position,
                                    const Eigen::Quaterniond& attitude) = 0;
    virtual void land();

    virtual void process(double dt_secs, const std::vector<double> & motorSpeedCommandIn, bool isCmdPercent) = 0;

    virtual Eigen::Vector3d getVehiclePosition() const = 0;
    virtual Eigen::Quaterniond getVehicleAttitude() const = 0;
    virtual Eigen::Vector3d getVehicleVelocity(void) const = 0;
    virtual Eigen::Vector3d getVehicleAngularVelocity(void) const = 0;
    virtual void getIMUMeasurement(Eigen::Vector3d & accOutput, Eigen::Vector3d & gyroOutput) = 0;
    virtual void enu2Geodetic(double east, double north, double up,
                              double *latitude, double *longitude, double *altitude) = 0;
    virtual int8_t calibrate(uint8_t calibrationType) {}


    enum CalibrationType_t{
        WORK_MODE,
        MAG_1_NORMAL=1,             // ROLL OK              ROTATE YAW POSITIVE
        MAG_2_OVERTURNED,           // ROLL INVERTED        ROTATE YAW NEGATIVE
        MAG_3_HEAD_DOWN,            // PITCH POSITIVE pi/2  ROTATE YAW POSITIVE
        MAG_4_HEAD_UP,              // PITCH NEGATIVE pi/2  ROTATE YAW NEGATIVE
        MAG_5_TURNED_LEFT,          // ROLL POSITIVE pi/2   ROTATE YAW POSITIVE
        MAG_6_TURNED_RIGHT,         // ROLL NEGATIVE pi/2   ROTATE YAW NEGATIVE

        ACC_1_NORMAL=11,            // ROLL OK
        ACC_2_OVERTURNED,           // ROLL INVERTED
        ACC_3_HEAD_DOWN,            // PITCH POSITIVE pi/2
        ACC_4_HEAD_UP,              // PITCH NEGATIVE pi/2
        ACC_5_TURNED_LEFT,          // ROLL POSITIVE pi/2
        ACC_6_TURNED_RIGHT,         // ROLL NEGATIVE pi/2
    };

protected:
    /**
     * @brief Publish static transform from UAV centroid to motor
     * @param timeStamp Tf timestamp
     * @param frame_id Parent (UAV) frame ID
     * @param child_frame_id Child (motor) frame ID
     * @param motorFrame Transformation
     */
    void publishStaticMotorTransform(const ros::Time & timeStamp,
                                     const char * frame_id,
                                     const char * child_frame_id,
                                     const Eigen::Isometry3d & motorFrame);
    tf2_ros::StaticTransformBroadcaster* staticMotorTfPub_;
};


#endif  // UAV_DYNAMICS_SIM_BASE_HPP
