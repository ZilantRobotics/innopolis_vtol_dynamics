/**
 * @file uavDynamicsSimBase.cpp
 * @author ponomarevda96@gmail.com
 */

#include "uavDynamicsSimBase.hpp"


void UavDynamicsSimBase::publishStaticMotorTransform(const ros::Time & timeStamp,
                                    const char * frame_id,
                                    const char * child_frame_id,
                                    const Eigen::Isometry3d & motorFrame){
    /**
     * @note Sometimes we need to run this module without roscore, for example when running a
     * test. If we declare staticMotorTfPub by value, the test will stack in the constructor.
     * By this reason, we use staticMotorTfPub as pointer to initialize it when we need it.
     */
    // ros::Duration(0.1).sleep();

    if(staticMotorTfPub_ == nullptr){
        staticMotorTfPub_ = new tf2_ros::StaticTransformBroadcaster;
        if(staticMotorTfPub_ == nullptr){
            std::cerr << "Can't create tf2_ros::StaticTransformBroadcaster" << std::endl;
            return;
        }
    }
    // ros::Duration(0.1).sleep();

    std::cout << "kek3" << std::endl;
    geometry_msgs::TransformStamped transformMotor;
    transformMotor.header.stamp = timeStamp;
    transformMotor.header.frame_id = frame_id;
    transformMotor.transform.translation.x = motorFrame.translation()(0);
    transformMotor.transform.translation.y = motorFrame.translation()(1);
    transformMotor.transform.translation.z = motorFrame.translation()(2);
    std::cout << "kek4" << std::endl;

    Eigen::Quaterniond motorAttitude(motorFrame.linear());
    transformMotor.transform.rotation.x = motorAttitude.x();
    transformMotor.transform.rotation.y = motorAttitude.y();
    transformMotor.transform.rotation.z = motorAttitude.z();
    transformMotor.transform.rotation.w = motorAttitude.w();
    transformMotor.child_frame_id = child_frame_id;
    std::cout << "kek5" << std::endl;

    staticMotorTfPub_->sendTransform(transformMotor);
    std::cout << "kek6" << std::endl;

}
