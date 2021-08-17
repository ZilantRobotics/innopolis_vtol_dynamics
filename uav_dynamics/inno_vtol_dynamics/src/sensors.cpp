/**
 * @file sensors.cpp
 * @author Dmitry Ponomarev
 */

#include "sensors.hpp"

// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/MagneticField.h>
// #include <uavcan_msgs/RawAirData.h>
// #include <uavcan_msgs/StaticPressure.h>
// #include <uavcan_msgs/StaticTemperature.h>
// #include <uavcan_msgs/Fix.h>

#include <uavcan_msgs/EscStatus.h>
#include <uavcan_msgs/IceReciprocatingStatus.h>
#include <uavcan_msgs/IceFuelTankStatus.h>
#include <sensor_msgs/BatteryState.h>


EscStatusSensor::EscStatusSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<uavcan_msgs::EscStatus>(topic, 16);
}
bool EscStatusSensor::publish(const std::vector<double>& rpm) {
    ///< The idea here is to publish each esc status with equal interval instead of burst
    auto crntTimeSec = ros::Time::now().toSec();
    if(isEnabled_ && rpm.size() > 0 && rpm.size() <= 8 && (nextPubTimeSec_ < crntTimeSec)){
        uavcan_msgs::EscStatus escStatusMsg;
        if(nextEscIdx_ >= rpm.size()){
            nextEscIdx_ = 0;
        }
        escStatusMsg.esc_index = nextEscIdx_;
        escStatusMsg.rpm = rpm[nextEscIdx_];
        publisher_.publish(escStatusMsg);
        nextPubTimeSec_ += PERIOD / rpm.size();
        nextEscIdx_++;
    }
}

IceStatusSensor::IceStatusSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<uavcan_msgs::IceReciprocatingStatus>(topic, 16);
}
bool IceStatusSensor::publish(double rpm) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(isEnabled_ && (nextPubTimeSec_ < crntTimeSec)){
        uavcan_msgs::IceReciprocatingStatus iceStatusMsg;
        iceStatusMsg.engine_speed_rpm = rpm;
        publisher_.publish(iceStatusMsg);
        nextPubTimeSec_ += PERIOD;
    }
}

FuelTankStatusSensor::FuelTankStatusSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<uavcan_msgs::IceFuelTankStatus>(topic, 16);
}
bool FuelTankStatusSensor::publish(double fuelLevel) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(isEnabled_ && (nextPubTimeSec_ < crntTimeSec)){
        uavcan_msgs::IceFuelTankStatus fuelTankMsg;
        fuelTankMsg.available_fuel_volume_percent = fuelLevel;
        publisher_.publish(fuelTankMsg);
        nextPubTimeSec_ += PERIOD;
    }
}


BatteryInfoStatusSensor::BatteryInfoStatusSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<sensor_msgs::BatteryState>(topic, 16);
}
bool BatteryInfoStatusSensor::publish(double percentage) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(isEnabled_ && (nextPubTimeSec_ < crntTimeSec)){
        sensor_msgs::BatteryState batteryInfoMsg;
        batteryInfoMsg.voltage = 4.1;
        batteryInfoMsg.percentage = percentage;
        batteryInfoMsg.capacity = 6;
        publisher_.publish(batteryInfoMsg);
        nextPubTimeSec_ += PERIOD;
    }
}
