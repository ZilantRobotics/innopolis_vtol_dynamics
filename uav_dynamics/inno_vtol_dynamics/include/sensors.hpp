/**
 * @file sensors.hpp
 * @author Dmitry Ponomarev
 */

#ifndef INNO_VTOL_DYNAMICS_SENSORS_HPP
#define INNO_VTOL_DYNAMICS_SENSORS_HPP

#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>

class BaseSensor{
    public:
        BaseSensor() = delete;
        BaseSensor(ros::NodeHandle* nh, double period): node_handler_(nh), PERIOD(period) {};
        void enable() {isEnabled_ = true;}
        void disable() {isEnabled_ = false;}
    protected:
        ros::NodeHandle* node_handler_;
        bool isEnabled_{false};
        const double PERIOD;
        ros::Publisher publisher_;
        double nextPubTimeSec_ = 0;
};

class EscStatusSensor : public BaseSensor{
    public:
        EscStatusSensor(ros::NodeHandle* nh, const char* topic, double period);
        bool publish(const std::vector<double>& rpm);
    private:
        uint8_t nextEscIdx_ = 0;
};

class IceStatusSensor : public BaseSensor{
    public:
        IceStatusSensor(ros::NodeHandle* nh, const char* topic, double period);
        bool publish(double rpm);
};

class FuelTankStatusSensor : public BaseSensor{
    public:
        FuelTankStatusSensor(ros::NodeHandle* nh, const char* topic, double period);
        bool publish(double rpm);
};

class BatteryInfoStatusSensor : public BaseSensor{
    public:
        BatteryInfoStatusSensor(ros::NodeHandle* nh, const char* topic, double period);
        bool publish(double rpm);
};

#endif  // INNO_VTOL_DYNAMICS_SENSORS_HPP
