#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <boost/function.hpp>

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/air_data/StaticTemperature.hpp>
#include <uavcan/equipment/air_data/StaticPressure.hpp>
#include <uavcan/equipment/air_data/RawAirData.hpp>
#include <uavcan/equipment/gnss/Fix.hpp>
#include <uavcan/equipment/ahrs/RawIMU.hpp>
#include <uavcan/equipment/ahrs/MagneticFieldStrength.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <innopolis_vtol_dynamics/StaticPressure.h>
#include <innopolis_vtol_dynamics/StaticTemperature.h>
#include <innopolis_vtol_dynamics/RawAirData.h>


extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();
constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> UavcanNode;
static UavcanNode& getUavcanNode(){
    static UavcanNode uavcan_node(getCanDriver(), getSystemClock());
    return uavcan_node;
}

class UavcanToRosConverter{
protected:
    ros::Publisher pub_;
};

class Actuators: public UavcanToRosConverter{
    typedef sensor_msgs::Joy OUT_ROS_MSG;
    typedef uavcan::equipment::esc::RawCommand IN_UAVCAN_MSG;
    static constexpr const char* ROS_TOPIC = "/uav/actuators";

    uavcan::Subscriber<IN_UAVCAN_MSG> sub_;
    OUT_ROS_MSG ros_msg_;

    // Additional - arming
    ros::Publisher arm_pub_;
    std_msgs::Bool arm_ros_msg_;

    void callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg){
        ros_msg_.header.stamp = ros::Time::now();

        if(uavcan_msg.cmd.size() == 8){
            ros_msg_.axes[0] = uavcan_msg.cmd[0] / 8191.0;
            ros_msg_.axes[1] = uavcan_msg.cmd[1] / 8191.0;
            ros_msg_.axes[2] = uavcan_msg.cmd[2] / 8191.0;
            ros_msg_.axes[3] = uavcan_msg.cmd[3] / 8191.0;

            ros_msg_.axes[4] = 0.5 + (uavcan_msg.cmd[4] - 4096) / 8191.0;
            ros_msg_.axes[5] = (uavcan_msg.cmd[5] == -1) ? 0 : uavcan_msg.cmd[5] / 4096 - 1;
            ros_msg_.axes[6] = (uavcan_msg.cmd[6] == -1) ? 0 : uavcan_msg.cmd[6] / 4096 - 1;

            ros_msg_.axes[7] = uavcan_msg.cmd[7] / 8191.0 / 0.75;
        }else{
            ros_msg_.axes[4] = 0.5; // aileron default position is 0.5
        }

        pub_.publish(ros_msg_);

        // Additional - arming
        arm_ros_msg_.data = false;
        if(uavcan_msg.cmd.size() == 8){
            for(auto raw_cmd : uavcan_msg.cmd){
                if(raw_cmd != -1){
                    arm_ros_msg_.data = true;
                }
            }
        }
        arm_pub_.publish(arm_ros_msg_);
    }

public:
    Actuators(ros::NodeHandle& ros_node, UavcanNode& uavcan_node): sub_(uavcan_node){
        pub_ = ros_node.advertise<OUT_ROS_MSG>(ROS_TOPIC, 1);
        for(size_t idx = 0; idx < 8; idx++){
            ros_msg_.axes.push_back(0);
        }
        sub_.start(std::bind(&Actuators::callback, this, std::placeholders::_1));

        // Additional - arming
        arm_pub_ = ros_node.advertise<std_msgs::Bool>("/uav/arm", 1);
    }
};


class RosToUavcanConverter{
protected:
    ros::Subscriber sub_;
};


class BaroStaticPressure: public RosToUavcanConverter{
    typedef uavcan::equipment::air_data::StaticPressure OUT_UAVCAN_MSG;
    typedef innopolis_vtol_dynamics::StaticPressure IN_ROS_MSG;
    static constexpr const char* ROS_TOPIC = "/uav/static_pressure";

    uavcan::Publisher<OUT_UAVCAN_MSG> pub_;
    OUT_UAVCAN_MSG out_uavcan_msg_;

    void callback(IN_ROS_MSG::Ptr in_ros_msg){
        auto pressure = in_ros_msg->static_pressure * 100 + std::rand() / float(RAND_MAX);
        out_uavcan_msg_.static_pressure = pressure;
        out_uavcan_msg_.static_pressure_variance = 1;
        int pub_res = pub_.broadcast(out_uavcan_msg_);
        if (pub_res < 0){
            std::cerr << "BaroStaticPressure publication failure: " << pub_res << std::endl;
        }
    }
public:
    BaroStaticPressure(ros::NodeHandle& ros_node, UavcanNode& uavcan_node):
        pub_(uavcan_node){
        sub_ = ros_node.subscribe(ROS_TOPIC, 1, &BaroStaticPressure::callback, this);
    }
};


class BaroStaticTemperature: public RosToUavcanConverter{
    typedef uavcan::equipment::air_data::StaticTemperature OUT_UAVCAN_MSG;
    typedef innopolis_vtol_dynamics::StaticTemperature IN_ROS_MSG;
    static constexpr const char* ROS_TOPIC = "/uav/static_temperature";

    uavcan::Publisher<OUT_UAVCAN_MSG> pub_;
    OUT_UAVCAN_MSG out_uavcan_msg_;

    void callback(IN_ROS_MSG::Ptr in_ros_msg){
        auto temperature = in_ros_msg->static_temperature * 100 + std::rand() / float(RAND_MAX);
        out_uavcan_msg_.static_temperature = temperature;
        out_uavcan_msg_.static_temperature_variance = 1;
        int pub_res = pub_.broadcast(out_uavcan_msg_);
        if (pub_res < 0){
            std::cerr << "BaroStaticTemperature publication failure: " << pub_res << std::endl;
        }
    }
public:
    BaroStaticTemperature(ros::NodeHandle& ros_node, UavcanNode& uavcan_node):
        pub_(uavcan_node){
        sub_ = ros_node.subscribe(ROS_TOPIC, 1, &BaroStaticTemperature::callback, this);
    }
};


class DiffPressure: public RosToUavcanConverter{
    typedef uavcan::equipment::air_data::RawAirData OUT_UAVCAN_MSG;
    typedef innopolis_vtol_dynamics::RawAirData IN_ROS_MSG;
    static constexpr const char* ROS_TOPIC = "/uav/raw_air_data";

    uavcan::Publisher<OUT_UAVCAN_MSG> pub_;
    OUT_UAVCAN_MSG out_uavcan_msg_;

    void callback(IN_ROS_MSG::Ptr in_ros_msg){
        double static_pressure = in_ros_msg->static_pressure * 100;
        double static_pressure_noise = std::rand() / float(RAND_MAX);

        double differential_pressure = in_ros_msg->differential_pressure * 100;
        double differential_pressure_noise = std::rand() / float(RAND_MAX);

        double temperature = 24; //self.temperature.static_temperature;
        double temperature_noise = std::rand() / float(RAND_MAX);

        out_uavcan_msg_.static_air_temperature = temperature + temperature_noise;
        out_uavcan_msg_.static_pressure = static_pressure + static_pressure_noise;
        out_uavcan_msg_.differential_pressure = differential_pressure + differential_pressure_noise;

        int pub_res = pub_.broadcast(out_uavcan_msg_);
        if (pub_res < 0){
            std::cerr << "DiffPressure publication failure: " << pub_res << std::endl;
        }
    }
public:
    DiffPressure(ros::NodeHandle& ros_node, UavcanNode& uavcan_node):
        pub_(uavcan_node){
        sub_ = ros_node.subscribe(ROS_TOPIC, 1, &DiffPressure::callback, this);
    }
};


class GPS: public RosToUavcanConverter{
    typedef uavcan::equipment::gnss::Fix OUT_UAVCAN_MSG;
    typedef sensor_msgs::NavSatFix IN_ROS_MSG;
    static constexpr const char* ROS_TOPIC = "/uav/gps_position";

    uavcan::Publisher<OUT_UAVCAN_MSG> pub_;
    OUT_UAVCAN_MSG out_uavcan_msg_;

    void callback(IN_ROS_MSG::Ptr in_ros_msg){
        uint64_t latitude = in_ros_msg->latitude * 100000000;
        uint64_t longitude = in_ros_msg->longitude * 100000000;
        uint64_t altitude = in_ros_msg->altitude * 1000;
        std::vector<float> ned_velocity(3);
        // ned_velocity[0] = ;
        // ned_velocity[1] = ;
        // ned_velocity[2] = ;
        // uint64_t ned_velocity = [self.velocity.linear.x,
        //                 self.velocity.linear.y,
        //                 self.velocity.linear.z]

        out_uavcan_msg_.latitude_deg_1e8 = latitude;
        out_uavcan_msg_.longitude_deg_1e8 = longitude;
        out_uavcan_msg_.height_msl_mm = altitude;
        out_uavcan_msg_.ned_velocity[0] = ned_velocity[0];
        out_uavcan_msg_.ned_velocity[1] = ned_velocity[1];
        out_uavcan_msg_.ned_velocity[2] = ned_velocity[2];
        out_uavcan_msg_.sats_used = 10;
        out_uavcan_msg_.status = 3;
        out_uavcan_msg_.pdop = 99;
        int pub_res = pub_.broadcast(out_uavcan_msg_);
        if (pub_res < 0){
            std::cerr << "GPS publication failure: " << pub_res << std::endl;
        }
    }
public:
    GPS(ros::NodeHandle& ros_node, UavcanNode& uavcan_node):
        pub_(uavcan_node){
        sub_ = ros_node.subscribe(ROS_TOPIC, 1, &GPS::callback, this);
    }
};


class IMU: public RosToUavcanConverter{
    typedef uavcan::equipment::ahrs::RawIMU OUT_UAVCAN_MSG;
    typedef sensor_msgs::Imu IN_ROS_MSG;
    static constexpr const char* ROS_TOPIC = "/uav/imu";

    uavcan::Publisher<OUT_UAVCAN_MSG> pub_;
    OUT_UAVCAN_MSG out_uavcan_msg_;

    void callback(IN_ROS_MSG::Ptr in_ros_msg){
        out_uavcan_msg_.rate_gyro_latest[0] = in_ros_msg->angular_velocity.x;
        out_uavcan_msg_.rate_gyro_latest[1] = in_ros_msg->angular_velocity.y;
        out_uavcan_msg_.rate_gyro_latest[2] = in_ros_msg->angular_velocity.z;

        out_uavcan_msg_.accelerometer_latest[0] = in_ros_msg->linear_acceleration.x;
        out_uavcan_msg_.accelerometer_latest[1] = in_ros_msg->linear_acceleration.y;
        out_uavcan_msg_.accelerometer_latest[2] = in_ros_msg->linear_acceleration.z;

        int pub_res = pub_.broadcast(out_uavcan_msg_);
        if (pub_res < 0){
            std::cerr << "IMU publication failure: " << pub_res << std::endl;
        }
    }
public:
    IMU(ros::NodeHandle& ros_node, UavcanNode& uavcan_node):
        pub_(uavcan_node){
        sub_ = ros_node.subscribe(ROS_TOPIC, 1, &IMU::callback, this);
    }
};


class Magnetometer: public RosToUavcanConverter{
    typedef uavcan::equipment::ahrs::MagneticFieldStrength OUT_UAVCAN_MSG;
    typedef sensor_msgs::MagneticField IN_ROS_MSG;
    static constexpr const char* ROS_TOPIC = "/uav/mag";

    uavcan::Publisher<OUT_UAVCAN_MSG> pub_;
    OUT_UAVCAN_MSG out_uavcan_msg_;

    void callback(IN_ROS_MSG::Ptr in_ros_msg){
        out_uavcan_msg_.magnetic_field_ga[0] = in_ros_msg->magnetic_field.x;
        out_uavcan_msg_.magnetic_field_ga[1] = in_ros_msg->magnetic_field.y;
        out_uavcan_msg_.magnetic_field_ga[2] = in_ros_msg->magnetic_field.z;

        int pub_res = pub_.broadcast(out_uavcan_msg_);
        if (pub_res < 0){
            std::cerr << "IMU publication failure: " << pub_res << std::endl;
        }
    }
public:
    Magnetometer(ros::NodeHandle& ros_node, UavcanNode& uavcan_node):
        pub_(uavcan_node){
        sub_ = ros_node.subscribe(ROS_TOPIC, 1, &Magnetometer::callback, this);
    }
};


int main(int argc, char** argv){
    // 1.1. Init ros node
    ros::init(argc, argv, "uavacn_px4_communicator");
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::NodeHandle ros_node;


    // 1.2. Init uavcan node
    auto& uavcan_node = getUavcanNode();
    const int self_node_id = 42;
    uavcan_node.setNodeID(self_node_id);
    uavcan_node.setName("inno.dynamics");
    uavcan::protocol::SoftwareVersion sw_version;
    sw_version.major = 1;
    uavcan_node.setSoftwareVersion(sw_version);
    uavcan::protocol::HardwareVersion hw_version;
    hw_version.major = 1;
    uavcan_node.setHardwareVersion(hw_version);
    const int node_start_res = uavcan_node.start();
    if (node_start_res < 0){
        throw std::runtime_error("Failed to start the uavcan_node; error: " + std::to_string(node_start_res));
    }


    // 1.0. Node status
    // uavcan::Subscriber<uavcan::protocol::NodeStatus> node_status_sub(uavcan_node);
    // const int node_status_start_res = node_status_sub.start(
    //     [&](const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg)
    //     {
    //         std::cout << msg << std::endl;
    //     });

    Actuators actuators(ros_node, uavcan_node);
    BaroStaticPressure baroStaticPressure(ros_node, uavcan_node);
    BaroStaticTemperature baroStaticTemperature(ros_node, uavcan_node);
    DiffPressure diffPressure(ros_node, uavcan_node);
    GPS gps(ros_node, uavcan_node);
    IMU imu(ros_node, uavcan_node);
    Magnetometer mag(ros_node, uavcan_node);

    // 2. Spinner
    uavcan_node.setModeOperational();
    uavcan_node.setHealthOk();
    std::cout << "Hello world! " << uavcan_node.getNodeID().get() + 0 << std::endl;
    while (ros::ok()){
        const int res = uavcan_node.spin(uavcan::MonotonicDuration::fromMSec(2));
        ros::spinOnce();
        if (res < 0){
            std::cerr << "Transient failure: " << res << std::endl;
        }
    }
}