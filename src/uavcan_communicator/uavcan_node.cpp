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

struct Actuators: public UavcanToRosConverter{
    typedef sensor_msgs::Joy OUT_ROS_MSG;
    typedef uavcan::equipment::esc::RawCommand IN_UAVCAN_MSG;
    static constexpr const char* ROS_TOPIC = "/uav/actuators";

    uavcan::Subscriber<IN_UAVCAN_MSG> sub_;
    OUT_ROS_MSG ros_msg_;
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
    }

public:
    Actuators(ros::NodeHandle& ros_node, UavcanNode& uavcan_node): sub_(uavcan_node){
        pub_ = ros_node.advertise<OUT_ROS_MSG>(ROS_TOPIC, 1);
        for(size_t idx = 0; idx < 8; idx++){
            ros_msg_.axes.push_back(0);
        }
        sub_.start(std::bind(&Actuators::callback, this, std::placeholders::_1));
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


    // 1.1. Actuators converter
    Actuators actuators(ros_node, uavcan_node);


    // 1.2. Baro converter
    // boost::function<void(const innopolis_vtol_dynamics::StaticPressure&)> f =
    //     [&](const innopolis_vtol_dynamics::StaticPressure& ros_msg){
    //         std::cout << ros_msg << std::endl;
    //     };
    // auto static_pressure_baro_sub = ros_node.subscribe("/uav/static_pressure", 1, f);

    // auto cb = [&](const innopolis_vtol_dynamics::StaticPressure& msg) {
    //         std::cout << msg << std::endl;
    //     };
    // auto sub = ros_node.subscribe("scan", 1, cb);

    // 1.3. Diff pressure converter
    
    
    // 1.4. Gps converter
    
    
    // 1.5. Imu converter
    uavcan::Publisher<uavcan::equipment::ahrs::RawIMU> imu_pub(uavcan_node);
    const int imu_pub_init_res = imu_pub.init();
    if (imu_pub_init_res < 0){
        throw std::runtime_error("Failed to start the publisher; error: " + std::to_string(imu_pub_init_res));
    }


    // 1.6. Mag converter


    // 2. Spinner
    uavcan_node.setModeOperational();
    uavcan_node.setHealthOk();
    std::cout << "Hello world! " << uavcan_node.getNodeID().get() + 0 << std::endl;
    while (ros::ok()){
        const int res = uavcan_node.spin(uavcan::MonotonicDuration::fromMSec(5));
        ros::spinOnce();
        if (res < 0){
            std::cerr << "Transient failure: " << res << std::endl;
        }


        uavcan::equipment::ahrs::RawIMU imu_msg;  // Always zero initialized
        imu_msg.rate_gyro_latest[0] = std::rand() / float(RAND_MAX);
        int pub_res = imu_pub.broadcast(imu_msg);
        if (pub_res < 0){
            std::cerr << "IMU publication failure: " << pub_res << std::endl;
        }
    }
}