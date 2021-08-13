/**
 * @file inno_vtol_reverse_mixer_node.cpp
 * @author Dmitry Ponomarev
 * @brief Reverse mixer node that maps actuator output group into control group
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

static constexpr char MAPPED_ACTUATOR_TOPIC_NAME[]  = "/uav/actuators";
static constexpr char RAW_ACTUATOR_TOPIC_NAME[]     = "/uav/actuators_raw";


class InnoVtolReverseMixer {
    public:
        explicit InnoVtolReverseMixer(ros::NodeHandle nh);
        int8_t init();

    private:
        ros::NodeHandle node_;

        ros::Publisher mappedActuatorPub_;
        ros::Subscriber rawActuatorsSub_;
        void rawActuatorsCallback(sensor_msgs::Joy msg);
};

InnoVtolReverseMixer::InnoVtolReverseMixer(ros::NodeHandle nh): node_(nh) {}


int8_t InnoVtolReverseMixer::init() {
    rawActuatorsSub_ = node_.subscribe(RAW_ACTUATOR_TOPIC_NAME,
                                          2,
                                          &InnoVtolReverseMixer::rawActuatorsCallback,
                                          this);
    mappedActuatorPub_ = node_.advertise<sensor_msgs::Joy>(MAPPED_ACTUATOR_TOPIC_NAME,
                                                            5);
    return 0;
}

void InnoVtolReverseMixer::rawActuatorsCallback(sensor_msgs::Joy msg) {
    if (msg.axes.size() == 8) {
        if (msg.axes[4] < 0) {
            msg.axes[4] = 0.5;
        }
        msg.axes[5] = (msg.axes[5] >= 0) ? msg.axes[5] * 2 - 1.0 : 0.0;
        msg.axes[6] = (msg.axes[6] >= 0) ? msg.axes[6] * 2 - 1.0 : 0.0;
        msg.axes[7] = msg.axes[7] / 0.75;
        mappedActuatorPub_.publish(msg);
    } else if (msg.axes.size() == 4) {
        msg.axes.push_back(0.5);
        msg.axes.push_back(0.0);
        msg.axes.push_back(0.0);
        msg.axes.push_back(0.0);
        mappedActuatorPub_.publish(msg);
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "inno_vtol_reverse_mixer_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle node_handler;
    InnoVtolReverseMixer reverse_mixer_node(node_handler);
    if (reverse_mixer_node.init() == -1){
        ROS_ERROR("Shutdown.");
        ros::shutdown();
        return 0;
    }

    ros::spin();
    return 0;
}
