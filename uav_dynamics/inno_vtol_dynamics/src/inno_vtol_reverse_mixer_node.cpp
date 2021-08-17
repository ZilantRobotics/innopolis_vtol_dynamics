/**
 * @file inno_vtol_reverse_mixer_node.cpp
 * @author Dmitry Ponomarev
 * @brief Reverse mixer node that maps actuator output group into control group
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

static constexpr char MAPPED_ACTUATOR_TOPIC[]   = "/uav/actuators";
static constexpr char RAW_ACTUATOR_TOPIC[]      = "/uav/actuators_raw";

enum ACTUATORS_OUTPUT {
    MC_MOTOR_0 = 0,
    MC_MOTOR_1,
    MC_MOTOR_2,
    MC_MOTOR_3,
    VTOL_ROLL,
    VTOL_PITCH,
    VTOL_YAW,
    VTOL_THROTTLE,
};

enum BABY_SHARK_OUTPUTS {
    BABY_SHARK_AILERONS = 0,
    BABY_SHARK_A_TAIL_LEFT,
    BABY_SHARK_PUSHER_MOTOR,
    BABY_SHARK_A_TAIL_RIGHT,
    BABY_SHARK_MOTOR_0,
    BABY_SHARK_MOTOR_1,
    BABY_SHARK_MOTOR_2,
    BABY_SHARK_MOTOR_3,
};

enum INNO_VTOL_OUTPUTS {
    INNO_VTOL_MOTOR_0 = 0,
    INNO_VTOL_MOTOR_1,
    INNO_VTOL_MOTOR_2,
    INNO_VTOL_MOTOR_3,
    INNO_VTOL_AILERON,
    INNO_VTOL_ELEVATOR,
    INNO_VTOL_RUDDER,
    INNO_VTOL_THROTLE,
};


class BaseReverseMixer {
    public:
        BaseReverseMixer(ros::NodeHandle nh): node_(nh) {}
        int8_t init();
    protected:
        ros::Publisher mappedActuatorPub_;
        sensor_msgs::Joy mappedActuatorMsg_;
        virtual void rawActuatorsCallback(sensor_msgs::Joy msg) = 0;
    private:
        ros::NodeHandle node_;
        ros::Subscriber rawActuatorsSub_;
};
int8_t BaseReverseMixer::init() {
    rawActuatorsSub_ = node_.subscribe(RAW_ACTUATOR_TOPIC, 2, &BaseReverseMixer::rawActuatorsCallback, this);
    mappedActuatorPub_ = node_.advertise<sensor_msgs::Joy>(MAPPED_ACTUATOR_TOPIC, 5);
    return 0;
}


class BabysharkReverseMixer : public BaseReverseMixer {
    public:
        BabysharkReverseMixer(ros::NodeHandle nh) : BaseReverseMixer(nh) {
            for (size_t channel = 0; channel < 8; channel++) {
                mappedActuatorMsg_.axes.push_back(0);
            }
        }
    protected:
        virtual void rawActuatorsCallback(sensor_msgs::Joy msg) override;
};
void BabysharkReverseMixer::rawActuatorsCallback(sensor_msgs::Joy msg) {
    if (msg.axes.size() == 8) {
        mappedActuatorMsg_.header = msg.header;

        mappedActuatorMsg_.axes[MC_MOTOR_0] = msg.axes[BABY_SHARK_MOTOR_0];
        mappedActuatorMsg_.axes[MC_MOTOR_1] = msg.axes[BABY_SHARK_MOTOR_1];
        mappedActuatorMsg_.axes[MC_MOTOR_2] = msg.axes[BABY_SHARK_MOTOR_2];
        mappedActuatorMsg_.axes[MC_MOTOR_3] = msg.axes[BABY_SHARK_MOTOR_3];

        float roll = msg.axes[BABY_SHARK_AILERONS];
        roll = (roll < 0) ? 0.5 : 1 - roll;
        mappedActuatorMsg_.axes[VTOL_ROLL] = roll;

        float pitch = -msg.axes[BABY_SHARK_A_TAIL_LEFT] + msg.axes[BABY_SHARK_A_TAIL_RIGHT];
        pitch = (pitch < 0) ? 0.0 : pitch / 0.8;
        mappedActuatorMsg_.axes[VTOL_PITCH] = pitch;

        float yaw = msg.axes[BABY_SHARK_A_TAIL_LEFT] + msg.axes[BABY_SHARK_A_TAIL_RIGHT];
        yaw = (yaw < 0) ? 0.0 : (1.0 - yaw) / 0.7;
        mappedActuatorMsg_.axes[VTOL_YAW] = yaw;

        mappedActuatorMsg_.axes[VTOL_THROTTLE] = msg.axes[BABY_SHARK_PUSHER_MOTOR];
        mappedActuatorPub_.publish(mappedActuatorMsg_);
    }
}


class InnoVtolReverseMixer : public BaseReverseMixer {
    public:
        InnoVtolReverseMixer(ros::NodeHandle nh) : BaseReverseMixer(nh) {}
    protected:
        virtual void rawActuatorsCallback(sensor_msgs::Joy msg) override;
};
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

class IrisReverseMixer : public BaseReverseMixer {
    public:
        IrisReverseMixer(ros::NodeHandle nh) : BaseReverseMixer(nh) {
            for (size_t channel = 0; channel < 4; channel++) {
                mappedActuatorMsg_.axes.push_back(0);
            }
        }
    protected:
        virtual void rawActuatorsCallback(sensor_msgs::Joy msg) override;
};
void IrisReverseMixer::rawActuatorsCallback(sensor_msgs::Joy msg) {
    if (msg.axes.size() >= 4) {
        mappedActuatorMsg_.axes[0] = msg.axes[0];
        mappedActuatorMsg_.axes[1] = msg.axes[1];
        mappedActuatorMsg_.axes[2] = msg.axes[2];
        mappedActuatorMsg_.axes[3] = msg.axes[3];

        mappedActuatorMsg_.header = msg.header;
        mappedActuatorPub_.publish(mappedActuatorMsg_);
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "inno_vtol_reverse_mixer_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle node_handler("inno_vtol_reverse_mixer");
    std::string airframe;
    if(!node_handler.getParam("airframe", airframe)){
        ROS_ERROR("ReverseMixer: There is no `airframe` parameter.");
        return -1;
    }

    BaseReverseMixer* reverseMixer;
    if (airframe == "babyshark_standard_vtol") {
        reverseMixer = new BabysharkReverseMixer(node_handler);
    } else if (airframe == "inno_standard_vtol") {
        reverseMixer = new InnoVtolReverseMixer(node_handler);
    } else if (airframe == "iris") {
        reverseMixer = new IrisReverseMixer(node_handler);
    } else {
        ROS_ERROR("ReverseMixer: Wrong `/uav/sim_params/airframe` parameter.");
        return -1;
    }
    ROS_INFO_STREAM("ReverseMixer: airframe is " << airframe.c_str());

    if (reverseMixer->init() == -1){
        ROS_ERROR("Shutdown.");
        ros::shutdown();
        return 0;
    }

    ros::spin();
    return 0;
}
