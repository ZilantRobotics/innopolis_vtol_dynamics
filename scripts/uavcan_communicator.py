#!/usr/bin/env python3
"""
This module allows to establish connection between PX4 and simulator via uavcan and ROS.
"""

# Common
import logging
import sys
import numpy as np
import queue

# ROS
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Bool
from drone_communicators.msg import RawAirData
from drone_communicators.msg import StaticTemperature
from drone_communicators.msg import StaticPressure
from drone_communicators.msg import Fix


# For uavcan v0.1
import uavcan
import can


DEV_PATH = "/dev/ttyACM1"
CAN_DEVICE_TYPE = "serial"


class UavcanCommunicatorV0:
    """
    Simple wrap on pyuavcan v0.1
    Based on example: https://legacy.uavcan.org/Implementations/Pyuavcan/Tutorials/
    """
    def __init__(self, can_device_type, node_id=42, node_name="uavcan communicator"):
        self.handlers = []
        self.node = None

        self.tx_can_error_counter = 0
        self.tx_full_buffer_error = 0
        self.spin_can_error_counter = 0
        self.spin_transfer_error_counter = 0

        if can_device_type == "serial":
            kawrgs = {"can_device_name" : DEV_PATH,
                      "baudrate" : 1000000}
        elif can_device_type == "can-slcan":
            kawrgs = {"can_device_name" : "slcan0",
                      "bustype" : "socketcan",
                      "bitrate" : 1000000}
        else:
            rospy.logerr("UavcanCommunicatorV0: Wrong can device type")
            sys.exit()

        node_info = uavcan.protocol.GetNodeInfo.Response()
        node_info.name = node_name
        node_info.software_version.major = 0
        node_info.software_version.minor = 2
        node_info.hardware_version.unique_id = b'12345'

        self.node = uavcan.make_node(node_id=node_id, node_info=node_info, **kawrgs)

    def __del__(self):
        if self.node is not None:
            self.node.close()


    def subscribe(self, data_type, callback):
        self.handlers.append(self.node.add_handler(data_type, callback))

    def publish(self, data_type, priority=uavcan.TRANSFER_PRIORITY_LOWEST):
        try:
            self.node.broadcast(data_type, priority=priority)
        except uavcan.driver.common.TxQueueFullError as e:
            self.tx_full_buffer_error += 1
            rospy.logerr("UavcanCommunicatorV0 TxQueueFullError {}, №{}".format(
                e, self.tx_full_buffer_error))
        except queue.Full as e:
            rospy.logerr("UavcanCommunicatorV0 queue.Full {}".format(e))
        except uavcan.driver.common.DriverError as e:
            rospy.logerr("UavcanCommunicatorV0 DriverError {}".format(e))

    def spin(self, period=0.00001):
        try:
            if(period == -1):
                self.node.spin()
            else:
                self.node.spin(period)
        except uavcan.transport.TransferError as e:
            self.spin_transfer_error_counter += 1
            rospy.logerr("spin uavcan.transport.TransferError {}, №{}".format(
                        e, self.spin_transfer_error_counter))
        # AttributeError: module 'can' has no attribute 'CanError'
        # except can.CanError as e:
        #     self.spin_can_error_counter += 1
        #     rospy.logerr("spin can.CanError {}, №{}".format(
        #                 e, self.spin_can_error_counter))
        except queue.Full as e:
            rospy.logerr("spin queue.Full {}".format(e))
        except uavcan.driver.common.TxQueueFullError as e:
            rospy.logerr("spin uavcan.driver.common.TxQueueFullError {}".format(e))


class Converter:
    def __init__(self, direction, communicator, ros_topic, ros_data_type, uavcan_data_type):
        self.publisher = None

        if direction == "ros->uavcan" and ros_topic is not None and ros_data_type is not None:
            rospy.Subscriber(ros_topic, ros_data_type, self._convert)
            self.publisher = communicator
            rospy.loginfo(str(ros_topic) + " -> uavcan")
        elif direction == "uavcan->ros" and uavcan_data_type is not None:
            communicator.subscribe(uavcan_data_type, self._convert)
            if ros_topic is not None and ros_data_type is not None:
                self.publisher = rospy.Publisher(ros_topic, ros_data_type, queue_size=10)
            rospy.loginfo("uavcan ->" + str(ros_topic))
        else:
            rospy.logerr("Wrong converter inputs!!!")

    def _convert(self, event):
        print('default', event)

class NodeStatus(Converter):
    def __init__(self, communicator):
        super().__init__(direction="uavcan->ros",
                         communicator=communicator,
                         ros_topic=None,
                         ros_data_type=None,
                         uavcan_data_type=uavcan.protocol.NodeStatus)

    def _convert(self, event):
        # print('NodeStatus', event)
        pass

class Actuators(Converter):
    def __init__(self, communicator):
        super().__init__(direction="uavcan->ros",
                         communicator=communicator,
                         ros_topic="/uav/actuators",
                         ros_data_type=Joy,
                         uavcan_data_type=uavcan.equipment.esc.RawCommand)
        self.msg = Joy()
        self.msg.header.frame_id = "todo: come up with a topic name"
        self.msg.axes = [0, 0, 0, 0, 0.5, 0, 0, -1]

        self.arm_publisher = rospy.Publisher("/uav/arm", Bool, queue_size=10)
        self.arm_msg = Bool()

    def _convert(self, event):
        # print("actuators callback", event.transfer.ts_monotonic, event.transfer.ts_real, rospy.get_time())
        raw_cmd_msg = list(event.transfer.payload.cmd)
        self.msg.header.stamp = rospy.get_rostime()
        raw_cmd_len = len(raw_cmd_msg)
        if raw_cmd_len == 0:
            self.msg.axes[0] = 0
            self.msg.axes[1] = 0
            self.msg.axes[2] = 0
            self.msg.axes[3] = 0

            self.msg.axes[4] = 0.5
            self.msg.axes[5] = 0
            self.msg.axes[6] = 0

            self.msg.axes[7] = -1
        elif raw_cmd_len == 8:
            if raw_cmd_msg[5] == -1:
                raw_cmd_msg[5] = 4096
            if raw_cmd_msg[6] == -1:
                raw_cmd_msg[6] = 4096
            self.msg.axes[0] = raw_cmd_msg[0] / 8191.0
            self.msg.axes[1] = raw_cmd_msg[1] / 8191.0
            self.msg.axes[2] = raw_cmd_msg[2] / 8191.0
            self.msg.axes[3] = raw_cmd_msg[3] / 8191.0

            self.msg.axes[4] = 0.5 + (raw_cmd_msg[4] - 4096) / 8191.0
            self.msg.axes[5] = raw_cmd_msg[5] / 4096 - 1
            self.msg.axes[6] = raw_cmd_msg[6] / 4096 - 1

            self.msg.axes[7] = raw_cmd_msg[7] / 8191.0 / 0.75
        self.publisher.publish(self.msg)

        self.arm_msg.data = False
        if len(raw_cmd_msg) == 8:
            for raw_cmd in raw_cmd_msg:
                if raw_cmd != -1:
                    self.arm_msg.data = True
        self.arm_publisher.publish(self.arm_msg)


class IMU(Converter):
    def __init__(self, communicator):
        super().__init__(direction="ros->uavcan",
                         communicator=communicator,
                         ros_topic="/uav/imu",
                         ros_data_type=Imu,
                         uavcan_data_type=uavcan.equipment.ahrs.RawIMU)

    def _convert(self, in_ros_msg):
        lin_accel = in_ros_msg.linear_acceleration
        ang_vel = in_ros_msg.angular_velocity

        gyro = [ang_vel.x, ang_vel.y, ang_vel.z]
        accel = [lin_accel.x, lin_accel.y, lin_accel.z]

        out_uavcan_msg = uavcan.equipment.ahrs.RawIMU(
            rate_gyro_latest=gyro,
            accelerometer_latest=accel)

        self.publisher.publish(out_uavcan_msg)

class GPS(Converter):
    def __init__(self, communicator):
        super().__init__(direction="ros->uavcan",
                         communicator=communicator,
                         ros_topic="/uav/gps_position",
                         ros_data_type=Fix,
                         uavcan_data_type=uavcan.equipment.gnss.Fix)
        rospy.Subscriber("/uav/velocity", Twist, self._save_velocity_msg)
        self.velocity = Twist()

    def _convert(self, in_ros_msg):
        geodetioc_pose = [int(in_ros_msg.latitude_deg_1e8),
                          int(in_ros_msg.longitude_deg_1e8),
                          int(in_ros_msg.height_msl_mm)]
        ned_velocity = [self.velocity.linear.x,
                        self.velocity.linear.y,
                        self.velocity.linear.z]
        out_uavcan_msg = uavcan.equipment.gnss.Fix(latitude_deg_1e8=geodetioc_pose[0],
                                                   longitude_deg_1e8=geodetioc_pose[1],
                                                   height_msl_mm=geodetioc_pose[2],
                                                   ned_velocity=ned_velocity,
                                                   sats_used=10,
                                                   status=3,
                                                   pdop=99)
        self.publisher.publish(out_uavcan_msg)

    def _save_velocity_msg(self, in_ros_msg):
        self.velocity = in_ros_msg

class BaroPressure(Converter):
    def __init__(self, communicator):
        super().__init__(direction="ros->uavcan",
                         communicator=communicator,
                         ros_topic="/uav/static_pressure",
                         ros_data_type=StaticPressure,
                         uavcan_data_type=uavcan.equipment.air_data.StaticPressure)

    def _convert(self, in_ros_msg):
        pressure = in_ros_msg.static_pressure * 100 # 98600
        pressure_noise = float(np.random.normal(0, 0.1, 1))
        out_uavcan_msg = uavcan.equipment.air_data.StaticPressure(
            static_pressure=pressure+pressure_noise,
            static_pressure_variance=1)
        self.publisher.publish(out_uavcan_msg)

class BaroTemperature(Converter):
    def __init__(self, communicator):
        super().__init__(direction="ros->uavcan",
                         communicator=communicator,
                         ros_topic="/uav/static_temperature",
                         ros_data_type=StaticTemperature,
                         uavcan_data_type=uavcan.equipment.air_data.StaticTemperature)

    def _convert(self, in_ros_msg):
        temperature = in_ros_msg.static_temperature * 100 # 98600
        temperature_noise = float(np.random.normal(0, 0.2, 1))
        out_uavcan_msg = uavcan.equipment.air_data.StaticTemperature(
            static_temperature=temperature+temperature_noise,
            static_temperature_variance=1)
        self.publisher.publish(out_uavcan_msg)

class Magnetometer(Converter):
    def __init__(self, communicator):
        super().__init__(direction="ros->uavcan",
                         communicator=communicator,
                         ros_topic="/uav/mag",
                         ros_data_type=MagneticField,
                         uavcan_data_type=uavcan.equipment.ahrs.MagneticFieldStrength)

    def _convert(self, in_ros_msg):
        mag = in_ros_msg.magnetic_field
        magnetic_field_ga = [mag.x, mag.y, mag.z]
        magnetic_field_ga_noise = list(np.random.normal(0, 0.001, 3))
        out_uavcan_msg = uavcan.equipment.ahrs.MagneticFieldStrength(
            magnetic_field_ga=[sum(i) for i in zip(magnetic_field_ga, magnetic_field_ga_noise)])
        self.publisher.publish(out_uavcan_msg)

class DiffPressure(Converter):
    def __init__(self, communicator):
        super().__init__(direction="ros->uavcan",
                         communicator=communicator,
                         ros_topic="/uav/raw_air_data",
                         ros_data_type=RawAirData,
                         uavcan_data_type=uavcan.equipment.air_data.RawAirData)
        rospy.Subscriber("/uav/static_temperature", StaticTemperature, self._save_temperature_msg)
        self.temperature = StaticTemperature()

    def _convert(self, in_ros_msg):
        static_pressure = in_ros_msg.static_pressure * 100 # 98600
        static_pressure_noise = float(np.random.normal(0, 0.1, 1))

        differential_pressure = in_ros_msg.differential_pressure * 100 # 0
        differential_pressure_noise = float(np.random.normal(0, 0.01, 1))

        temperature = self.temperature.static_temperature
        temperature_noise = float(np.random.normal(0, 0.01, 1))

        out_uavcan_msg = uavcan.equipment.air_data.RawAirData(
            static_air_temperature=temperature+temperature_noise,
            static_pressure=static_pressure+static_pressure_noise,
            differential_pressure=differential_pressure+differential_pressure_noise)

        self.publisher.publish(out_uavcan_msg)

    def _save_temperature_msg(self, msg):
        self.temperature = msg

if __name__=="__main__":
    uavcan_communicator = UavcanCommunicatorV0(can_device_type='serial')
    rospy.init_node("px4_uavcan_communicator", log_level=rospy.DEBUG)
    rospy.sleep(1)

    converters = []
    converters.append(NodeStatus(uavcan_communicator))
    converters.append(Actuators(uavcan_communicator))
    converters.append(IMU(uavcan_communicator))
    converters.append(GPS(uavcan_communicator))
    converters.append(BaroPressure(uavcan_communicator))
    converters.append(BaroTemperature(uavcan_communicator))
    converters.append(Magnetometer(uavcan_communicator))
    converters.append(DiffPressure(uavcan_communicator))

    while not rospy.is_shutdown():
        uavcan_communicator.spin(0.001)
