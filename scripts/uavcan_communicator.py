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
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Bool

# For uavcan v0.1
import uavcan
import can

ACTUATORS_PUB_TOPIC = "/uav/actuators"
ARM_PUB_TOPIC = "/uav/arm"

GPS_POSITION_SUB_TOPIC = "/uav/gps_position"
ATTITUDE_SUB_TOPIC = "/uav/attitude"
VELOCITY_SUB_TOPIC = "/uav/velocity"
IMU_SUB_TOPIC = "/uav/imu"
MAG_SUB_TOPIC = "/uav/mag"

JOY_TOPIC = "Joy"
NODE_STATUS_TOPIC = "NodeStatus"

ARM_PERIOD = 0.2
ACTUATORS_PERIOD = 0.025
IMU_PERIOD = 0.005
GPS_PERIOD = 0.2
BARO_PERIOD = 0.05
MAG_PERIOD = 0.05
DIFF_PRESSURE_PERIOD = 0.05

class UavcanCommunicatorV0:
    """
    Simple wrap on pyuavcan v0.1
    Based on example: https://legacy.uavcan.org/Implementations/Pyuavcan/Tutorials/
    """
    def __init__(self, can_device_type, node_id=42, node_name="uavcan communicator"):
        """
        Simply create a node without starting it.
        param can_device_type - could be 'serial' or 'can-slcan'
        """
        self.handlers = []
        self.node = None

        self.tx_can_error_counter = 0
        self.tx_full_buffer_error = 0
        self.spin_can_error_counter = 0
        self.spin_transfer_error_counter = 0

        if can_device_type == "serial":
            kawrgs = {"can_device_name" : "/dev/ttyACM2",
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
        """
        param data_type - https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/
        param callback - any function with single parameter - event

        Example:
        data_type = uavcan.protocol.NodeStatus
        callback = lambda event: print(uavcan.to_yaml(event))
        communicator.subscribe(data_type, callback)
        """
        self.handlers.append(self.node.add_handler(data_type, callback))

    def publish(self, data_type, priority=uavcan.TRANSFER_PRIORITY_LOWEST):
        """
        param data_type - https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/

        Example:
        fix2 = uavcan.equipment.gnss.Fix2(pdop=10)
        communicator.publish(fix2)
        """
        try:
            self.node.broadcast(data_type, priority=priority)
        except can.CanError as e:
            self.tx_can_error_counter += 1
            rospy.logerr("tx can.CanError {}, №{}".format(
                         e, self.tx_can_error_counter))
        except uavcan.driver.common.TxQueueFullError as e:
            self.tx_full_buffer_error += 1
            rospy.logerr("tx uavcan.driver.common.TxQueueFullError {}, №{}".format(
                         e, self.tx_full_buffer_error))
        except queue.Full as e:
            rospy.logerr("tx queue.Full {}".format(e))

    def create_spin_thread(self):
        """
        This approach works fine for experimenting, however, it must never be used in production
        because the library is not thread safe.
        """
        while not rospy.is_shutdown():
            self.spin(period=-1)

    def spin(self, period=0.00001):
        """
        period - blocking time, where -1 means infinity, 0 means non-blocking
        """
        try:
            if(period == -1):
                self.node.spin()
            else:
                self.node.spin(period)
        except uavcan.transport.TransferError as e:
            self.spin_transfer_error_counter += 1
            rospy.logerr("spin uavcan.transport.TransferError {}, №{}".format(
                        e, self.spin_transfer_error_counter))
        except can.CanError as e:
            self.spin_can_error_counter += 1
            rospy.logerr("spin can.CanError {}, №{}".format(
                        e, self.spin_can_error_counter))
        except queue.Full as e:
            rospy.logerr("spin queue.Full {}".format(e))
        except uavcan.driver.common.TxQueueFullError as e:
            rospy.logerr("spin uavcan.driver.common.TxQueueFullError {}".format(e))


class SubscriberManager:
    """It allows to subscribe on UAVCAN or ROS and simply store data to dict by callback"""
    def __init__(self, communicator):
        self.communicator = communicator
        self.in_msgs = dict()
        self.msg_timestamps = dict()

    def subscribe(self, protocol_type, in_data_type, in_topic):
        callback = lambda event : self._callback_save_msg(event, in_topic)
        if protocol_type == "ros":
            rospy.Subscriber(in_topic, in_data_type, callback)
        elif protocol_type == "uavcan":
            self.communicator.subscribe(in_data_type, callback)
        self.in_msgs[in_topic] = None
        self.msg_timestamps[in_topic] = rospy.get_time()

    def _callback_save_msg(self, event, in_topic):
        self.in_msgs[in_topic] = event
        self.msg_timestamps[in_topic] = rospy.get_time()


class Publisher:
    """
    Simple wrap on publisher that has 2 key features:
    - it has access to all subscription data throught SubscriberManager field - self.in_msgs
    - it has method 'publish' that allows unifiedly perform publication of any data type"""
    def __init__(self, subscriber_manager, sub_topic, pub_topic):
        self.in_msgs = subscriber_manager.in_msgs
        self.timestamp = rospy.get_time()
        self.msg_timestamps = subscriber_manager.msg_timestamps
        self.number_of_msgs = 0
        self.sub_topic = sub_topic
        self.pub_topic = pub_topic
        self.period = 1
        self.publisher = None
        self.TIMEOUT_PERIOD = 2

    def publish(self):
        crnt_time = rospy.get_time()
        last_mgs_timestamp = self.msg_timestamps[self.sub_topic]

        if last_mgs_timestamp > crnt_time - self.period and self.timestamp < crnt_time - self.period:
            msg = self.fill_msg()
            if msg is not None and self.publisher is not None:
                self.publisher.publish(msg)
                self.number_of_msgs += 1
                self.timestamp = crnt_time
        elif self.timestamp + self.TIMEOUT_PERIOD < crnt_time:
            rospy.logwarn_throttle(2, "{} was publishhed long time ago: {}".format(
                                   self.pub_topic, self.timestamp))

    def fill_msg(self):
        return None


class IMU(Publisher):
    def __init__(self, subscriber_manager):
        super().__init__(subscriber_manager, IMU_SUB_TOPIC, IMU_SUB_TOPIC)
        self.period = IMU_PERIOD
        self.publisher = subscriber_manager.communicator

    def fill_msg(self):
        if self.in_msgs[IMU_SUB_TOPIC] is not None:
            lin_accel = self.in_msgs[IMU_SUB_TOPIC].linear_acceleration
            ang_vel = self.in_msgs[IMU_SUB_TOPIC].angular_velocity

            gyro = [ang_vel.x, ang_vel.y, ang_vel.z]
            accel = [lin_accel.x, lin_accel.y, lin_accel.z]

            msg = uavcan.equipment.ahrs.RawIMU(
                rate_gyro_latest=gyro,
                accelerometer_latest=accel)
        else:
            msg = None
        return msg


class GPS(Publisher):
    def __init__(self, subscriber_manager):
        super().__init__(subscriber_manager, GPS_POSITION_SUB_TOPIC, GPS_POSITION_SUB_TOPIC)
        self.period = GPS_PERIOD
        self.publisher = subscriber_manager.communicator

    def fill_msg(self):
        if self.in_msgs[GPS_POSITION_SUB_TOPIC] is None:
            msg = None
        else:
            geodetioc_pose = [int(self.in_msgs[GPS_POSITION_SUB_TOPIC].latitude * 100000000),
                              int(self.in_msgs[GPS_POSITION_SUB_TOPIC].longitude * 100000000),
                              int(self.in_msgs[GPS_POSITION_SUB_TOPIC].altitude * 1000)]
            msg = uavcan.equipment.gnss.Fix(latitude_deg_1e8=geodetioc_pose[0],
                                            longitude_deg_1e8=geodetioc_pose[1],
                                            height_msl_mm=geodetioc_pose[2],
                                            sats_used=10,
                                            status=3,
                                            pdop=99)
        return msg


class Actuators(Publisher):
    def __init__(self, subscriber_manager):
        super().__init__(subscriber_manager, JOY_TOPIC, ACTUATORS_PUB_TOPIC)
        self.period = ACTUATORS_PERIOD
        self.publisher = rospy.Publisher(self.pub_topic, Joy, queue_size=10)
        self.msg = Joy()
        self.msg.header.frame_id = "todo: come up with a topic name"
        self.msg.axes = [0, 0, 0, 0, 0.5, 0, 0, -1]

    def fill_msg(self):
        """
        input[0-7] are normalized into [-8192, 8191]; negative values indicate reverse rotation
        output[0-3] are from -1.0 to +1.0
        output[4] is from 0.0 to 1.0 with middle value 0.5
        output[5-6] are from -1.0 to +1.0 with middle value 0
        output[7] from -1.0 to 1.0 with default value 0
        """
        if self.in_msgs[self.sub_topic] is not None:
            raw_cmd_msg = list(self.in_msgs[self.sub_topic].transfer.payload.cmd)
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

                self.msg.axes[7] = raw_cmd_msg[7] / 8191.0
            else:
                rospy.logerr_throttle(1,
                    "Wrong Actuators (RawCmd) size={}. May be wrong mixer?".format(raw_cmd_len))
        else:
            return None
        return self.msg


class Arm(Publisher):
    def __init__(self, subscriber_manager):
        super().__init__(subscriber_manager, JOY_TOPIC, ARM_PUB_TOPIC)
        self.period = ARM_PERIOD
        self.publisher = rospy.Publisher(ARM_PUB_TOPIC, Bool, queue_size=10)
        self.msg = Bool()

    def publish(self):
        crnt_time = rospy.get_time()
        if self.timestamp + self.period < crnt_time:
            msg = self.fill_msg()
            self.publisher.publish(msg)
            self.number_of_msgs += 1
            self.timestamp = crnt_time

    def fill_msg(self):
        self.msg.data = False
        if self.in_msgs[self.sub_topic] is not None:
            raw_cmd_msg = list(self.in_msgs[self.sub_topic].transfer.payload.cmd)
            if len(raw_cmd_msg) == 8:
                for raw_cmd in raw_cmd_msg:
                    if raw_cmd != -1:
                        self.msg.data = True
        return self.msg

class Baro(Publisher):
    def __init__(self, subscriber_manager):
        super().__init__(subscriber_manager, None, None)
        self.period = BARO_PERIOD
        self.publisher = subscriber_manager.communicator

    def publish(self):
        crnt_time = rospy.get_time()
        if self.timestamp + self.period < crnt_time:
            msgs = self.fill_msg()
            for msg in msgs:
                self.publisher.publish(msg)
                self.number_of_msgs += 1
                self.timestamp = crnt_time

    def fill_msg(self):
        temperature = 27.42+273
        temperature_noise = float(np.random.normal(0, 0.1, 1))

        pressure = 98600
        pressure_noise = float(np.random.normal(0, 0.2, 1))

        temperature_msg = uavcan.equipment.air_data.StaticTemperature(
            static_temperature=temperature+temperature_noise,
            static_temperature_variance=1)
        pressure_msg = uavcan.equipment.air_data.StaticPressure(
            static_pressure=pressure+pressure_noise,
            static_pressure_variance=1)
        return [temperature_msg, pressure_msg]


class Magnetometr(Publisher):
    def __init__(self, subscriber_manager):
        super().__init__(subscriber_manager, MAG_SUB_TOPIC, MAG_SUB_TOPIC)
        self.period = MAG_PERIOD
        self.publisher = subscriber_manager.communicator

    def fill_msg(self):
        if self.in_msgs[MAG_SUB_TOPIC] is not None:
            mag = self.in_msgs[MAG_SUB_TOPIC].magnetic_field
            magnetic_field_ga = [mag.x, mag.y, mag.z]
            magnetic_field_ga_noise = list(np.random.normal(0, 0.001, 3))
            msg = uavcan.equipment.ahrs.MagneticFieldStrength(
                magnetic_field_ga=[sum(i) for i in zip(magnetic_field_ga, magnetic_field_ga_noise)])
        else:
            msg = None
        return msg


class DiffPressure(Publisher):
    def __init__(self, subscriber_manager):
        super().__init__(subscriber_manager, None, None)
        self.period = DIFF_PRESSURE_PERIOD
        self.publisher = subscriber_manager.communicator

    def publish(self):
        crnt_time = rospy.get_time()
        if self.timestamp + self.period < crnt_time:
            msg = self.fill_msg()
            if msg is not None:
                self.publisher.publish(msg)
                self.number_of_msgs += 1
                self.timestamp = crnt_time

    def fill_msg(self):
        static_pressure = 98600
        static_pressure_noise = float(np.random.normal(0, 0.1, 1))

        differential_pressure = 0
        differential_pressure_noise = float(np.random.normal(0, 0.1, 1))

        msg = uavcan.equipment.air_data.RawAirData(
            static_pressure=static_pressure+static_pressure_noise,
            differential_pressure=differential_pressure+differential_pressure_noise)
        return msg


class PX4UavcanCommunicator:
    def __init__(self):
        rospy.init_node("px4_uavcan_communicator", log_level=rospy.DEBUG)
        self.rate = rospy.Rate(10000)
        rospy.sleep(1)

        self.publishers = list()
        self.last_log_time = 0
        self.LOG_PERIOD = 3

        self._connect_uavcan()
        self.subscriber_manager = SubscriberManager(self.communicator)
        self._subscribe_and_advertise()

    def create_uavcan_spin_thread(self):
        import threading
        threading.Thread(target=self.communicator.create_spin_thread, daemon=True).start()

    def spin_and_publish(self):
        self.rate.sleep()
        for publisher in self.publishers:
            publisher.publish()

        if self.last_log_time + self.LOG_PERIOD < rospy.get_time():
            self.last_log_time = rospy.get_time()
            hz = list(map(lambda x : int(x.number_of_msgs/self.LOG_PERIOD), self.publishers))
            rospy.logdebug("hz: act={}/gps={}/baro={}/diff_pres={}/imu={}/mag={}".format(*hz))
            for publisher in self.publishers:
                publisher.number_of_msgs = 0

    def _connect_uavcan(self):
        self.communicator = None
        while not rospy.is_shutdown() and self.communicator is None:
            try:
                self.communicator = UavcanCommunicatorV0(can_device_type="serial")
            except OSError as e:
                rospy.logerr("{}. Check you device. Trying to reconnect.".format(e))
                rospy.sleep(2)

    def _subscribe_and_advertise(self):
        self.subscriber_manager.subscribe("uavcan", uavcan.equipment.esc.RawCommand, JOY_TOPIC)
        self.subscriber_manager.subscribe("uavcan", uavcan.protocol.NodeStatus, NODE_STATUS_TOPIC)

        self.subscriber_manager.subscribe("ros",    NavSatFix,            GPS_POSITION_SUB_TOPIC)
        self.subscriber_manager.subscribe("ros",    QuaternionStamped,    ATTITUDE_SUB_TOPIC)
        self.subscriber_manager.subscribe("ros",    Twist,                VELOCITY_SUB_TOPIC)
        self.subscriber_manager.subscribe("ros",    Imu,                  IMU_SUB_TOPIC)
        self.subscriber_manager.subscribe("ros",    MagneticField,        MAG_SUB_TOPIC)


        self.publishers.append(Actuators(self.subscriber_manager))

        self.publishers.append(GPS(self.subscriber_manager))
        self.publishers.append(Baro(self.subscriber_manager))
        self.publishers.append(DiffPressure(self.subscriber_manager))
        self.publishers.append(IMU(self.subscriber_manager))
        self.publishers.append(Magnetometr(self.subscriber_manager))
        self.publishers.append(Arm(self.subscriber_manager))

if __name__=="__main__":
    px4_uavcan_communicator = PX4UavcanCommunicator()
    px4_uavcan_communicator.create_uavcan_spin_thread()
    while not rospy.is_shutdown():
        px4_uavcan_communicator.spin_and_publish()
