#!/usr/bin/env python3
'''
This module allows to establish connection between PX4 and simulator via uavcan and ROS.
Example of connection for version 0: https://legacy.uavcan.org/Implementations/Pyuavcan/Tutorials/
'''

# Common
import logging
import sys

# ROS
import rospy
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy

# For uavcan v0.1
# import serial
import uavcan

class UavcanCommunicatorV0:
    """
    General-purpose uavcan communicator.
    It simply allows to create a node and provides subscribe and publish methods
    """
    def __init__(self, can_device_type, node_id=42, node_name="uavcan communicator"):
        """
        Simply create a node without starting it.
        param can_device_type - could be 'serial' or 'can-slcan'
        """
        self.node_id = node_id
        self.node_name = node_name
        self.handlers = []

        if can_device_type == 'serial':
            self.kawrgs = {
                "can_device_name" : "/dev/ttyACM0",
                "baudrate" : 1000000,
            }
        elif can_device_type == 'can-slcan':
            self.kawrgs = {
                "can_device_name" : "slcan0",
                "bustype" : "socketcan",
                "bitrate" : 1000000,
            }
        else:
            rospy.logerr("UavcanCommunicatorV0: Wrong can device type")
            sys.exit()

        self.node = self._create_node()

    def subscribe(self, data_type, callback):
        """
        param data_type - https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/
        param callback - any function with single parameter - event

        Example:
        data_type = uavcan.protocol.NodeStatus
        callback = lambda event: print(uavcan.to_yaml(event))
        communicator.subscribe(uavcan.protocol.NodeStatus, callback)
        """
        if self.node is None:
            rospy.logerr("UavcanCommunicatorV0: can't subscribe.")
        else:
            self.handlers.append(self.node.add_handler(data_type, callback))

    def publish(self, data_type, priority=uavcan.TRANSFER_PRIORITY_LOWEST):
        """
        param data_type - https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/
        using constructor with corresponding positional arguments

        Example with gps:
        fix2 = uavcan.equipment.gnss.Fix2(pdop=10)
        communicator.publish(fix2)
        """
        if self.node is not None:
            self.node.broadcast(data_type, priority=priority)
        else:
            rospy.logerr_throttle(5, "uavcan: can't publish.")

    def spin(self):
        """
        Note: cycle instead of thread because v0.1 is not thread-safe
        It can't be threading.Thread(target=self.node.spin, daemon=True).start()
        """
        if self.node is not None:
            try:
                self.node.spin(0.0001)
            except uavcan.transport.TransferError as e:
                print('spin Transferuavcan.transport.TransferErrorError')
        else:
            rospy.logerr_throttle(5, "uavcan: can't spin.")

    def _create_node(self):
        node_info = uavcan.protocol.GetNodeInfo.Response()
        node_info.name = self.node_name
        node_info.software_version.major = 0
        node_info.software_version.minor = 2
        node_info.hardware_version.unique_id = b'12345'
        try:
            node = uavcan.make_node(node_id=self.node_id,
                                    node_info=node_info,
                                    **self.kawrgs)
        except OSError as e:
            logging.warning(e)
            rospy.logerr(e)
            node = None
        return node


class SubscriberManager:
    """It allows to subscribe on UAVCAN or ROS and simply store data to dict by callback"""
    def __init__(self, communicator):
        self.communicator = communicator
        self.in_msgs = dict()
        self.msg_timestamps = dict()
        self.publishers = list()

    def subscribe(self, protocol_type, in_data_type, in_topic):
        callback = lambda event : self._callback_save_msg(event, in_topic)
        if protocol_type == 'ros':
            rospy.Subscriber(in_topic, in_data_type, callback)
        elif protocol_type == 'uavcan':
            self.communicator.subscribe(in_data_type, callback)

    def _callback_save_msg(self, event, in_topic):
        self.in_msgs[in_topic] = event
        self.msg_timestamps[in_topic] = rospy.get_time()
        rospy.loginfo_throttle(3, self.in_msgs[in_topic].transfer.payload)


class Publisher:
    """
    Simple wrap on publisher that has 2 key features:
    - it has access to all subscription data throught SubscriberManager field - self.in_msgs
    - it has method 'publish' that allows unifiedly perform publication of any data type"""
    def __init__(self, subscriber_manager):
        self.in_msgs = subscriber_manager.in_msgs
        self.timestamp = rospy.get_time()

    def publish(self):
        pass

    def fill_msg(self):
        return None


class IMU(Publisher):
    def __init__(self, subscriber_manager):
        super().__init__(subscriber_manager)
        self.period = 0.1
        self.communicator = subscriber_manager.communicator

    def publish(self):
        crnt_time = rospy.get_time()
        if self.timestamp + self.period < crnt_time:
            self.timestamp = crnt_time
            msg = self.fill_msg()
            if msg is not None:
                self.communicator.publish(msg)

    def fill_msg(self):
        msg = uavcan.equipment.ahrs.RawIMU(rate_gyro_latest=[0.42, 0.27, 0.13],
                                            accelerometer_latest=[0.42, 0.27, 27.13])
        return msg


class GPS(Publisher):
    def __init__(self, subscriber_manager):
        super().__init__(subscriber_manager)
        self.period = 0.1
        self.communicator = subscriber_manager.communicator

    def publish(self):
        crnt_time = rospy.get_time()
        if self.timestamp + self.period < crnt_time:
            self.timestamp = crnt_time
            msg = self.fill_msg()
            if msg is not None:
                self.communicator.publish(msg)

    def fill_msg(self):
        msg = uavcan.equipment.gnss.Fix(longitude_deg_1e8=4874268390,
                                        latitude_deg_1e8=5575444250,
                                        height_msl_mm=-6500,
                                        sats_used=10,
                                        status=3,
                                        pdop=99)
        return msg


class Actuators(Publisher):
    def __init__(self, subscriber_manager):
        super().__init__(subscriber_manager)
        self.period = 0.05
        self.publisher = rospy.Publisher('sim/actuators', Joy, queue_size=10)

    def publish(self):
        crnt_time = rospy.get_time()
        if self.timestamp + self.period < crnt_time:
            self.timestamp = crnt_time
            msg = self.fill_msg()
            if msg is not None:
                self.publisher.publish(msg)

    def fill_msg(self):
        msg = Joy()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = 'kek'
        # msg.axes.append()
        return msg


class Baro(Publisher):
    def __init__(self, subscriber_manager):
        super().__init__(subscriber_manager)
        self.period = 0.1
        self.communicator = subscriber_manager.communicator

    def publish(self):
        crnt_time = rospy.get_time()
        if self.timestamp + self.period < crnt_time:
            self.timestamp = crnt_time
            msgs = self.fill_msg()
            for msg in msgs:
                self.communicator.publish(msg)

    def fill_msg(self):
        temperature = uavcan.equipment.air_data.StaticTemperature(static_temperature=42,
                                                                  static_temperature_variance=42)
        pressure = uavcan.equipment.air_data.StaticPressure(static_pressure=27,
                                                            static_pressure_variance=42)
        return [temperature, pressure]

class Magnetometr(Publisher):
    def __init__(self, subscriber_manager):
        super().__init__(subscriber_manager)
        self.period = 0.1
        self.communicator = subscriber_manager.communicator

    def publish(self):
        crnt_time = rospy.get_time()
        if self.timestamp + self.period < crnt_time:
            self.timestamp = crnt_time
            msg = self.fill_msg()
            if msg is not None:
                self.communicator.publish(msg)

    def fill_msg(self):
        msg = uavcan.equipment.ahrs.MagneticFieldStrength()
        return msg

class DiffPressure(Publisher):
    def __init__(self, subscriber_manager):
        super().__init__(subscriber_manager)
        self.period = 0.1
        self.communicator = subscriber_manager.communicator

    def publish(self):
        crnt_time = rospy.get_time()
        if self.timestamp + self.period < crnt_time:
            self.timestamp = crnt_time
            msg = self.fill_msg()
            if msg is not None:
                self.communicator.publish(msg)

    def fill_msg(self):
        msg = uavcan.equipment.air_data.RawAirData(static_pressure=42,
                                                    differential_pressure=27)
        return msg


class PX4UavcanCommunicator:
    def __init__(self):
        rospy.init_node('px4_uavcan_communicator', log_level=rospy.DEBUG)
        self.rate = rospy.Rate(10000)
        rospy.sleep(1)

        self.communicator = UavcanCommunicatorV0(can_device_type='can-slcan')
        self.subscriber_manager = SubscriberManager(self.communicator)
        self.publishers = list()
        self._subscribe_and_advertise()

    def spin_and_publish(self):
        self.communicator.spin()
        self.rate.sleep()

        for publisher in self.publishers:
            publisher.publish()

    def _subscribe_and_advertise(self):
        self.subscriber_manager.subscribe("uavcan", uavcan.equipment.esc.RawCommand, 'Joy')
        self.subscriber_manager.subscribe("uavcan", uavcan.protocol.NodeStatus, 'NodeStatus')

        self.subscriber_manager.subscribe("ros",    NavSatFix,            '/sim/gps_position')
        self.subscriber_manager.subscribe("ros",    QuaternionStamped,    '/sim/attitude')
        self.subscriber_manager.subscribe("ros",    Twist,                '/sim/velocity')
        self.subscriber_manager.subscribe("ros",    Imu,                  '/sim/imu')

        self.publishers.append(Actuators(self.subscriber_manager))
        self.publishers.append(Baro(self.subscriber_manager))
        self.publishers.append(Magnetometr(self.subscriber_manager))
        self.publishers.append(DiffPressure(self.subscriber_manager))
        self.publishers.append(GPS(self.subscriber_manager))
        self.publishers.append(IMU(self.subscriber_manager))


if __name__=="__main__":
    px4_uavcan_communicator = PX4UavcanCommunicator()
    while not rospy.is_shutdown():
        px4_uavcan_communicator.spin_and_publish()
