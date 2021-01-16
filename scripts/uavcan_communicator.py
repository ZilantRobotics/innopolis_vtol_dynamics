#!/usr/bin/env python3
'''
This module allows to establish connection between PX4 and simulator via uavcan and ROS.
Example of connection for version 0: https://legacy.uavcan.org/Implementations/Pyuavcan/Tutorials/
'''

# Common
import logging
import sys
from random import uniform

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
import can

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
        self.handlers = []
        self.node = None

        if can_device_type == 'serial':
            kawrgs = {"can_device_name" : "/dev/ttyACM0",
                      "baudrate" : 1000000}
        elif can_device_type == 'can-slcan':
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
            rospy.logerr(e)
        except uavcan.driver.common.TxQueueFullError as e:
            rospy.logerr(e)

    def spin(self):
        """
        Note: cycle instead of thread because v0.1 is not thread-safe
        It can't be threading.Thread(target=self.node.spin, daemon=True).start()
        """
        try:
            self.node.spin(0.00005)
        except uavcan.transport.TransferError as e:
            rospy.logerr('spin %s', e)
        except can.CanError as e:
            rospy.logerr('spin %s', e)


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
        self.in_msgs[in_topic] = None
        self.msg_timestamps[in_topic] = rospy.get_time()

    def _callback_save_msg(self, event, in_topic):
        self.in_msgs[in_topic] = event
        self.msg_timestamps[in_topic] = rospy.get_time()
        # rospy.loginfo_throttle(3, self.in_msgs[in_topic].transfer.payload)


class Publisher:
    """
    Simple wrap on publisher that has 2 key features:
    - it has access to all subscription data throught SubscriberManager field - self.in_msgs
    - it has method 'publish' that allows unifiedly perform publication of any data type"""
    def __init__(self, subscriber_manager):
        self.in_msgs = subscriber_manager.in_msgs
        self.timestamp = rospy.get_time()
        self.msg_timestamps = subscriber_manager.msg_timestamps

    def publish(self):
        """ Publish if:
        - timestamp of last received data is less than period + crnt_time
        - timestamp of last sended data is bigger than period + crnt_time
        """
        pass

    def fill_msg(self):
        return None


class IMU(Publisher):
    def __init__(self, subscriber_manager):
        super().__init__(subscriber_manager)
        self.period = 0.005
        self.communicator = subscriber_manager.communicator
        self.number_of_published_msg = 0

    def publish(self):
        crnt_time = rospy.get_time()
        if self.timestamp + self.period < crnt_time:
            self.timestamp = crnt_time
            msg = self.fill_msg()
            if msg is not None:
                self.number_of_published_msg += 1
                self.communicator.publish(msg)

    def fill_msg(self):
        gyro = [0.00, 0.00, 0.1]
        gyro_noise = [uniform(-0.0001, 0.0001), uniform(-0.0001, 0.0001), uniform(-0.0001, 0.0001)]

        accel = [0.00, 0.00, -19.81]
        accel_noise = [uniform(-0.01, 0.01), uniform(-0.01, 0.01), uniform(-0.01, 0.01)]

        msg = uavcan.equipment.ahrs.RawIMU(
            rate_gyro_latest=[sum(i) for i in zip(gyro, gyro_noise)],
            accelerometer_latest=[sum(i) for i in zip(accel, accel_noise)])
        return msg


class GPS(Publisher):
    def __init__(self, subscriber_manager):
        super().__init__(subscriber_manager)
        self.period = 0.1
        self.communicator = subscriber_manager.communicator

    def publish(self):
        crnt_time = rospy.get_time()

        last_mgs_timestamp = self.msg_timestamps['/sim/gps_position']

        if last_mgs_timestamp > crnt_time - self.period and self.timestamp < crnt_time - self.period:
            self.timestamp = crnt_time
            msg = self.fill_msg()
            if msg is not None:
                self.communicator.publish(msg)

    def fill_msg(self):
        if self.in_msgs['/sim/gps_position'] is None:
            msg = None
        else:
            geodetioc_pose = [int(self.in_msgs['/sim/gps_position'].latitude),
                              int(self.in_msgs['/sim/gps_position'].longitude),
                              int(self.in_msgs['/sim/gps_position'].altitude)]
            msg = uavcan.equipment.gnss.Fix(longitude_deg_1e8=geodetioc_pose[0],
                                            latitude_deg_1e8=geodetioc_pose[1],
                                            height_msl_mm=geodetioc_pose[2],
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
        temperature = 42.42+273
        temperature_noise = uniform(0, 1)

        pressure = 98600
        pressure_noise = uniform(0, 2)

        temperature_msg = uavcan.equipment.air_data.StaticTemperature(
            static_temperature=temperature+temperature_noise,
            static_temperature_variance=1)
        pressure_msg = uavcan.equipment.air_data.StaticPressure(
            static_pressure=pressure+pressure_noise,
            static_pressure_variance=1)
        return [temperature_msg, pressure_msg]

class Magnetometr(Publisher):
    def __init__(self, subscriber_manager):
        super().__init__(subscriber_manager)
        self.period = 0.1
        self.communicator = subscriber_manager.communicator

    def publish(self):
        crnt_time = rospy.get_time()

        attitude_timestamp = self.msg_timestamps['/sim/attitude']
        gps_position_timestamp = self.msg_timestamps['/sim/gps_position']
        last_mgs_timestamp = min(attitude_timestamp, gps_position_timestamp)

        if last_mgs_timestamp > crnt_time - self.period and self.timestamp < crnt_time - self.period:
            self.timestamp = crnt_time
            msg = self.fill_msg()
            if msg is not None:
                self.communicator.publish(msg)

    def fill_msg(self):
        if self.in_msgs['/sim/attitude'] is None or self.in_msgs['/sim/gps_position'] is None:
            msg = None
        else:
            attitude = self.in_msgs['/sim/attitude'].quaternion
            geodetioc_pose = [self.in_msgs['/sim/gps_position'].latitude,
                              self.in_msgs['/sim/gps_position'].longitude,
                              self.in_msgs['/sim/gps_position'].altitude]
            # [+1.60e-05, -0.38e-05, -5.17e-05]
            magnetic_field_ga = [-0.0720, +0.1050, 0.3480]
            magnetic_field_ga_noise = [0.001, 0.001, 0.001]

            msg = uavcan.equipment.ahrs.MagneticFieldStrength(
                magnetic_field_ga=[sum(i) for i in zip(magnetic_field_ga, magnetic_field_ga_noise)])
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
        static_pressure = 98600
        static_pressure_noise = uniform(0, 1)

        differential_pressure = 0
        differential_pressure_noise = uniform(0, 1)

        msg = uavcan.equipment.air_data.RawAirData(
            static_pressure=static_pressure+static_pressure_noise,
            differential_pressure=differential_pressure+differential_pressure_noise)
        return msg


class PX4UavcanCommunicator:
    def __init__(self):
        rospy.init_node('px4_uavcan_communicator', log_level=rospy.DEBUG)
        self.rate = rospy.Rate(1000)
        rospy.sleep(1)

        self.publishers = list()
        self.uavcan_spin_time = 0
        self.ros_spin_time = 0
        self.publishing_time = 0
        self.last_log_time = 0
        self.log_period = 1

        self.communicator = None
        while not rospy.is_shutdown() and self.communicator is None:
            try:
                self.communicator = UavcanCommunicatorV0(can_device_type='can-slcan')
                self.subscriber_manager = SubscriberManager(self.communicator)
                self._subscribe_and_advertise()
            except OSError as e:
                rospy.logerr("{}. Check you device. Trying to reconnect.".format(e))
                rospy.sleep(2)


    def spin_and_publish(self):
        start_time = rospy.get_time()
        self.communicator.spin()
        self.uavcan_spin_time += rospy.get_time() - start_time

        start_time = rospy.get_time()
        self.rate.sleep()
        self.ros_spin_time += rospy.get_time() - start_time

        start_time = rospy.get_time()
        for publisher in self.publishers:
            publisher.publish()
        self.publishing_time += rospy.get_time() - start_time

        rospy.logdebug_throttle(1, "Log: time usage: uavcan={}, ros={}, pub={}; imu_pub_times={}".format(
            round(self.uavcan_spin_time, 1),
            round(self.ros_spin_time, 2),
            round(self.publishing_time, 1),
            self.publishers[5].number_of_published_msg))

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
