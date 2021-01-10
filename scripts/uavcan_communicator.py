#!/usr/bin/env python3
'''
This module allows to establish connection between PX4 and simulator via uavcan and ROS.
Example of connection for version 0: https://legacy.uavcan.org/Implementations/Pyuavcan/Tutorials/
'''

# For uavcan v0.1
import uavcan
import serial

# Common
import logging

# ROS
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class UavcanCommunicatorV0:
    """
    General-purpose uavcan communicator.
    It simply allows to create a node and provides subscribe and publish methods
    """
    def __init__(self, kawrgs, node_id=42, node_name="uavcan communicator"):
        """
        Simply create a node without starting it.

        param kawrgs - dict with specific fields in uavcan notation, for example:
        1. for serial:
        kwargs = {"can_device_name" : "/dev/ttyACM0", "baudrate" : 1000000}
        2. for can-slcan:
        kwargs = {"can_device_name" : "slcan0", "bustype" : "socketcan", "bitrate" : 1000000}
        """
        self.node_id = node_id
        self.node_name = node_name
        self.kawrgs = kawrgs
        self.node = self._create_node()
        self.handlers = []

    def subscribe(self, data_type, callback):
        """
        General-purpose subscription

        param data_type - https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/

        param callback - any function with single parameter - event

        Example:
        data_type = uavcan.protocol.NodeStatus
        callback = lambda event: print(uavcan.to_yaml(event))
        communicator.subscribe(uavcan.protocol.NodeStatus, callback)
        """
        if self.node is None:
            rospy.logerr("uavcan: can't subscribe.")
        else:
            self.handlers.append(self.node.add_handler(data_type, callback))

    def publish(self, msg, priority=uavcan.TRANSFER_PRIORITY_LOWEST):
        """
        General-purpose publication

        param msg - just instancination from
        https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/
        using constructor with corresponding positional arguments

        Example with gps:
        fix2 = uavcan.equipment.gnss.Fix2(pdop=10)
        communicator.publish(fix2)
        """
        if self.node is not None:
            self.node.broadcast(msg, priority=priority)
        else:
            rospy.logerr_throttle(5, "uavcan: can't publish.")

    def spin(self):
        if self.node is not None:
            self.node.spin()
        else:
            rospy.logerr_throttle(5, "uavcan: can't spin.")

    def start_spin(self):
        """
        Note: cycle instead of thread because v0.1 is not thread-safe
        It can't be threading.Thread(target=self.node.spin, daemon=True).start()
        """
        while True:
            try:
                self.node.spin()
            except uavcan.UAVCANException as ex:
                print('Node error:', ex)
                logging.warning('Node error! %s', ex)

    def publish_periodically(self, frequency, calback):
        period = 1 / frequency if frequency > 0 else 0
        self.node.periodic(period, gnss_publisher)

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

class PX4UavcanCommunicator:
    def __init__(self):
        self._init_params('serial')
        self._init_ros()
        self._init_uavcan()

        self._spin_and_publish()

    def _init_params(self, can_device_type):
        self.info = dict()

        if can_device_type == 'serial':
            kwargs = {
                "can_device_name" : "/dev/ttyACM0",
                "baudrate" : 1000000,
            }
        elif can_device_type == 'can-slcan':
            kwargs = {
                "can_device_name" : "slcan0",
                "bustype" : "socketcan",
                "bitrate" : 1000000,
            }
        print("PX4UavcanCommunicator config:", kwargs)
        self.params = kwargs

    def _init_ros(self):
        """ Start ROS node and perform advertising and subscription """
        rospy.init_node('px4_uavcan_communicator', log_level=rospy.DEBUG)
        self.rate = rospy.Rate(1000)

        self.actuator_publisher = rospy.Publisher('/sim/actuators', Joy, queue_size=1)

        gps_position_cb = lambda event: self.assign_msg_to_dict('gps_position', event)
        attitude_cb = lambda event: self.assign_msg_to_dict('attitude', event)
        imu_cb = lambda event: self.assign_msg_to_dict('imu', event)
        velocity_cb = lambda event: self.assign_msg_to_dict('velocity', event)

        rospy.Subscriber('/sim/gps_position',   NavSatFix,          gps_position_cb)
        rospy.Subscriber('/sim/attitude',       QuaternionStamped,  attitude_cb)
        rospy.Subscriber('/sim/imu',            Imu,                imu_cb)
        rospy.Subscriber('/sim/velocity',       Twist,              velocity_cb)

        self.info['gps_position'] = {'msg' : None}
        self.info['attitude'] = {'msg' : None}
        self.info['imu'] = {'msg' : None}
        self.info['velocity'] = {'msg' : None}

        rospy.sleep(1)
        rospy.logdebug("PX4UavcanCommunicator ROS node successfully initiated.")

    def _init_uavcan(self):
        """ Start UAVCAN node and perform subscription """
        self.communicator = UavcanCommunicatorV0(kawrgs=self.params)
        raw_command_cb = lambda event: self.assign_msg_to_dict('RawCommand', event)
        self.communicator.subscribe(uavcan.equipment.esc.RawCommand, raw_command_cb)
        self.info['RawCommand'] = {'msg' : None}
        if self.communicator.node is not None:
            rospy.logdebug("PX4UavcanCommunicator UAVCAN node successfully initiated.")
        else:
            rospy.logerr("uavcan: UAVCAN node failed.")

    def assign_msg_to_dict(self, msg_type_name, msg):
        print(msg)
        self.info[msg_type_name]['msg'] = msg
        
    def _spin_and_publish(self):
        """
        We need to publish to:
        #  name             type        period, Hz  
        1. Actuator         ros         1000        
        2. Barometr         uavcan      10
        3. Magnetometr      uavcan      10
        4. Airspeed         uavcan      10
        5. Diff pressure    uavcan      10
        6. GPS              uavcan      10
        7. Accel/gyro       uavcan      1000
        """
        self.communicator.spin()
        self.rate.sleep()

        self._publish_actuator()
        self._publish_baro()
        self._publish_diff_pressure()
        rospy.sleep(20)

    
    def _publish_actuator(self):
        in_uavcan_msg = self.info['RawCommand']['msg']
        out_ros_msg = Joy()
        out_ros_msg.header.stamp = rospy.get_rostime()
        out_ros_msg.axes = list()
        self.actuator_publisher.publish(out_ros_msg)

    def _publish_baro(self):
        in_ros_pose_msg = self.info['gps_position']['msg']
        in_ros_pose_msg = self.info['velocity']['msg']

        temperature = uavcan.equipment.air_data.StaticTemperature(static_temperature=42,
                                                                  static_temperature_variance=42)
        pressure = uavcan.equipment.air_data.StaticPressure(static_pressure=42,
                                                            static_pressure_variance=42)
        self.communicator.publish(temperature)
        self.communicator.publish(pressure)

    def _publish_mag(self):
        magnetic_field = uavcan.equipment.ahrs.MagneticFieldStrength()
        self.communicator.publish(magnetic_field)

    def _publish_diff_pressure(self):
        in_ros_pose_msg = self.info['gps_position']['msg']
        in_ros_pose_msg = self.info['velocity']['msg']

        raw_air_data = uavcan.equipment.air_data.RawAirData(static_pressure=42,
                                                            differential_pressure=42)
        self.communicator.publish(raw_air_data)

    def _publish_gps(self):
        gnss_fix = uavcan.equipment.gnss.Fix()
        self.communicator.publish(gnss_fix)

    def _publish_imu(self):
        raw_imu = uavcan.equipment.ahrs.RawIMU()
        self.communicator.publish(raw_imu)


if __name__=="__main__":
    px4_uavcan_communicator = PX4UavcanCommunicator()