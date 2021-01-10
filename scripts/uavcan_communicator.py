#!/usr/bin/env python3
'''
This module allows to establish connection between uavcan network and landing station.
Example of connection for version 0: https://legacy.uavcan.org/Implementations/Pyuavcan/Tutorials/
'''

# For uavcan v.0
import uavcan
import serial

import threading
import errno
import logging


class UavcanCommunicatorV0():
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
        communicator.subscribe(uavcan.protocol.NodeStatus, node_status_callback)
        """
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
        self.node.broadcast(msg, priority=priority)

    def start_spin(self):
        """
        Note: cycle instead of thread because v.0 is not thread-safe
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
        logging.info("Periodic publisher was subscribed")

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
            exit(-1)
        return node


if __name__=="__main__":
    # Init params
    can_device_type = 'serial'
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
    
    # Instanciate communicator
    communicator = UavcanCommunicatorV0(kawrgs=kwargs)

    # Example with subscription on uavcan.protocol.NodeStatus
    communicator.subscribe(uavcan.protocol.NodeStatus, node_status_callback)

    # Example with gnss
    fix2 = uavcan.equipment.gnss.Fix2(pdop=10)
    gnss_publisher = lambda : communicator.publish(fix2)
    communicator.publish_periodically(frequency=1, calback=gnss_publisher)
    
    # Example with rangefinder
    measurement = uavcan.equipment.range_sensor.Measurement(sensor_type=2, range=10)
    rangefinder_publisher = lambda : communicator.publish(measurement)
    communicator.publish_periodically(frequency=1, calback=rangefinder_publisher)

    communicator.start_spin()