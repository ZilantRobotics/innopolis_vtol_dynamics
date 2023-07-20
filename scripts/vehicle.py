#!/usr/bin/env python3
import time
import struct
import yaml
import os
import sys
import serial
from pymavlink import mavutil
from pymavlink.mavutil import mavlink

SOURCE_SYSTEM = 2
SOURCE_COMPONENT = 1

MAV_PARAM_TYPE_INT8 = 2
MAV_PARAM_TYPE_INT16 = 4
MAV_PARAM_TYPE_INT32 = 6
MAV_PARAM_TYPE_REAL32 = 9
PARAM_TYPE_TO_STRING = {
    MAV_PARAM_TYPE_INT8 : "INT8",
    MAV_PARAM_TYPE_INT16 : "INT16",
    MAV_PARAM_TYPE_INT32 : "INT32",
    MAV_PARAM_TYPE_REAL32 : "FLOAT"
}

def float_to_integer(float_number):
    return struct.unpack('!I', struct.pack('!f', float_number))[0]

def integer_to_float(int_number):
    return struct.unpack('!f', struct.pack('!I', int_number))[0]

def is_integer_param(param_type):
    integer_types = [
        MAV_PARAM_TYPE_INT8,
        MAV_PARAM_TYPE_INT16,
        MAV_PARAM_TYPE_INT32
    ]
    return True if param_type in integer_types else False
        

class Vehicle:
    """
    autopilot should be either `PX4` or `ArduPilot`
    """
    def __init__(self, autopilot) -> None:
        self.autopilot = autopilot

    def connect(self):
        while True:
            try:
                self.device_path = Vehicle.get_autopilot_path(self.autopilot)
                if self.device_path is not None:
                    self._connect()
                    print(f"Connected: {self.device_path}")
                    break
            except serial.serialutil.SerialException:
                pass

            time.sleep(1)
            print(f"Waiting for the Autopilot {self.device_path}...")

    @staticmethod
    def get_autopilot_path(autopilot):
        serial_devices = os.popen('ls /dev/serial/by-id').read().splitlines()
        device_path = None
        for serial_device in serial_devices:
            if -1 != serial_device.find(autopilot):
                device_path = serial_device
                break
        return f"/dev/serial/by-id/{device_path}"

    def configure(self, file_with_params, verbose=False):
        self._read_yaml_parameters(file_with_params)

        for param_name in self.params:
            param_value = self.params[param_name]
            self.set_specific_param(param_name, param_value)
            self.read_specific_param(param_name, verbose)

        self.reboot()
        time.sleep(2)
        self.connect()

    def _read_yaml_parameters(self, filename, verbose=False):
        with open(filename) as f:
            self.params = yaml.load(f, Loader=yaml.FullLoader)
        
        if verbose:
            print(f"{filename} has : {self.params}")

    def _connect(self):
        self.master = mavutil.mavlink_connection(
            self.device_path,
            source_component=SOURCE_COMPONENT,
            source_system=SOURCE_SYSTEM)
        self.master.mav.heartbeat_send(
            type=mavlink.MAV_TYPE_CHARGING_STATION,
            autopilot=6,
            base_mode=12,
            custom_mode=0,
            system_status=4)
        self.master.wait_heartbeat()

        print("Heartbeat from system (system %u component %u)" % (self.master.target_system, self.master.target_component))

    def download_log(self):
        self.master.mav.log_request_list_send(
            self.master.target_system,
            self.master.target_component,
            0,
            1024)

        ack_msg = None
        while ack_msg is None:
            ack_msg = self.master.recv_match(type='LOG_ENTRY', blocking=False)
        print(f"last log number is {ack_msg.last_log_num}")

        self.master.mav.log_request_data_send(
            self.master.target_system,
            self.master.target_component,
            id=ack_msg.last_log_num,
            ofs=0,
            count=90)

        ack_msg = None
        while ack_msg is None:
            ack_msg = self.master.recv_match(type='LOG_DATA', blocking=False)
        print(f"loaded: {ack_msg.ofs}, {ack_msg.count}")

    def read_all_params(self):
        self.master.mav.param_request_list_send(
            self.master.target_system, self.master.target_component
        )
        self.params = {}
        prev_recv_time_sec = time.time()
        while prev_recv_time_sec + 1.0 > time.time():
            time.sleep(0.01)
            try:
                message = self.master.recv_match(type='PARAM_VALUE', blocking=False)
                if message is not None:
                    if message.param_type == MAV_PARAM_TYPE_INT32:
                        message.param_value = float_to_integer(message.param_value)
                    message = message.to_dict()
                    self.params[message['param_id']] = message['param_value']
                    print(f"name: {message['param_id']} value: {message['param_value']}")
                    prev_recv_time_sec = time.time()
            except Exception as error:
                print(error)
                sys.exit(0)
        print("Done!")

    def read_specific_param(self, param_name, verbose=False):
        if verbose:
            print(f"{param_name: <18}", end = '', flush=True)

        for attempt in range(100):
            self.master.mav.param_request_read_send(
                self.master.target_system,
                self.master.target_component,
                bytes(param_name, 'utf-8'),
                -1
            )

            msg = self.master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
            recv_msg_name = msg['param_id']
            recv_msg_type = PARAM_TYPE_TO_STRING[msg['param_type']]
            if is_integer_param(msg['param_type']):
                msg['param_value'] = int(msg['param_value'])

            if recv_msg_name == param_name:
                if verbose:
                    print(f"{recv_msg_type: <6} {msg['param_value']}")
                break


    def set_specific_param(self, param_name, param_value):
        if isinstance(param_value, int):
            param_value = integer_to_float(param_value)

        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            bytes(param_name, 'utf-8'),
            param_value,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )
        message = self.master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
        if message['param_type'] == MAV_PARAM_TYPE_INT32:
            message['param_value'] = float_to_integer(message['param_value'])

    def reset_params_to_default(self):
        self._reset_params_to_default()
        self.reboot()
        time.sleep(2)
        self.connect()

    def _reset_params_to_default(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
            0,
            2, -1, 0, 0, 0, 0, 0)

    def force_calibrate(self):
        param2 = 76
        param5 = 76
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                          0, param2, 0, 0, param5, 0, 0)

    def reboot(self):
        self.master.reboot_autopilot()
        self.master.close()
