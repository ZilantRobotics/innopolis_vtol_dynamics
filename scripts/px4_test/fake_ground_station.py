#!/usr/bin/env python3
'''
This script emulate ground station and establish connection with a PX4-Autopilot.
Based on https://docs.px4.io/master/en/robotics/dronekit.html
'''

import logging
try:
    import coloredlogs
    coloredlogs.install()
except:
    pass
import math
import time
from enum import Enum
from dronekit import connect, Command, LocationGlobal, APIException
from pymavlink import mavutil
mavutil.set_dialect("ardupilotmega")

DEV_PATH = "/dev/ttyACM0"
BAUD_RATE = 115200


class VehicleCommunicator:
    """ Example: https://dev.px4.io/master/en/robotics/dronekit.html """
    def __init__(self):
        self.vehicle = connect(DEV_PATH, wait_ready=True)
        self.home = self.vehicle.location.global_relative_frame
        print("home", self.home)

    def upload_mission(self):
        cmds = self.vehicle.commands
        cmds.clear()

        wp = self.get_location_offset_meters(self.home, 0, 0, 30)
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
        cmds.add(cmd)

        wp = self.get_location_offset_meters(wp, 10, 0, 0)
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
        cmds.add(cmd)

        wp = self.get_location_offset_meters(wp, 0, 10, 0)
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
        cmds.add(cmd)

        wp = self.get_location_offset_meters(wp, -10, 0, 0)
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
        cmds.add(cmd)

        wp = self.get_location_offset_meters(wp, 0, -10, 0)
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
        cmds.add(cmd)

        wp = self.get_location_offset_meters(self.home, 0, 0, 10)
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
        cmds.add(cmd)

        cmds.upload()
        time.sleep(2)

    def get_location_offset_meters(self, original_location, dNorth, dEast, alt):
        earth_radius=6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        return LocationGlobal(newlat, newlon,original_location.alt+alt)

    def subscribe(self, msg_type, callback):
        self.vehicle.add_message_listener(msg_type, callback)

    def publish(self, msg):
        self.vehicle.send_mavlink(msg)

    def reboot_vehicle(self):
        print("vehicle.reboot()")
        self.vehicle.reboot()
        time.sleep(1)

    def reconnect(self):
        self.vehicle.close()
        time.sleep(1)
        for attempt_num in range(10):
            try:
                self.vehicle = connect(DEV_PATH, wait_ready=True)
                print('attempt {} :)'.format(attempt_num))
                break
            except APIException:
                self.vehicle.close()
                print('attempt {} :('.format(attempt_num))
                time.sleep(1)

    def start_mission(self):
        print("Start_mission:")
        print("- Type: %s" % self.vehicle._vehicle_type)
        print("- Armed: %s" % self.vehicle.armed)
        print("- System status: %s" % self.vehicle.system_status.state)
        print("- GPS: %s" % self.vehicle.gps_0)
        print("- Alt: %s" % self.vehicle.location.global_relative_frame.alt)

        MAV_MODE_AUTO = 4
        self.vehicle._master.mav.command_long_send(self.vehicle._master.target_system, self.vehicle._master.target_component,
                                                   mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                                   MAV_MODE_AUTO,
                                                   0, 0, 0, 0, 0, 0)
        time.sleep(1)

        print("1. Upload mission")
        self.upload_mission()

        print("2. Arm")
        self.vehicle.armed = True
        print("self.vehicle.armed", self.vehicle.armed)

        print("3. Monitor mission execution")
        nextwaypoint = self.vehicle.commands.next
        while nextwaypoint < len(self.vehicle.commands):
            print("- nextwaypoint", nextwaypoint, "armed=", self.vehicle.armed)
            if self.vehicle.commands.next > nextwaypoint:
                display_seq = self.vehicle.commands.next + 1
                print("Moving to waypoint %s" % display_seq)
                nextwaypoint = self.vehicle.commands.next
            time.sleep(1)

        print("4. Wait for the vehicle to land")
        while self.vehicle.commands.next > 0:
            time.sleep(1)

        print("5. Disarm vehicle")
        self.vehicle.armed = False
        time.sleep(1)



if __name__=="__main__":
    # Example of usage:
    communicator = VehicleCommunicator()
    communicator.reboot_vehicle()
    communicator.reconnect()
    communicator.start_mission()