from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import logging
import sys
import os
import threading

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config

logging.basicConfig(
    format='%(asctime)s - %(pathname)s[line:%(lineno)d] - %(levelname)s: %(message)s',
    level=logging.DEBUG)
b_test = False
b_stop = False


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi *
                                            original_location.lat / 180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobal(newlat, newlon, original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

class DroneKitControl:
    def __init__(self, connection_string, logger=None):
        if logger is None:
            self.logger = logging
        else:
            self.logger = logger
        self.connection_string = connection_string

        self.vehicle = None
        # Connect to the Vehicle
        self.connect_drone()
        print(dir(self.vehicle))
        print(self.vehicle.system_status)

    def connect_drone(self):
        self.logger.info('Connecting to vehicle on: %s' % self.connection_string)
        try:
            self.vehicle = connect(self.connection_string, wait_ready=True)
        except Exception as e:
            self.logger.error({'connect drone error': e})

    def distance_to_current_waypoint(self):
        """
        Gets distance in metres to the current waypoint.
        It returns None for the first waypoint (Home location).
        """
        nextwaypoint = self.vehicle.commands.next
        if nextwaypoint == 0:
            return None
        # commands are zero indexed
        missionitem = self.vehicle.commands[nextwaypoint - 1]
        lat = missionitem.x
        lon = missionitem.y
        alt = missionitem.z
        targetWaypointLocation = LocationGlobalRelative(lat, lon, alt)
        distancetopoint = get_distance_metres(
            self.vehicle.location.global_frame,
            targetWaypointLocation)
        return distancetopoint

    def download_mission(self, clear=False):
        """
        Download the current mission from the vehicle.
        """
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()  # wait until download is complete.
        if clear:
            cmds.clear()
            cmds.upload()
            print(" Clear any existing commands")

    def adds_square_mission(self, aLocation, aSize):
        """
        Adds a takeoff command and four waypoint commands to the current mission.
        The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).
        The function assumes vehicle.commands matches the vehicle mission state
        (you must have called download at least once in the session and after clearing the mission)
        """

        cmds = self.vehicle.commands

        print(" Clear any existing commands")
        cmds.clear()

        print(" Define/add new commands.")
        # Add new commands. The meaning/order of the parameters is documented
        # in the Command class.

        # Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is
        # already in the air.
        cmds.add(
            Command(
                0,
                0,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                10))
        # Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
        point1 = get_location_metres(aLocation, aSize, -aSize)
        point2 = get_location_metres(aLocation, aSize, aSize)
        point3 = get_location_metres(aLocation, -aSize, aSize)
        point4 = get_location_metres(aLocation, -aSize, -aSize)
        cmds.add(
            Command(
                0,
                0,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                0,
                0,
                0,
                0,
                0,
                point1.lat,
                point1.lon,
                0))
        cmds.add(
            Command(
                0,
                0,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                0,
                0,
                0,
                0,
                0,
                point2.lat,
                point2.lon,
                0))
        cmds.add(
            Command(
                0,
                0,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                0,
                0,
                0,
                0,
                0,
                point3.lat,
                point3.lon,
                0))
        cmds.add(
            Command(
                0,
                0,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                0,
                0,
                0,
                0,
                0,
                point4.lat,
                point4.lon,
                0))
        # add dummy waypoint "5" at point 4 (lets us know when have reached
        # destination)
        cmds.add(
            Command(
                0,
                0,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                0,
                0,
                0,
                0,
                0,
                point4.lat,
                point4.lon,
                0))

        print(" Upload new commands to vehicle")
        cmds.upload()

    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode

        while not self.vehicle.mode.name == 'GUIDED' and not self.vehicle.armed:
            print(" Getting ready to take off ...")
            time.sleep(1)

        a_location = LocationGlobalRelative(-34.364114, 149.166022, 30)
        self.vehicle.simple_goto(a_location)
        print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        # vehicle.simple_takeoff(aTargetAltitude)  # Take off to target
        # altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * \
                    0.95:  # Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)
            # Take off to target altitude
            self.vehicle.simple_takeoff(aTargetAltitude)

    def arm(self, mode='MANUAL', b_check_aemable=False):
        """
            Arms vehicle and fly to aTargetAltitude.
            """
        # vehicle.mode = VehicleMode("GUIDED")
        # mode = "GUIDED"
        # mode = "AUTO"
        self.vehicle.mode = VehicleMode(mode)
        if b_check_aemable:
            self.logger.info("Basic pre-arm checks")
            # Don't let the user try to arm until autopilot is ready
            while not self.vehicle.is_armable:
                print(" Waiting for vehicle to initialise...")
                time.sleep(1)
        self.vehicle.armed = True
        print("Arming motors")
        # Copter should arm in GUIDED mode
        print('vehicle.mode.name', self.vehicle.mode.name)
        print('vehicle.armed', self.vehicle.armed)
        while not self.vehicle.mode.name == mode and not self.vehicle.armed:
            print(" Getting ready to take off ...")
            time.sleep(1)

    # From Copter 3.3 you will be able to take off using a mission item. Plane
    # must take off using a mission item (currently).

    # vehicle.airspeed=10
    # vehicle.groundspeed=20

    def simple_point(self):
        # 单点飞行
        self.arm()
        # 清除任务
        self.download_mission(clear=True)

        aLocation = self.vehicle.location.global_frame
        target_point = get_location_metres(aLocation, 50, -50)
        point1 = LocationGlobalRelative(-35.961354, 149.165218, 10)
        print('simple_goto')
        # self.vehicle.simple_takeoff(0)
        # vehicle.
        self.vehicle.simple_goto(target_point, groundspeed=10)

        print('sleep(1000)')
        # time.sleep(10)
        while True:
            distance = get_distance_metres(
                self.vehicle.location.global_frame, target_point)
            print('Distance to :', distance)
            time.sleep(1)

    def multi_points(self):
        self.download_mission(clear=True)
        # 任务航行模式
        self.arm()
        # arm_and_takeoff(10)

        print('Create a new mission (for current location)')
        # 初始经纬度 -35.363261,149.165230
        home_point1 = LocationGlobalRelative(-35.961354, 149.165218, 10)
        home_point2 = self.vehicle.location.global_frame
        print('vehicle.location.global_frame',
              self.vehicle.location.global_frame)
        self.adds_square_mission(home_point2, 200)
        # Reset mission set to first (0) waypoint
        self.vehicle.commands.next = 0

        # Set mode to AUTO to start mission
        self.vehicle.mode = VehicleMode("AUTO")
        # vehicle.c
        # Monitor mission.
        # Demonstrates getting and setting the command number
        # Uses distance_to_current_waypoint(), a convenience function for finding the
        #   distance to the next waypoint.
        print("Starting mission")

        while True:
            nextwaypoint = self.vehicle.commands.next
            print('Distance to waypoint (%s): %s' %
                  (nextwaypoint, self.distance_to_current_waypoint()))

            if nextwaypoint == 3:  # Skip to next waypoint
                print('Skipping to Waypoint 5 when reach waypoint 3')
                self.vehicle.commands.next = 5
            # Dummy waypoint - as soon as we reach waypoint 4 this is true and
            # we exit.
            if nextwaypoint == 5:
                print("Exit 'standard' mission when start heading to final waypoint (5)")
                break
            time.sleep(1)

        print('Return to launch')
        self.vehicle.mode = VehicleMode("RTL")

        # Close vehicle object before exiting script
        print("Close vehicle object")
        self.vehicle.close()

    def move_square(self, harf_w):
        """
        移动一个正方形区域
        :param harf_w:
        :return:
        """
        # 清除之前任务
        self.download_mission(clear=True)
        # 任务航行模式
        print('Create a new mission (for current location)')
        # 初始经纬度 -35.363261,149.165230
        current_point = self.vehicle.location.global_frame
        print('vehicle.location.global_frame', current_point)
        self.adds_square_mission(current_point, harf_w)
        # Reset mission set to first (0) waypoint
        self.vehicle.commands.next = 0

        # Set mode to AUTO to start mission
        # self.vehicle.mode = VehicleMode("AUTO")
        self.mode_control("AUTO")
        # vehicle.c
        # Monitor mission.
        # Demonstrates getting and setting the command number
        # Uses distance_to_current_waypoint(), a convenience function for finding the
        #   distance to the next waypoint.
        print("Starting mission")

        while True:
            try:
                nextwaypoint = self.vehicle.commands.next
                print('Distance to waypoint (%s): %s' % (nextwaypoint, self.distance_to_current_waypoint()))
                # if nextwaypoint == 3:  # Skip to next waypoint
                #     print('Skipping to Waypoint 5 when reach waypoint 3')
                #     self.vehicle.commands.next = 5
                # Dummy waypoint - as soon as we reach waypoint 4 this is true and
                # we exit.
                if nextwaypoint == 5:
                    print("Exit 'standard' mission when start heading to final waypoint (5)")
                    break
                time.sleep(1)
            except KeyboardInterrupt:
                print({'error': e})
                break
        # print('Return to launch')
        # self.vehicle.mode = VehicleMode("RTL")
        # Close vehicle object before exiting script
        # print("Close vehicle object")
        # self.vehicle.close()

    def point_control(self, dNorth, dEast, b_stop=False, gotoFunction=None, arrive_distance=4):
        """
        Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

        The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for
        the target position. This allows it to be called with different position-setting commands.
        By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

        The method reports the distance to target every two seconds.
        """
        if gotoFunction is None:
            gotoFunction = self.vehicle.simple_goto
        currentLocation = self.vehicle.location.global_relative_frame
        targetLocation = get_location_metres(currentLocation, dNorth, dEast)
        targetDistance = get_distance_metres(currentLocation, targetLocation)
        gotoFunction(targetLocation)

        # # Stop action if we are no longer in guided mode.
        while self.vehicle.mode.name == "GUIDED" and not b_stop:
            try:
                # print "DEBUG: mode: %s" % vehicle.mode.name
                remainingDistance = get_distance_metres(
                    self.vehicle.location.global_relative_frame, targetLocation)
                self.logger.info({"Distance to target: ": remainingDistance})
                # Just below target, in case of undershoot.
                if remainingDistance <= arrive_distance:
                    print("Reached target")
                    break
                time.sleep(1)
            except Exception as e:
                self.logger.error({'error': e})
                break

    # 控制速度
    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors and
        for the specified duration.

        This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only
        velocity components
        (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).

        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version
        (sending the message multiple times does not cause problems).

        See the above link for information on the type_mask (0=enable, 1=ignore).
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0, 0,
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # vehicle.send_mavlink(msg)
        # send command to vehicle on 1 Hz cycle
        for x in range(0, duration):
            print('vehicle.velocity', self.vehicle.velocity)
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

    # Fly south and up.
    def goto_position_target_global_int(self, aLocation):
        """
        Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

        For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

        See the above link for information on the type_mask (0=enable, 1=ignore).
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b0000111111111000,  # type_mask (only speeds enabled)
            aLocation.lat * 1e7,  # lat_int - X Position in WGS84 frame in 1e7 * meters
            aLocation.lon * 1e7,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
            aLocation.alt,
            # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
            0,  # X velocity in NED frame in m/s
            0,  # Y velocity in NED frame in m/s
            0,  # Z velocity in NED frame in m/s
            # afx, afy, afz acceleration (not supported yet, ignored in
            # GCS_Mavlink)
            0, 0, 0,
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def yaw_control(self, heading, relative=False):
        """
        Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

        This method sets an absolute heading by default, but you can set the `relative` parameter
        to `True` to set yaw relative to the current yaw heading.

        By default the yaw of the vehicle will follow the direction of travel. After setting
        the yaw using this function there is no way to return to the default yaw "follow direction
        of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

        For more information see:
        http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
        """
        if relative:
            is_relative = 1  # yaw relative to direction of travel
        else:
            is_relative = 0  # yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            1,  # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)  # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        # while True:
        #     print('vehicle.attitude', self.vehicle.attitude)
        #     time.sleep(1)

    def speed_control(
            self,
            velocity_x,
            velocity_y,
            velocity_z,
            duration):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        msg1 = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0,  # lat_int - X Position in WGS84 frame in 1e7 * meters
            0,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0,
            # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            velocity_x,  # X velocity in NED frame in m/s
            velocity_y,  # Y velocity in NED frame in m/s
            velocity_z,  # Z velocity in NED frame in m/s
            0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle on 1 Hz cycle
        self.vehicle.send_mavlink(msg1)
        for x in range(0, duration):
            self.vehicle.send_mavlink(msg1)
            print('vehicle.velocity', self.vehicle.velocity)
            time.sleep(1)

    def channel_control(self, channel_pwm):
        """
        控制通道输出pwm波
        :param channel_pwm 通道和pwm波字典
        :return:
        """
        current_1 = self.vehicle.channels['1']
        current_3 = self.vehicle.channels['3']
        target_1 = channel_pwm['1']
        target_3 = channel_pwm['3']
        if current_1 == 0 or current_3 == 0 or current_1 is None or current_3 is None:
            current_1 = 1500
            current_3 = 1500
            overrides_pwd = {'1': current_1, '3': current_3}
            self.vehicle.channels.overrides = overrides_pwd
            print(" Channel overrides: %s" % self.vehicle.channels.overrides)
            time.sleep(4)

        overrides_pwd = {'1': current_1, '3': current_3}
        self.vehicle.channels.overrides = overrides_pwd
        print(" Channel overrides: %s" % self.vehicle.channels.overrides)
        try:
            while current_1 != target_1 or current_3 != target_3:
                current_1 = current_1 + (target_1 - current_1) // abs(target_1 - current_1) * 100
                current_3 = current_3 + (target_3 - current_3) // abs(target_3 - current_3) * 100
                overrides_pwd = {'1': current_1, '3': current_3}
                self.vehicle.channels.overrides = overrides_pwd
                print(" Channel overrides: %s" % self.vehicle.channels.overrides)
                time.sleep(0.1)

        except BaseException:
            self.vehicle.channels.overrides = {'1': 1500, '2': 1500, '3': 1500}

    def arm_control(self, armed):
        """
        :param armed: True  arm or False disarm
        :return:
        """
        print('current vehicle armed', drone_obj.vehicle.armed)
        current_armed = drone_obj.vehicle.armed
        if current_armed == armed:
            pass
        else:
            drone_obj.vehicle.armed = armed
            while drone_obj.vehicle.armed != armed:
                print('wait switch arm')
                time.sleep(1)
            print('target vehicle armed', drone_obj.vehicle.armed)

    def mode_control(self, mode):
        """
        控制arm与disarm,控制模式切换
        :param mode: 模式  'MANUAL'  'GUIDED'  'RTL'  "AUTO"
        :return:
        """
        print('current vehicle mode', drone_obj.vehicle.mode)
        current_mode = drone_obj.vehicle.armed
        if current_mode == mode:
            pass
        else:
            # 切换模式需要先disarm
            # if self.vehicle.armed:
            #     self.arm_control(False)
            while drone_obj.vehicle.mode != mode:
                drone_obj.vehicle.mode = VehicleMode(mode)
                print('wait switch mode:%s' % (mode))
                time.sleep(1)
            # self.arm_control(True)
        print('current vehicle mode', drone_obj.vehicle.mode)

    def get_set_home_location(self, set_home_location_list=None):
        """
        获取和设置home位置，传入home_location_list表示设置，不传表示获取
        :param home_location_list:[纬度，经度]
        :return:
        """
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        home = drone_obj.vehicle.home_location
        try:
            if home is None:
                print('home还未初始化成功')
            else:
                print('current vehicle.home_location', home.alt, home.lat, home.lon)
                if set_home_location_list is not None:
                    drone_obj.vehicle.home_location = LocationGlobal(set_home_location_list[1],
                                                                     set_home_location_list[0], home.alt)
        except Exception as e:
            print({'error': e})

    def get_current_location(self):
        try:
            current_location = self.vehicle.location.global_relative_frame
            if current_location is None:
                print({'currentLocation': current_location})
            else:
                lng_lat = [current_location.lon, current_location.lat, current_location.alt]
                print({'lng_lat': lng_lat})
        except Exception as e:
            print({'error': e})


# mavproxy.py --master="com22" --baudrate=57600 --out=COM17:5660
# --out=tcp:127.0.0.1:5662
# 1是右侧  3是左侧电机
if __name__ == '__main__':
    # pix_port = '/dev/ttyACM0'
    pix_port = 'tcp:127.0.0.1:5760'
    pix_baud = 115200
    b_use_pix = False
    drone_obj = DroneKitControl(pix_port)
    drone_obj.download_mission(True)
    # simple_point()
    # multi_points()
    while True:
        try:
            print("Channel values from RC Tx:", drone_obj.vehicle.channels)
            # w,a,s,d 为前后左右，q为后退 按键后需要按回车才能生效
            key_input = input('please input:')
            # 前 后 左 右 停止  1为右侧电机是反桨叶  3位左侧电机是正桨叶
            gear = None
            if key_input.startswith('w') or key_input.startswith('a') or key_input.startswith(
                    's') or key_input.startswith('d'):
                try:
                    gear = int(key_input[1])
                except Exception as e:
                    print({'error': e})
                    gear = None
            if key_input.startswith('w'):
                if gear is not None:
                    drone_obj.channel_control({'1': 1600 + 100 * gear, '3': 1600 + 100 * gear})
                else:
                    drone_obj.channel_control({'1': 1800, '3': 1800})
            elif key_input.startswith('a'):
                if gear is not None:
                    drone_obj.channel_control({'1': 1600 + 100 * gear, '3': 1400 - 100 * gear})
                else:
                    drone_obj.channel_control({'1': 1800, '3': 1200})
            elif key_input.startswith('s'):
                if gear is not None:
                    drone_obj.channel_control({'1': 1400 - 100 * gear, '3': 1400 - 100 * gear})
                else:
                    drone_obj.channel_control({'1': 1200, '3': 1200})
            elif key_input.startswith('d'):
                if gear is not None:
                    drone_obj.channel_control({'1': 1400 - 100 * gear, '3': 1600 + 100 * gear})
                else:
                    drone_obj.channel_control({'1': 1200, '3': 1800})
            elif key_input == 'q':
                drone_obj.channel_control({'1': 1500, '3': 1500})

            # arm
            elif key_input == 'z':
                drone_obj.arm_control(True)
            # disarm
            elif key_input == 'x':
                drone_obj.arm_control(False)

            # manual模式
            elif key_input == 'm':
                drone_obj.mode_control('MANUAL')
            # guide模式
            elif key_input == 'g':
                drone_obj.mode_control('GUIDED')
            # b 回家
            elif key_input == 'b':
                drone_obj.mode_control('RTL')

            # c 清除所有任务
            elif key_input == 'c':
                drone_obj.download_mission(clear=True)

            # 获取当前GPS位置和船头朝向
            elif key_input == 'l':
                drone_obj.get_current_location()
                print("船头方向: %s" % drone_obj.vehicle.heading)

            # 设置返航点h114.110000,30.120000
            elif key_input.startswith('h'):
                try:
                    if len(key_input) <= 3:
                        # 获取
                        drone_obj.get_set_home_location()
                    else:
                        str_lon, str_lat = key_input[1:].split(',')
                        lon, lat = float(str_lon), float(str_lat)
                        # 设置
                        drone_obj.get_set_home_location(set_home_location_list=[lon, lat])
                except Exception as e:
                    print({'error': e})

            # 角度控制
            elif key_input.startswith('r'):
                try:
                    theta = int(key_input[1:])
                    print('theta:', theta)
                    # 转换角度方向
                    theta = 360 - theta
                    drone_obj.yaw_control(theta, True)
                except Exception as e:
                    print({'error': e})

            # 运动方向速度控制
            elif key_input.startswith('n'):
                speed_x = None
                speed_y = None
                try:
                    str_x, str_y = key_input[1:].split(',')
                    speed_x = int(str_x)
                    speed_y = int(str_y)
                    print(speed_x, speed_y)
                    drone_obj.speed_control(speed_x, speed_y, 0, 10)
                except Exception as e:
                    print({'error': e})
            elif key_input.startswith('v'):
                drone_obj.simple_point()

            # 到达目标点控制
            elif key_input.startswith('t'):
                point_x = None
                point_y = None
                try:
                    str_x, str_y = key_input[1:].split(',')
                    point_x = int(str_x)
                    point_y = int(str_y)
                    print(point_x, point_y)
                    # 控制点需要guided模式、
                    drone_obj.mode_control('GUIDED')
                    drone_obj.point_control(point_x, point_y, False)
                    # p_thread= threading.Thread(target=drone_obj.point_control,args=(point_x, point_y,b_stop))
                    # p_thread.start()
                    # while not b_stop:
                    #     try:
                    #         currentLocation = drone_obj.vehicle.location.global_relative_frame
                    #         targetLocation = get_location_metres(currentLocation, point_x, point_x)
                    #         remainingDistance = get_distance_metres(
                    #         drone_obj.vehicle.location.global_relative_frame, targetLocation)
                    #         print({"Distance to target: ":remainingDistance})
                    #         # Just below target, in case of undershoot.
                    #         if remainingDistance <= 4:
                    #             print("Reached target")
                    #             break
                    #         print('wait to arriver point...')
                    #         # key_input = input('please input:')
                    #         # if int(key_input)==0:
                    #         #     break
                    #         time.sleep(1)
                    #     except Exception as e:
                    #         print({'error':e})
                    #         b_stop=True
                    #         break
                    # b_stop=True
                except Exception as e:
                    print({'error': e})

            # 简单走矩形区域
            elif key_input.startswith('p'):
                # 半边长
                half_w = int(key_input[1:])
                drone_obj.move_square(half_w)

        except Exception as e:
            print({'error': e})
            drone_obj.mode_control('MANUAL')
            drone_obj.arm_control(armed=False)
            # drone_obj.vehicle.close()
            continue
