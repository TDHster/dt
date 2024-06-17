from pymavlink import mavutil
# from pymavlink_iq_utilites import *
from time import sleep, time
import argparse
from bcolors import bcolors

parser = argparse.ArgumentParser(description="Mavlink сontrol")

parser.add_argument(
    "-c",
    "--connectionstring",
    type=str,
    default="udpin:127.0.0.1:14550",
    help="Specify path for mavlink connection",
)

args = parser.parse_args()
connection_string = args.connectionstring
print(f"Using mavlink connection string: {connection_string}")


class MavlinkDrone:
    _vx, _vy, _vz, _yaw = 0, 0, 0, 0
    TO_TARGET_VELOCITY = 2  # m/s

    def __init__(self, connection_string='udpin:localhost:14550'):
        '''
          Args:
            connection_string:
            The mavutil.mavlink_connection() connection string has the format:

            [protocol:]address[:port]
            where:

            protocol (optional): The IP protocol. If not specified pymavlink will attempt to determine if the address is a serial port (e.g. USB) or a file, and if not will default to a UDP address.
            tcp: Initiate a TCP connection on the specified address and port.
            tcpin: Listen for a TCP connection on the specified address and port.
            udpin: Listen for a UDP connection on the specified address and port.
            udpout: Initiate a TCP connection on the specified address and port.
            udp: By default, same as udpin. Set mavlink_connection parameter input=False to make same as udpout.
            udpcast: Broadcast UDP address and port. This is the same as udp with mavlink_connection() parameters input=False and broadcast=True.
            address: IP address, serial port name, or file name
            port: IP port (only if address is an IP address)

        '''
        self.connection = mavutil.mavlink_connection(connection_string)
        self.connection.wait_heartbeat()
        # print("Heartbeat from system (system %u component %u)" %
        #       (self.connection.target_system, self.connection.target_component))
        self.autopilot_info = self._get_autopilot_info()
        print(f'Connected to {self.autopilot_info["autopilot"]} autopilot')

        self._moving_to_target = False


    def _arm(self, arm: bool = True, force_code=0):
        self.connection.wait_heartbeat()
        if arm:
            arm_command = 1
        else:
            arm_command = 0

        self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component,
                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                              0, arm_command, force_code, 0, 0, 0, 0, 0)
        arm_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if arm_msg:
            if arm_msg.result == 0:
                print(f"{bcolors.OKGREEN}Arm command ok.{bcolors.ENDC} {force_code=}")
                if arm:
                    print("Waiting motors arming...")
                    self.connection.motors_armed_wait()
                    print(f'{bcolors.OKGREEN}Motors start confirmed.{bcolors.ENDC}')
                    return True
            else:
                print(f"{bcolors.FAIL}Arm ACK: {bcolors.ENDC} {arm_msg.result}")
        else:
            print(f'{bcolors.FAIL}Arm command FAIL (no COMMAND_ACK){bcolors.ENDC}')
        return False


    def arm(self):
        print('Usual arm')
        self._arm(arm=True)

    def arm_2989(self):
        print('Force arm, code 2989')
        self._arm(arm=True, force_code=2989)

    def arm_21196(self):
        print('Force arm, code 21196')
        self._arm(arm=True, force_code=21196)

    def disarm(self):
        self._arm(arm=False)

    def disarm_2989(self):
        print('Force disarm, code 2989')
        self._arm(arm=False, force_code=2989)

    def disarm_21196(self):
        print('Force disarm, code 21196')
        self._arm(arm=False, force_code=21196)

    def emergency_stop(self):
        self.disarm_21196()
        self.disarm_2989()
        self.disarm()
        # self.mode_brake()
        self.mode_land()
        # self.disarm()

    def _wait_heartbeat(self, timeout=3):
        """
        Waits for a HEARTBEAT message from the given sysid

        Arguments:
        connection -- pymavlink connection object
        sysid      -- system id to search for
        timeout    -- time to wait for the message in seconds

        Returns:
        The HEARTBEAT message or None if timeout is reached
        """
        # self.connection.wait_heartbeat()
        # print("Heartbeat from system (system %u component %u)" %
        #       (self.connection.target_system, self.connection.target_component))

        start_time = time()
        while time() - start_time < timeout:
            msg = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
            if msg.get_srcSystem() == self.connection.target_system:
                print(f"{bcolors.OKGREEN}Heartbeat{bcolors.ENDC} from system")
                return msg
        return None

    def takeoff(self, takeoff_altitude=1):
        # self.mode_guided()
        # self.arm_2989()
        self._takeoff_cmd_nav_takeoff(takeoff_altitude)

    def _takeoff_cmd_nav_takeoff(self, takeoff_altitude=1):

        self._wait_heartbeat()

        self._wait_until_position_aiding(self.connection)

        # autopilot_info = get_autopilot_info(self.connection, tgt_sys_id)
        # autopilot_info = get_autopilot_info(self.connection, self.connection.target_system)

        if self.autopilot_info["autopilot"] == "ardupilotmega":
            print("Connected to ArduPilot autopilot")
            mode_id = self.connection.mode_mapping()["GUIDED"]
            takeoff_params = [0, 0, 0, 0, 0, 0, takeoff_altitude]

        elif self.autopilot_info["autopilot"] == "px4":
            print("Connected to PX4 autopilot")
            print(self.connection.mode_mapping())
            mode_id = self.connection.mode_mapping()["TAKEOFF"][1]
            print(mode_id)
            msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            starting_alt = msg.alt / 1000
            takeoff_params = [0, 0, 0, 0, float("NAN"), float("NAN"), starting_alt + takeoff_altitude]

        else:
            raise ValueError("Autopilot not supported")

        # Change mode to guided (Ardupilot) or takeoff (PX4)
        self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component,
                                              mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                              0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id,
                                              0, 0, 0, 0, 0)
        ack_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Change Mode ACK:  {bcolors.OKGREEN}{ack_msg}{bcolors.ENDC}")

        # self.arm()
        self.arm_2989()
        # self.arm_21196()

        # Command Takeoff
        self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component,
                                              mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, takeoff_params[0],
                                              takeoff_params[1], takeoff_params[2], takeoff_params[3],
                                              takeoff_params[4], takeoff_params[5], takeoff_params[6])

        takeoff_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if takeoff_msg:
            if takeoff_msg.result == 0:
                print(f"{bcolors.OKGREEN}Takeoff command OK.{bcolors.ENDC}")
            else:
                print(f"{bcolors.FAIL}Takeoff command ACK not ok: {takeoff_msg.result}{bcolors.ENDC}")
            return takeoff_msg.result

        else:
            print(f"{bcolors.FAIL}Takeoff command fail (no COMMAND_ACK){bcolors.ENDC}")
        return None

    def mode_land(self):
        self._set_mode('LAND')

    def mode_guided(self):
        self._set_mode('GUIDED')

    def mode_guided_nogps(self):
        self._set_mode('GUIDED_NOGPS')

    def mode_acro(self):
        self._set_mode('ACRO')

    def mode_alt_hold(self):
        self._set_mode('ALT_HOLD')

    def mode_position_hold(self):
        self._set_mode('POSHOLD')

    def mode_stabilize(self):
        self._set_mode('STABILIZE')

    def mode_brake(self):
        self._set_mode('BRAKE')

    def mode_rtl(self):
        self._set_mode('RTL')

    def _set_mode(self, mode='LAND'):
        '''
        Modes: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'LOITER', 'RTL', 'CIRCLE', 'POSITION', 'LAND',
                'OF_LOITER', 'DRIFT', 'SPORT', 'FLIP', 'AUTOTUNE', 'POSHOLD', 'BRAKE', 'THROW', 'AVOID_ADSB',
                'GUIDED_NOGPS', 'SMART_RTL', 'FLOWHOLD', 'FOLLOW', 'ZIGZAG', 'SYSTEMID', 'AUTOROTATE', 'AUTO_RTL']
        '''
        self._mode = mode
        self.connection.set_mode(mode)
        ack_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack_msg:
            if ack_msg.result == 0:
                print(f"{bcolors.OKCYAN}{mode} mode{bcolors.ENDC} set OK")
            else:
                print(f"Change Mode to {bcolors.FAIL}{mode}{bcolors.ENDC} FAIL: {ack_msg.result}")
        else:
            print(f'{bcolors.FAIL}Set mode command{bcolors.ENDC} return None.')

    def set_yaw(self, yaw_angle=0, yaw_rate=0):
        """Set yaw of MAVLink client.

         Args:
             connection: A pymavlink MAVLink connection object.
             yaw_angle: The yaw angle to set.
             yaw_rate: The yaw rate to set deg per second.
             direction: The direction to set. -1 for left, 1 for right.
             abs_rel_flag: The absolute/relative flag to set. 0 for absolute, 1 for relative.
         """
        self._yaw = yaw_angle

        if yaw_angle < 0:
            direction = -1  # CCW
        else:
            direction = 1  # CW

        yaw_angle = abs(yaw_angle)

        YAW_CHANGE_RELATIVE = 0
        # YAW_CHANGE_ABSOLUTE = 1

        abs_rel_flag = YAW_CHANGE_RELATIVE  # 0 - relative, 1 - abs(0 - is North)

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, yaw_angle, yaw_rate, direction, abs_rel_flag, 0, 0, 0)

    def move(self, relative_x=0, relative_y=0, relative_z=0,
             velocity_x=0, velocity_y=0, velocity_z=0,
             axel_x=0, axel_y=0, axel_z=0,
             type_mask='Use Velocity'):
        """
        time_boot_ms	Sender's system time in milliseconds since boot (any random 10 ms ex.)
        target_system	System ID of vehicle
        target_component	Component ID of flight controller or just 0
        coordinate_frame	Valid options are listed below
        type_mask
        Bitmask to indicate which fields should be ignored by the vehicle (see POSITION_TARGET_TYPEMASK enum)

        bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, bit7:AccX, bit8:AccY, bit9:AccZ, bit11:yaw, bit12:yaw rate

        When providing Pos, Vel and/or Accel all 3 axis must be provided. At least one of Pos, Vel and Accel must be provided
        (e.g. providing Yaw or YawRate alone is not supported)
        Use Position :     0b110111111000 / 0x0DF8 / 3576 (decimal)
        Use Velocity :     0b110111000111 / 0x0DC7 / 3527 (decimal)
        Use Acceleration : 0b110000111111 / 0x0C3F / 3135 (decimal)
        Use Pos+Vel :      0b110111000000 / 0x0DC0 / 3520 (decimal)
        Use Pos+Vel+Accel: 0b110000000000 / 0x0C00 / 3072 (decimal)
        Use Yaw :          0b100111111111 / 0x09FF / 2559 (decimal)
        Use Yaw Rate :     0b010111111111 / 0x05FF / 1535 (decimal)

        x	X Position in meters (positive is forward or North)
        y	Y Position in meters (positive is right or East)
        z	Z Position in meters (positive is down)
        vx	X velocity in m/s (positive is forward or North)
        vy	Y velocity in m/s (positive is right or East)
        vz	Z velocity in m/s (positive is down)
        afx	X acceleration in m/s/s (positive is forward or North)
        afy	Y acceleration in m/s/s (positive is right or East)
        afz	Z acceleration in m/s/s (positive is down)
        yaw	yaw or heading in radians (0 is forward or North)
        yaw_rate	yaw rate in rad/s
        """

        time_boot_ms = 10  # ms - just for some value

        # yaw bit set to use with 0 value
        if type == 'Use Position':
            type_mask = int(0b100111111000)
        elif type == 'Use Velocity':
            type_mask = int(0b100111000111)
        elif type == 'Use Acceleration':
            type_mask = int(0b100000111111)

        # x, y, z = 0, 0, -1
        # velocity_x, velocity_y, velocity_z = 0, 0, 0
        # axel_x, axel_y, axel_z = 0, 0, 0
        yaw = 0  # radians
        yaw_rate = 0
        x, y, z = relative_x, relative_y, relative_z
        self._vx, self._vy, self._vz = velocity_x, velocity_y, velocity_z
        if self._moving_to_target:
            velocity_x = self.TO_TARGET_VELOCITY

        self.connection.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                time_boot_ms, self.connection.target_system,
                # self.connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                self.connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                type_mask,
                x, y, z,
                velocity_x, velocity_y, velocity_z,
                axel_x, axel_y, axel_z,
                yaw, yaw_rate
            )
        )

    def hover(self):
        self.move(velocity_x=0, velocity_y=0, velocity_z=0)
        self.set_yaw(0)


    @property
    def yaw(self):
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        self._yaw = value
        self.set_yaw(self._yaw)


    @property
    def vx(self):
        return self._vx

    @vx.setter
    def vx(self, value):
        self._vx = value
        self.move(velocity_x=self._vx, velocity_y=self._vy, velocity_z=self._vz)

    @property
    def vy(self):
        return self._vy

    @vy.setter
    def vy(self, value):
        self._vy = value
        self.move(velocity_x=self._vx, velocity_y=self._vy, velocity_z=self._vz)

    @property
    def vz(self):
        return self._vz

    @vz.setter
    def vz(self, value):
        self._vz = value
        self.move(velocity_x=self._vx, velocity_y=self._vy, velocity_z=self._vz)

    def to_target(self, safety=True):
        self._moving_to_target = True
        self.move(velocity_x=self.TO_TARGET_VELOCITY, type_mask='Use Velocity')
        if safety:
            sleep(0.5)
            self.hover()
            # self.move(velocity_x=0, type_mask='Use Velocity')
            self._moving_to_target = False

    @property
    def moving_to_target(self):
        return self._moving_to_target

    @moving_to_target.setter
    def moving_to_target(self, value):
        self._moving_to_target = value
        self.to_target()

    def _get_message(self, type='LOCAL_POSITION_NED', blocking=True):
        msg = self.connection.recv_match(type=type, blocking=blocking)
        # print(msg)
        return msg

    def get_message_local_position_ned(self):
        return self._get_message(type='LOCAL_POSITION_NED')

    def get_message_attitude(self):
        return self._get_message(type='ATTITUDE')

    # pymvalink_iq utility functions
    def _wait_until_position_aiding(self, timeout=120):
        """
        Wait until the MAVLink connection has EKF position aiding.

        Args:
            mav_connection (mavutil.mavlink_connection): The MAVLink connection to check.
            timeout (int, optional): The maximum time to wait for EKF position aiding in seconds. Defaults to 120.

        Raises:
            TimeoutError: If EKF position aiding is not achieved within the specified timeout.

        Returns:
            None
        """
        autopilot_info = self._get_autopilot_info()
        if autopilot_info["autopilot"] == "ardupilotmega":
            estimator_status_msg = "EKF_STATUS_REPORT"
        elif autopilot_info["autopilot"] == "px4":
            estimator_status_msg = "ESTIMATOR_STATUS"
        else:
            raise ValueError("Autopilot not supported")

        flags = ["EKF_PRED_POS_HORIZ_REL", "EKF_PRED_POS_HORIZ_REL"]
        time_start = time()
        while True:
            if self._ekf_pos_aiding(flags, estimator_status_msg) or time() - time_start > timeout:
                break
            print(f"Waiting for position aiding: {time() - time_start} seconds elapsed")

        if time() - time_start > timeout:
            raise TimeoutError(f"Position aiding did not become available within {timeout} seconds")

    def _ekf_pos_aiding(self, flags, estimator_status_msg="ESTIMATOR_STATUS"):
        """
        Check the EKF position aiding status of a MAVLink connection.

        Args:
            mav_connection (mavutil.mavlink_connection): The MAVLink connection to check.
            flags (List[str]): The flags to check in the EKF status.
            estimator_status_msg (str, optional): The name of the estimator status message. Defaults to "ESTIMATOR_STATUS".

        Returns:
            bool: True if all flags are present in the EKF status, False otherwise.
        """
        msg = self.connection.recv_match(type=estimator_status_msg, blocking=True, timeout=3)
        if not msg:
            raise ValueError(f"No message of type {estimator_status_msg} received within the timeout")

        print(f"from sysid {msg.get_srcSystem()} {msg}")
        ekf_flags = msg.flags

        for flag in flags:
            flag_val = self._get_enum_value_by_name(mavutil.mavlink.enums["EKF_STATUS_FLAGS"], flag)
            if not ekf_flags & flag_val:
                return False

        return True

    def _get_enum_value_by_name(self, enum_dict, name):
        """
        Get the value of an enum entry by its name.

        Args:
            enum_dict (Dict[str, Enum]): The enum dictionary to search.
            name (str): The name of the enum entry.

        Returns:
            int: The value of the enum entry.

        Raises:
            ValueError: If no enum entry with the given name is found.
        """
        for key, enum_entry in enum_dict.items():
            if enum_entry.name == name:
                return key
        raise ValueError("No enum entry with name: " + name)

    def _get_autopilot_info(self):
        """
        Get the autopilot information for the MAVLink connection.

        Args:
            connection (mavutil.mavlink_connection): The MAVLink connection.
            sysid (int, optional): System ID to search for. Defaults to 1.

        Returns:
            dict: A dictionary containing autopilot info.
                  Includes "autopilot", "type", and "version".
        """
        # Initialize dictionary to hold autopilot info
        autopilot_info = {"autopilot": "", "type": "", "version": ""}

        # Receive 'HEARTBEAT' message from MAVLink connection
        msg = self._wait_heartbeat()
        # If no message received, return empty autopilot_info
        if not msg:
            return autopilot_info

        # Get autopilot type and MAV type from message and add to autopilot_info
        autopilot = mavutil.mavlink.enums['MAV_AUTOPILOT'][msg.autopilot].name.replace("MAV_AUTOPILOT_", "").lower()
        autopilot_info["autopilot"] = autopilot
        autopilot_info["type"] = mavutil.mavlink.enums['MAV_TYPE'][msg.type].name.replace("MAV_TYPE_", "").lower()

        # If autopilot type is ArduPilot Mega, request autopilot version and add to autopilot_info. I don't think this is implemented for PX4.
        if autopilot_info["autopilot"] == "ardupilotmega":
            msg = self._request_autopilot_version()
            if msg:
                autopilot_info["version"] = self._get_fc_version_from_msg(msg)

        # Return the autopilot information
        return autopilot_info

    def _get_fc_version_from_msg(self, msg):
        """Extract and return the flight controller version from an AUTOPILOT_VERSION message"""
        major = str(msg.flight_sw_version >> 24)
        sub = str(msg.flight_sw_version >> 16 & 0xFF)
        rev = str(msg.flight_sw_version >> 8 & 0xFF)
        return f"{major}.{sub}.{rev}"

    def _request_autopilot_version(self):
        """Request and return an AUTOPILOT_VERSION message from a mavlink connection"""
        self.connection.mav.send(mavutil.mavlink.MAVLink_autopilot_version_request_message(
            self.connection.target_system, self.connection.target_component))
        return self.connection.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=3)


if __name__ == '__main__':
    wait_time = 3
    box_size = 2
    altitude = 1
    # mavproxy.py --master=/dev/ttyACM0 --out=udpout:0.0.0.0:14550
    # connection_string = 'udpin:127.0.0.1:14540' # working

    # connection_string = 'udpout:127.0.0.1:14550'
    # connection_string = 'tcp:localhost:5063'

    #working config:
    # udpin:127.0.0.1:14550 =  udpin:127.0.0.1:14550
    # pi@raspberrypi:~ $ mavproxy.py --master=/dev/ttyACM0 --out=udpout:0.0.0.0:14550
    # arm uncheck all
    print(f'Trying to connect: {connection_string}')
    drone = MavlinkDrone(connection_string)
    # drone = MavlinkDrone('udpout:localhost:14550')
    print('Arming')
    drone.arm()
    print('Taking off')
    sleep(wait_time)
    exit(0)
    # print(drone.get_message_local_position_ned())
    print('yaw left')
    drone.yaw(-30)
    print(drone.get_message_local_position_ned())
    sleep(wait_time)

    drone.yaw(60)
    print('yaw right')
    print(drone.get_message_local_position_ned())
    sleep(wait_time)



    print(f'Forward')
    drone.move_test(box_size, 0, -altitude)
    print(drone.get_message_local_position_ned())
    sleep(wait_time)

    print('Landing command')
    drone.land_now()
    drone.disarm()
    exit(0)


    # print(drone.get_message_local_position_ned())
    print(f'Right')
    drone.move_test(0, box_size, -altitude)
    # print(drone.get_message_local_position_ned())
    sleep(wait_time)

    # print(drone.get_message_local_position_ned())
    print(f'Back')
    drone.move_test(-box_size, 0, -altitude)
    sleep(wait_time)

    # print(drone.get_message_local_position_ned())
    print(f'Left')
    drone.move_test(0, -box_size, -altitude)
    sleep(wait_time)

    print(drone.get_message_local_position_ned())
    sleep(wait_time)

    print(drone.get_message_local_position_ned())
    print('Landing command')
    drone.land_now()
    exit(0)

    # drone.move(box_size, 0, -1)
    # print(drone.get_message_local_position_ned())
    # sleep(wait_time)
    # drone.move(0, box_size, -1)
    # print(drone.get_message_local_position_ned())
    # sleep(wait_time)
    # drone.move(-box_size, 0, -1)
    # print(drone.get_message_local_position_ned())
    # sleep(wait_time)
    # drone.move(0, -box_size, -1)
    # print(drone.get_message_local_position_ned())
    # sleep(wait_time)
    # print(drone.get_message_local_position_ned())
    drone.land_now()
    while True:
        print(drone.get_message_local_position_ned())
