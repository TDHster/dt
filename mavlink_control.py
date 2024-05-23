from pymavlink import mavutil
import time

class MavlinkDron:
    def __init__(self, connection_string='udpin:localhost:14550'):
        self.connection = mavutil.mavlink_connection(connection_string)
        self.connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" %
              (self.connection.target_system, self.connection.target_component))
        self.autopilot_info = get_autopilot_info(self.connection, self.connection.target_system)
        print(f"Connected to {self.autopilot_info["autopilot"]} autopilot")

    def _arm(self, arm_command=1):
        self.connection.wait_heartbeat()
        self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_command, 0, 0, 0, 0, 0, 0)
        msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)
        return msg

    def arm(self):
        print('Arming')
        return self._arm(1)

    def disarm(self):
        print('Disrming')
        return self._arm(0)

    def set_mode(self, mode):
        pass

    def takeoff(self, takeoff_altitude=1,  tgt_sys_id: int = 1, tgt_comp_id=1):

        print("Heartbeat from system (system %u component %u)" %
              (self.connection.target_system, self.connection.target_component))

        wait_until_position_aiding(self.connection)

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
        self.connection.mav.command_long_send(tgt_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                             0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0,
                                             0)
        ack_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Change Mode ACK:  {ack_msg}")

        # Arm the UAS
        # self.connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
        #                                      mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        #
        # arm_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        # print(f"Arm ACK:  {arm_msg}")
        self.arm()

        # Command Takeoff
        self.connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, takeoff_params[0],
                                             takeoff_params[1], takeoff_params[2], takeoff_params[3], takeoff_params[4],
                                             takeoff_params[5], takeoff_params[6])

        takeoff_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Takeoff ACK:  {takeoff_msg}")

        return takeoff_msg.result

    def land(self):
        pass

    def yaw(self, yaw_angle, yaw_rate=25):
        """Set yaw of MAVLink client.

         Args:
             connection: A pymavlink MAVLink connection object.
             yaw_angle: The yaw angle to set.
             yaw_rate: The yaw rate to set deg per second.
             direction: The direction to set. -1 for left, 1 for right.
             abs_rel_flag: The absolute/relative flag to set. 0 for absolute, 1 for relative.
         """
        # if autopilot == "ardupilotmega":
        if yaw_angle < 0:
            direction = -1  # CCW
        else:
            direction = 1  # CW

        yaw_angle = abs(yaw_angle)

        YAW_CHANGE_RELATIVE = 0
        YAW_CHANGE_ABSOLUTE = 1

        abs_rel_flag = YAW_CHANGE_RELATIVE  # 0 - relative, 1 - abs(0 - is North)

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, yaw_angle, yaw_rate, direction, abs_rel_flag, 0, 0, 0)

    def move(self, relative_x, relative_y, relative_z):
        '''
            time_boot_ms	Sender's system time in milliseconds since boot (any random 10 ms ex.)
            target_system	System ID of vehicle
            target_component	Component ID of flight controller or just 0
            coordinate_frame	Valid options are listed below
            type_mask
            Bitmask to indicate which fields should be ignored by the vehicle (see POSITION_TARGET_TYPEMASK enum)

            bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, bit7:AccX, bit8:AccY, bit9:AccZ, bit11:yaw, bit12:yaw rate

            When providing Pos, Vel and/or Accel all 3 axis must be provided. At least one of Pos, Vel and Accel must be provided
            (e.g. providing Yaw or YawRate alone is not supported)
            Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)
            Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)
            Use Acceleration : 0b110000111111 / 0x0C3F / 3135 (decimal)
            Use Pos+Vel : 0b110111000000 / 0x0DC0 / 3520 (decimal)
            Use Pos+Vel+Accel : 0b110000000000 / 0x0C00 / 3072 (decimal)
            Use Yaw : 0b100111111111 / 0x09FF / 2559 (decimal)
            Use Yaw Rate : 0b010111111111 / 0x05FF / 1535 (decimal)

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
            '''

        time_boot_ms = 10  # ms
        type_mask = int(0b010111111000)
        # x, y, z = 0, 0, -1
        velocity_x, velocity_y, velocity_z = 0, 0, 0
        axel_x, axel_y, axel_z = 0, 0, 0
        yaw = 0  # radians
        yaw_rate = 0
        x, y, z = relative_x, relative_y, relative_z
        self.connection.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                time_boot_ms, self.connection.target_system,
                self.connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                x, y, z,
                velocity_x, velocity_y, velocity_z,
                axel_x, axel_y, axel_z,
                yaw, yaw_rate
            )
        )

    def _get_message(self, type='LOCAL_POSITION_NED', blocking=True):
        msg = self.connection.recv_match(
            type=type, blocking=blocking)
        # print(msg)
        return msg

    def get_message_local_position_ned(self):
        return self._get_message(type='LOCAL_POSITION_NED')

    def get_message_attitude(self):
        return self._get_message(type='ATTITUDE')

    def emergency_stop(self):
        #Terminate flight immediately. Crash landing.

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, 1, 0, 0, 0, 0, 0, 0)

    def return_to_launch(self):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0)

    def land_now(self):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)


def get_enum_value_by_name(enum_dict, name):
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


def wait_until_position_aiding(mav_connection, timeout=120):
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
    autopilot_info = get_autopilot_info(mav_connection)
    if autopilot_info["autopilot"] == "ardupilotmega":
        estimator_status_msg = "EKF_STATUS_REPORT"
    elif autopilot_info["autopilot"] == "px4":
        estimator_status_msg = "ESTIMATOR_STATUS"
    else:
        raise ValueError("Autopilot not supported")

    flags = ["EKF_PRED_POS_HORIZ_REL", "EKF_PRED_POS_HORIZ_REL"]
    time_start = time.time()
    while True:
        if ekf_pos_aiding(mav_connection, flags, estimator_status_msg) or time.time() - time_start > timeout:
            break
        print(f"Waiting for position aiding: {time.time() - time_start} seconds elapsed")

    if time.time() - time_start > timeout:
        raise TimeoutError(f"Position aiding did not become available within {timeout} seconds")



def ekf_pos_aiding(mav_connection, flags, estimator_status_msg="ESTIMATOR_STATUS"):
    """
    Check the EKF position aiding status of a MAVLink connection.

    Args:
        mav_connection (mavutil.mavlink_connection): The MAVLink connection to check.
        flags (List[str]): The flags to check in the EKF status.
        estimator_status_msg (str, optional): The name of the estimator status message. Defaults to "ESTIMATOR_STATUS".

    Returns:
        bool: True if all flags are present in the EKF status, False otherwise.
    """
    msg = mav_connection.recv_match(type=estimator_status_msg, blocking=True, timeout=3)
    if not msg:
        raise ValueError(f"No message of type {estimator_status_msg} received within the timeout")

    print(f"from sysid {msg.get_srcSystem()} {msg}")
    ekf_flags = msg.flags

    for flag in flags:
        flag_val = get_enum_value_by_name(mavutil.mavlink.enums["EKF_STATUS_FLAGS"], flag)
        if not ekf_flags & flag_val:
            return False

    return True


def get_autopilot_info(connection, sysid=1):
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
    msg = wait_for_heartbeat(connection, sysid)
    # If no message received, return empty autopilot_info
    if not msg:
        return autopilot_info

    # Get autopilot type and MAV type from message and add to autopilot_info
    autopilot = mavutil.mavlink.enums['MAV_AUTOPILOT'][msg.autopilot].name.replace("MAV_AUTOPILOT_", "").lower()
    autopilot_info["autopilot"] = autopilot
    autopilot_info["type"] = mavutil.mavlink.enums['MAV_TYPE'][msg.type].name.replace("MAV_TYPE_", "").lower()

    # If autopilot type is ArduPilot Mega, request autopilot version and add to autopilot_info. I don't think this is implemented for PX4.
    if autopilot_info["autopilot"] == "ardupilotmega":
        msg = request_autopilot_version(connection)
        if msg:
            autopilot_info["version"] = get_fc_version_from_msg(msg)

    # Return the autopilot information
    return autopilot_info


def wait_for_heartbeat(connection, sysid, timeout=3):
    """
    Waits for a HEARTBEAT message from the given sysid

    Arguments:
    connection -- pymavlink connection object
    sysid      -- system id to search for
    timeout    -- time to wait for the message in seconds

    Returns:
    The HEARTBEAT message or None if timeout is reached
    """
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
        if msg.get_srcSystem() == sysid:
            return msg
    return None


def request_autopilot_version(connection):
    """Request and return an AUTOPILOT_VERSION message from a mavlink connection"""
    connection.mav.send(mavutil.mavlink.MAVLink_autopilot_version_request_message(
        connection.target_system, connection.target_component))
    return connection.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=3)


def get_fc_version_from_msg(msg):
    """Extract and return the flight controller version from an AUTOPILOT_VERSION message"""
    major = str(msg.flight_sw_version >> 24)
    sub = str(msg.flight_sw_version >> 16 & 0xFF)
    rev = str(msg.flight_sw_version >> 8 & 0xFF)
    return f"{major}.{sub}.{rev}"
