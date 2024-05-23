import time


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
