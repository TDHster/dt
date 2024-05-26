from pymavlink import mavutil
from takeoff import takeoff
from time import sleep

mav_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14540')

mav_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (mav_connection.target_system, mav_connection.target_component))


# takeoff by hand
# Arm the UAS
mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
# Command Takeoff
takeoff_altitude = 1
mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                     0, 0, 0, 0, 0, 0, 0,
                                     takeoff_altitude)

takeoff_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
print(f"Takeoff ACK:  {takeoff_msg}")
# end takeoff by hand

#------
# takeoff(mav_connection, 1)
#------

mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)


def set_yaw(mavlink_connection, yaw, yaw_rate=25):
    """Set yaw of MAVLink client.

     Args:
         connection: A pymavlink MAVLink connection object.
         yaw: The yaw angle to set.
         yaw_rate: The yaw rate to set deg per second.
         direction: The direction to set. -1 for left, 1 for right.
         abs_rel_flag: The absolute/relative flag to set. 0 for absolute, 1 for relative.
     """
    # if autopilot == "ardupilotmega":
    if yaw < 0:
        direction = -1  # CCW
    else:
        direction = 1  # CW

    yaw = abs(yaw)

    YAW_CHANGE_RELATIVE = 0
    YAW_CHANGE_ABSOLUTE = 1

    abs_rel_flag = YAW_CHANGE_RELATIVE  #0 - relative, 1 - abs(0 - is North)

    mavlink_connection.mav.command_long_send(
        mavlink_connection.target_system,
        mavlink_connection.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, yaw, yaw_rate, direction, abs_rel_flag, 0, 0, 0)


if __name__ == '__main__':
    set_yaw(mav_connection, 10)
    sleep(3)
    set_yaw(mav_connection, 10)
    sleep(3)
    set_yaw(mav_connection, 20)
    sleep(3)
    set_yaw(mav_connection, 30)
    sleep(3)
    set_yaw(mav_connection, -10)
    sleep(3)
    set_yaw(mav_connection, -20)
    sleep(3)
    set_yaw(mav_connection, -30)
