from pymavlink import mavutil
from time import sleep, time

from takeoff import takeoff

import argparse

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


def make_movement(mav_connection, x, y, z):
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
    mav_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            time_boot_ms, mav_connection.target_system,
            mav_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            x, y, z,
            velocity_x, velocity_y, velocity_z,
            axel_x, axel_y, axel_z,
            yaw, yaw_rate
        )
    )


# the_connection.mav.send(
#     mavutil.mavlink.MAVLink_set_position_target_global_int_message(
#         10, the_connection.target_system,
#         the_connection.target_component,
#         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#         int(0b110111111000),
#         int(-35.3629849 * 10 ** 7),
#         int(149.1649185 * 10 ** 7), 10,
#         0, 0, 0, 0, 0, 0, 1.57, 0.5))

def receive_mavlink_message(mav_connection, type='LOCAL_POSITION_NED', blocking=True):
    msg = mav_connection.recv_match(
        type='LOCAL_POSITION_NED', blocking=blocking)
    print(msg)
    return msg


def run_for_seconds(duration_seconds, mav_connection):
    start_time = time()
    end_time = start_time + duration_seconds

    while time() < end_time:
        # Perform the loop iteration here
        receive_mavlink_message(mav_connection)
        # Add a slight delay to avoid overwhelming the system
        sleep(0.01)


if __name__ == '__main__':

    # Start a connection listening to a UDP port
    # the_connection = mavutil.mavlink_connection('udpout:localhost:14550')  # original connection string
    # mav_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14540')
    mav_connection = mavutil.mavlink_connection(connection_string)

    # Wait for the first heartbeat
    #   This sets the system and component ID of remote system for the link
    mav_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    # mav_connection.set_mode('POSHOLD')
    takeoff(mav_connection, 1)

    time_between_wpts = 4

    make_movement(mav_connection, 1, 0, -1)
    run_for_seconds(time_between_wpts, mav_connection)
    # sleep(time_between_wpts)
    make_movement(mav_connection, 0, 1, -1)
    run_for_seconds(time_between_wpts, mav_connection)
    # sleep(time_between_wpts)
    make_movement(mav_connection, -1, 0, -1)
    run_for_seconds(time_between_wpts, mav_connection)
    # sleep(time_between_wpts)
    make_movement(mav_connection, 0, -1, -1)
    run_for_seconds(time_between_wpts, mav_connection)
    # sleep(time_between_wpts)

    while True:
        receive_mavlink_message(mav_connection)
