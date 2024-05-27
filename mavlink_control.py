from pymavlink import mavutil
from pymavlink_iq_utilites import *
from time import sleep
import argparse

# Create the parser object
parser = argparse.ArgumentParser(description="MavlinkcControl")

# Add an argument for the camera type with a default value
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
    def __init__(self, connection_string='udpin:localhost:14550'):
        self.connection = mavutil.mavlink_connection(connection_string)
        self.connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" %
              (self.connection.target_system, self.connection.target_component))
        self.autopilot_info = get_autopilot_info(self.connection, self.connection.target_system)
        print(f'Connected to {self.autopilot_info["autopilot"]} autopilot')

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

    def mode_land(self):
        self._set_mode('LAND')

    def _set_mode(self, mode='LAND'):
        '''
        Modes: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'LOITER', 'RTL', 'CIRCLE', 'POSITION', 'LAND',
                'OF_LOITER', 'DRIFT', 'SPORT', 'FLIP', 'AUTOTUNE', 'POSHOLD', 'BRAKE', 'THROW', 'AVOID_ADSB',
                'GUIDED_NOGPS', 'SMART_RTL', 'FLOWHOLD', 'FOLLOW', 'ZIGZAG', 'SYSTEMID', 'AUTOROTATE', 'AUTO_RTL']
        '''
        self.connection.set_mode(mode)

    def yaw(self, yaw_angle, yaw_rate=5):
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

    def move(self, relative_x=0, relative_y=0, relative_z=0,
             velocity_x=0, velocity_y=0, velocity_z=0,
             axel_x=0, axel_y=0, axel_z=0,
             type_mask='Use Position'):
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

        time_boot_ms = 10  # ms - just for some value

        if type == 'Use Position':
            type_mask = int(0b110111111000)
        elif type == 'Use Velocity':
            type_mask = int(0b110111000111)
        elif type == 'Use Acceleration':
            type_mask = int(0b110000111111)

        # x, y, z = 0, 0, -1
        # velocity_x, velocity_y, velocity_z = 0, 0, 0
        # axel_x, axel_y, axel_z = 0, 0, 0
        yaw = 0  # radians
        yaw_rate = 0
        x, y, z = relative_x, relative_y, relative_z
        self.connection.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                time_boot_ms, self.connection.target_system,
                # self.connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                self.connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
                type_mask,
                x, y, z,
                velocity_x, velocity_y, velocity_z,
                axel_x, axel_y, axel_z,
                yaw, yaw_rate
            )
        )

    def move_test(self, rel_x, rel_y, rel_z):
        time_boot_ms = 10  # ms
        type_mask = int(0b010111111000)
        # x, y, z = 0, 0, -1
        velocity_x, velocity_y, velocity_z = 0, 0, 0
        axel_x, axel_y, axel_z = 0, 0, 0
        yaw = 0  # radians
        yaw_rate = 0
        self.connection.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                time_boot_ms, self.connection.target_system,
                self.connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, # seems to be working
                # self.connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # best
                # self.connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
                type_mask,
                rel_x, rel_y, rel_z,
                velocity_x, velocity_y, velocity_z,
                axel_x, axel_y, axel_z,
                yaw, yaw_rate
            )
        )

    def to_target(self):
        pass
        self.move(2,0,0)
        # self.move(velocity_x=30, type='Use Velocity')

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


if __name__ == '__main__':
    wait_time = 5
    box_size = 2
    altitude = 1
    # mavproxy.py --master=/dev/ttyACM0 --out=udpout:0.0.0.0:14550
    # connection_string = 'udpin:127.0.0.1:14540' # working

    # connection_string = 'udpin:127.0.0.1:14550'
    connection_string = 'tcp:localhost:5763'
    print(f'Trying to connect: {connection_string}')
    drone = MavlinkDrone(connection_string)
    # drone = MavlinkDrone('udpout:localhost:14550')
    print('Taking off')
    drone.takeoff(altitude)
    sleep(wait_time)
    print(drone.get_message_local_position_ned())
    print('yaw left')
    drone.yaw(-30)
    print(drone.get_message_local_position_ned())
    sleep(wait_time)

    drone.yaw(60)
    print('yaw right')
    print(drone.get_message_local_position_ned())
    sleep(wait_time)

    exit(0)

    print(f'Forward')
    drone.move_test(box_size, 0, -altitude)
    print(drone.get_message_local_position_ned())
    sleep(wait_time)

    print(drone.get_message_local_position_ned())
    print(f'Right')
    drone.move_test(0, box_size, -altitude)
    print(drone.get_message_local_position_ned())
    sleep(wait_time)

    print(drone.get_message_local_position_ned())
    print(f'Back')
    drone.move_test(-box_size, 0, -altitude)
    sleep(wait_time)

    print(drone.get_message_local_position_ned())
    print(f'Left')
    drone.move_test(0, -box_size, -altitude)
    sleep(wait_time)

    print(drone.get_message_local_position_ned())
    sleep(wait_time)

    print(drone.get_message_local_position_ned())
    print('Landing command')
    drone.land_now()
    exit(0)

    drone.move(box_size, 0, -1)
    print(drone.get_message_local_position_ned())
    sleep(wait_time)
    drone.move(0, box_size, -1)
    print(drone.get_message_local_position_ned())
    sleep(wait_time)
    drone.move(-box_size, 0, -1)
    print(drone.get_message_local_position_ned())
    sleep(wait_time)
    drone.move(0, -box_size, -1)
    print(drone.get_message_local_position_ned())
    sleep(wait_time)
    print(drone.get_message_local_position_ned())
    drone.land_now()
    while True:
        print(drone.get_message_local_position_ned())
