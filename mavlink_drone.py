from pymavlink import mavutil
# from pymavlink_iq_utilites import *
from time import sleep, time
import threading
import queue
from bcolors import bcolors
# from config import connection_string  # for sim in mission planner
from math import pi


class MavlinkDrone:
    _to_target = False

    def __init__(self, connection_string):
        print(f"Using mavlink connection string: {connection_string}")
        self.connection = mavutil.mavlink_connection(connection_string)
        self.connection.wait_heartbeat()
        print(f"{bcolors.OKGREEN}Heartbeat{bcolors.ENDC} "
              f"from system (system {self.connection.target_system} component {self.connection.target_component})")

    def _set_mode(self, mode: str = 'LAND'):
        '''
        Modes: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'LOITER', 'RTL', 'CIRCLE', 'POSITION', 'LAND',
                'OF_LOITER', 'DRIFT', 'SPORT', 'FLIP', 'AUTOTUNE', 'POSHOLD', 'BRAKE', 'THROW', 'AVOID_ADSB',
                'GUIDED_NOGPS', 'SMART_RTL', 'FLOWHOLD', 'FOLLOW', 'ZIGZAG', 'SYSTEMID', 'AUTOROTATE', 'AUTO_RTL']
                https://ardupilot.org/copter/docs/flight-modes.html#flight-modes
        '''
        # mode = 'GUIDED'
        # mode = 'POSHOLD'
        self.connection.set_mode(mode)
        ack_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack_msg:
            if ack_msg.result == 0:
                print(f"{bcolors.OKGREEN}{mode} mode{bcolors.ENDC} set OK")
            else:
                print(f"Change Mode to {bcolors.FAIL}{mode}{bcolors.ENDC} FAIL: {ack_msg.result}")
        else:
            print(f'{bcolors.FAIL}Set mode command{bcolors.ENDC} return None.')

    def set_mode_guided(self):
        self._set_mode('GUIDED')

    def set_mode_poshold(self):
        self._set_mode('POSHOLD')

    def set_mode_land(self):
        self._set_mode('LAND')

    def set_mode_return_to_land(self):
        self._set_mode('RTL')

    def set_mode_brake(self):
        self._set_mode('BRAKE')

    def _arm(self, arm: bool = True, force=0):  # work
        # force 21196, 2989
        # https://ardupilot.org/dev/docs/mavlink-arming-and-disarming.html
        # Need "arm uncheck all" in mavproxy
        if arm == True:
            arm_command = 1
        else:
            arm_command = 0
        # if force == True:
        #     # force = 21196  # from docs
        #     force = 2989  # from pymavlink source
        #     print(f'Using force arm. Code {force}')
        # else:
        #     force = 0
        # message COMMAND_LONG 0 0 400 0 1 21196 0 0 0 0 0 force arm
        self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component,
                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                              0,
                                              arm_command,
                                              force, 0, 0, 0, 0, 0)
        arm_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if arm_msg:
            if arm_msg.result == 0:
                print(f"{bcolors.OKGREEN}Arm command ok.{bcolors.ENDC}")
            else:
                print(f"{bcolors.FAIL}Arm ACK: {bcolors.ENDC} {arm_msg.result}")
        else:
            print(f'{bcolors.FAIL}Arm command FAIL (no COMMAND_ACK){bcolors.ENDC}')
            return False
        if arm:
            print("Waiting motors arming...")
            self.connection.motors_armed_wait()
            print(f'{bcolors.OKGREEN}Motors start confirmed.{bcolors.ENDC}')

    def arm(self):
        self._arm(arm=True)
        print('Usual arm')
        self._arm(arm=True, force=2989)
        print('Force arm, code 2989')
        self._arm(arm=True, force=21196)
        print('Force arm, code 21196')


    def disarm(self):
        self._arm(arm=False, force=2989)
        self._arm(arm=False)
        self._arm(arm=False, force=21196)


    def emergency_stop(self):
        self._arm(arm=False, force=True)  # Immediately stop, maybe crash
        self.set_mode_brake()

    def _takeoff(self, takeoff_altitude):
        self.connection.mav.command_long_send(
            self.connection.target_system,  # target_system
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0,
            0, 0, takeoff_altitude)

        takeoff_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if takeoff_msg:
            if takeoff_msg.result == 0:
                print(f"{bcolors.OKGREEN}Takeoff command OK.{bcolors.ENDC}")
            else:
                print(f"{bcolors.FAIL}Takeoff command ACK not ok: {takeoff_msg.result}{bcolors.ENDC}")
        else:
            print(f"{bcolors.FAIL}Takeoff command fail (no COMMAND_ACK){bcolors.ENDC}")

    def takeoff(self, takeoff_altitude):
        self.set_mode_guided()
        self.arm()
        self._takeoff(takeoff_altitude)
        sleep(2)
        self.change_position(dz=1)
        sleep(1)
        self.change_position(dz=1)

    def change_position(self, dx=0, dy=0, dz=0):
        dz = -dz  # original axis down
        yaw = 0
        yaw_rate = 0
        # yaw = 30 / (180 * pi)
        # yaw_rate = 15 / (180 * pi)
        yaw = yaw / (180 * pi)
        yaw_rate = yaw_rate / (180 * pi)
        time_boot_ms = 10  # ms
        """
        Use Yaw :          0b100111111111 / 0x09FF / 2559 (decimal)
        Use Yaw Rate :     0b010111111111 / 0x05FF / 1535 (decimal)
        Use Acceleration : 0b110000111111 / 0x0C3F / 3135 (decimal)
        Use Velocity :     0b110111000111 / 0x0DC7 / 3527 (decimal)
        Use Position :     0b110111111000 / 0x0DF8 / 3576 (decimal)
        Use Pos+Vel :      0b110111000000 / 0x0DC0 / 3520 (decimal)
        Use Pos+Vel+Accel: 0b110000000000 / 0x0C00 / 3072 (decimal)
        """
        type_mask_position = int(0b110111111000)  # Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)
        # type_mask_yaw = int(0b100111111111)  # # Use Yaw: 0b100111111111 / 0x09FF / 2559(decimal)
        # type_mask = type_mask_position
        type_mask = int(0b100111111000)  # works yaw and deltas x,y,z
        if self._to_target == True:
            dx = 3
        velocity_x, velocity_y, velocity_z = 0, 0, 0
        axel_x, axel_y, axel_z = 0, 0, 0
        print(f'Inside change_position: {dx=:.1f}, {dy=:.1f}, {dz=:.1f}')
        self.connection.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                time_boot_ms, self.connection.target_system, self.connection.target_component,
                # In frames,
                # _OFFSET_ means “relative to vehicle position” while
                # _LOCAL_ is “relative to home position”
                # (these have no impact on velocity directions).
                # _BODY_ means that velocity components are relative to the
                #  heading of the vehicle rather than the NED frame.
                # mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                type_mask,
                dx, dy, dz,
                velocity_x, velocity_y, velocity_z,
                axel_x, axel_y, axel_z,
                yaw, yaw_rate
            )
        )
        # cmd_ack = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)

        # if cmd_ack:
        #     if cmd_ack.result == 0:
        #         print(f"{bcolors.OKGREEN}set_position_target OK{bcolors.ENDC}")
        #     else:
        #         print(f"{bcolors.FAIL}set_position_target ACK:  {cmd_ack.result}{bcolors.ENDC}")
        # else:
        #     print(f"{bcolors.FAIL}set_position_target fail{bcolors.ENDC}")

        # cmd_ack = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        # print(cmd_ack)

    def to_target(self, mode: bool = True):
        self._to_target = mode
        self.change_position()

    def to_target_status(self):
        return self._to_target

    def yaw(self, yaw=0, yaw_rate=None, abs_rel_flag=1):

        if not yaw_rate:
            # yaw_rate_PID = 0.75
            # yaw_rate = abs(yaw)
            yaw_rate = 40
            yaw_rate = 10 * yaw / 6
        if yaw >= 0:
            direction = 1
        else:
            direction = -1
            yaw = abs(yaw)
        print(f'Inside conditional yaw: {yaw=:0.1f}, {yaw_rate=:0.1f}')
        # direction = -1  # -1, 1
        # abs_rel_flag = 1  # The absolute/relative flag to set. 0 for absolute from north, 1 for relative.from heading
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
            yaw,
            yaw_rate,
            direction,
            abs_rel_flag, 0, 0, 0)

    def pwm_control(self, pitch, roll, thrust, yaw):
        raise Exception('Need to be sent periodically else land because RC lost')
        # pwm work only yaw
        # pitch, roll = 1100, 1900
        # thrust = 1900
        # yaw = 1500

        CHANNEL_PITCH = 1
        CHANNEL_ROLL = 2
        CHANNEL_THROTTLE = 3
        CHANNEL_YAW = 4

        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[CHANNEL_PITCH - 1] = pitch
        rc_channel_values[CHANNEL_ROLL - 1] = roll
        rc_channel_values[CHANNEL_THROTTLE - 1] = thrust
        rc_channel_values[CHANNEL_YAW - 1] = yaw

        # print("Starting loop")
        # start_time = time()
        # while time() - start_time < 3: # for yaw loop don't needed
        #     connection.mav.rc_channels_override_send(
        #         connection.target_system, connection.target_component,
        #         *rc_channel_values)
        #     sleep(1/10)
        # print("End loop")
        print('Sending PWM')
        connection.mav.rc_channels_override_send(
            connection.target_system, connection.target_component,
            *rc_channel_values)

    # pwm_control(1100, 1900,1500, 1500)
    # while True:True
    #     sleep(1/10)
    #     pwm_control(1500, 1500,1500, 1500)

    # set_mode('GUIDED')

    def manual_control_send(self):
        raise Exception('Need to be sent periodically else land because RC lost')

        # need poshold mode
        # need to send data periodically, else failsafe land

        # Send a positive x value, negative y, negative z,
        x = 200
        y = 0
        z = 500
        yaw = 0
        buttons = 0
        # https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
        # Warning: Because of some legacy workaround, z will work between [0-1000]
        # where 0 is full reverse, 500 is no output and 1000 is full throttle.
        # x,y and r will be between [-1000 and 1000].
        connection.mav.manual_control_send(
            connection.target_system,
            x,
            y,
            z,
            yaw,
            buttons)


if __name__ == '__main__':
    print('Pymavlink test file run.')

    connection_string = 'udpin:127.0.0.1:14550'
    drone = MavlinkDrone(connection_string=connection_string)

    # drone.set_mode_return_to_land()
    drone.set_mode_guided()
    drone.arm()
    sleep(5)
    drone.disarm()
    # drone.takeoff(2)
    # drone._takeoff(2)
    # sleep(10)
    # drone.set_mode_land()

