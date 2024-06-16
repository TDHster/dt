from pymavlink import mavutil
# from pymavlink_iq_utilites import *
from time import sleep
import threading
import queue
from bcolors import bcolors
from config import connection_string
from math import pi
def normalize_value(value: float, min_norm=-1000, max_norm=1000):
    raise Warning('Sholud not be used in this module')
    """
    Normalizes a value between -1 and 1 to a specified range.

    Args:
        value (float): The input value between -1 and 1.
        min_norm (float): The minimum value of the normalized range.
        max_norm (float): The maximum value of the normalized range.

    Returns:
        float: The normalized value within the specified range.
    """
    if not (-1 <= value <= 1):
        # raise ValueError("Input value must be between -1 and 1")
        print("normalize_value: Input value must be between -1 and 1")
    if value >= 1:
        value = 1
    if value <= -1:
        value = -1
    # Normalize to range 0-1
    normalized_value = (value + 1) / 2
    # Scale to desired range
    return int(min_norm + normalized_value * (max_norm - min_norm))


def normalize_PWM_range(value: float):
    """Normalizes a value from -1 to 1 to a servo control range of 1100 to 1900.

    Args:
        value (float): The input value between -1 and 1.

    Returns:
        int: The normalized servo control value between 1100 and 1900.
    """

    if value < -1:
        value = -1
    elif value > 1:
        value = 1

    # Normalize to range 0 to 2
    normalized_value = (value + 1) / 2

    # Scale to servo range 1100 to 1900
    servo_value = 1100 + normalized_value * 800

    return int(servo_value)


class MavlinkDrone:
    _pitch, _roll, _yaw, _thrust = 0, 0, 0, 0
    _mode = ''
    CHANNEL_PITCH = 1
    CHANNEL_ROLL = 2
    CHANNEL_THROTTLE = 3
    CHANNEL_YAW = 4

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

        print(f"Using mavlink connection string: {connection_string}")
        self.connection = mavutil.mavlink_connection(connection_string)
        self.connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" %
              (self.connection.target_system, self.connection.target_component))
        # self.autopilot_info = get_autopilot_info(self.connection, self.connection.target_system)
        # print(f'Connected to {self.autopilot_info["autopilot"]} autopilot')

        # Create a shared queue to pass attitude commands
        # self.attitude_command_queue = queue.Queue()

        # Create and start the attitude control thread
        # self.attitude_control_thread = AttitudeControlThread(self.attitude_command_queue, self.connection)
        # self.attitude_control_thread.start()
        # print('Attitude command thread started.')

        self.attitude_command_queue = queue.Queue()
        self.attitude_control_thread = AttitudePWMControlThread(self.attitude_command_queue, self.connection)
        self.attitude_control_thread.start()
        print('Attitude command thread started.')

        self._moving_to_target = False

    def wait_heartbeat(self):
        self.connection.wait_heartbeat()

    def _set_mode(self, mode='LAND'):
        '''
        Modes: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'LOITER', 'RTL', 'CIRCLE', 'POSITION', 'LAND',
                'OF_LOITER', 'DRIFT', 'SPORT', 'FLIP', 'AUTOTUNE', 'POSHOLD', 'BRAKE', 'THROW', 'AVOID_ADSB',
                'GUIDED_NOGPS', 'SMART_RTL', 'FLOWHOLD', 'FOLLOW', 'ZIGZAG', 'SYSTEMID', 'AUTOROTATE', 'AUTO_RTL']
                https://ardupilot.org/copter/docs/flight-modes.html#flight-modes
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

    def mode_land(self):
        """
        Reduces altitude to ground level, attempts to go straight down
        """
        self._set_mode('LAND')

    def mode_return_to_land(self):
        """
        Returns above takeoff location, landing
        """
        # self._set_mode('RTL')
        self._set_mode('SMART_RTL')

    def mode_alt_hold(self):
        """
        Holds altitude and self-levels the roll & pitch
        """
        self._mode = 'ALT_HOLD'  # need for correct thrust
        self._set_mode('ALT_HOLD')

    def mode_guided(self):
        """
        Navigates to single points commanded by GCS
        """
        self._set_mode('GUIDED')

    def mode_stabilize(self):
        """
        Self-levels the roll and pitch axis
        """
        self._set_mode('STABILIZE')

    def mode_brake(self):
        """
        Brings copter to an immediate stop (don't work at test)
        """
        self._set_mode('BRAKE')

    def mode_position_hold(self):
        """
        Like loiter, but manual roll and pitch when sticks not centered
        """
        self._set_mode('POSHOLD')

    def mode_auto(self):
        self._set_mode('AUTO')

    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        """ Set RC channel pwm value
        Args:
            channel_id (TYPE): Channel ID
            pwm (int, optional): Channel pwm value 1100-1900, 1500 neutral
        """
        if channel_id < 1 or channel_id > 18:
            print("Channel does not exist.")
            return

        # Mavlink 2 supports up to 18 channels:
        # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel_id - 1] = pwm
        self.connection.mav.rc_channels_override_send(
            self.connection.target_system,
            self.connection.target_component,
            *rc_channel_values)
        # print(f'{bcolors.WARNING}\tSended PWM:\t{channel_id=}\t{pwm=} {bcolors.ENDC}')

    def _arm(self, arm: bool = True, force=0):
        self.connection.wait_heartbeat()
        if arm == True:
            arm_command = 1
        else:
            arm_command = 0
        # if force == True:
        #     print(f'Arming force.')
        #     force = 21196  # float; 0: arm - disarm; unless; prevented; by; safety; checks, 21196: force; arming or disarming
        # else:
        #     force = 0
        self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component,
                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                              0,
                                              arm_command,
                                              force, 0, 0, 0, 0, 0)
        arm_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if arm_msg:
            if arm_msg.result == 0:
                print(f"{bcolors.OKGREEN}Arm command ok.{bcolors.ENDC} {force=}")
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
        self._arm(arm=True)
        print('Usual arm')

    def arm_2989(self):
        self._arm(arm=True, force=2989)
        print('Force arm, code 2989')

    def arm_21196(self):
        self._arm(arm=True, force=21196)

    def disarm(self):
        self._arm(arm=False)

    def disarm_2989(self):
        self._arm(arm=False, force=2989)
        print('Force disarm, code 2989')

    def disarm_21196(self):
        self._arm(arm=False, force=21196)
        print('Force disarm, code 21196')

    def emergency_stop(self):
        self.disarm_2989()
        self.disarm_21196()
        self.mode_brake()
        self.mode_land()
        # self.disarm()

    def set_yaw_cmd_cond_yaw(self, yaw: float, yaw_rate: float = 15, abs_rel_flag: int = 1,):
        """Set yaw of MAVLink client.

        Args:
            yaw: The yaw angle to set.
            yaw_rate: The yaw rate to set.
            direction: The direction to set. -1 for left, 1 for right.
            abs_rel_flag: The absolute/relative flag to set. 0 for absolute, 1 for relative.
        """
        if yaw >= 0:
            direction = 1
        else:
            direction = -1
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            yaw,  # target angle [0-360]. Absolute angles: 0 is north. Relative angle: 0 is initial yaw. Direction set by param3.
            yaw_rate,  # angular speed
            direction,  # direction: -1: counter clockwise, 0: shortest direction, 1: clockwise
            abs_rel_flag,  # 0: absolute angle, 1: relative offset
            0, 0, 0
        )
        set_yaw_ack = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Set Yaw ACK:  {set_yaw_ack}")
        # return set_yaw_ack.result

    def make_set_position(self, dx=0, dy=0, dz=0):
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

    @property
    def pitch(self):
        return self._pitch

    @pitch.setter
    def pitch(self, pitch: float):
        self._pitch = pitch
        normalized_pitch = normalize_PWM_range(pitch)
        # self.set_rc_channel_pwm(self.CHANNEL_PITCH, normalized_pitch)
        self.attitude_command_queue.put({'pitch': normalized_pitch})

    @property
    def roll(self):
        return self._roll

    @roll.setter
    def roll(self, roll: float):
        self._roll = roll
        norm_roll = normalize_PWM_range(roll)
        # self.set_rc_channel_pwm(self.CHANNEL_ROLL, norm_roll)
        self.attitude_command_queue.put({'roll': norm_roll})

    @property
    def yaw(self):
        return self._yaw

    @yaw.setter
    def yaw(self, yaw_angle: float):
        self._yaw = yaw_angle
        norm_yaw = normalize_PWM_range(yaw_angle)
        # self.set_rc_channel_pwm(self.CHANNEL_YAW, norm_yaw)
        # self.set_yaw_mavlink(yaw_angle)
        self.attitude_command_queue.put({'yaw': norm_yaw})

    @property
    def thrust(self):
        return self._thrust

    @thrust.setter
    def thrust(self, thrust: float):
        self._thrust = thrust
        if self._mode == 'ALT_HOLD':
            # Outside of the mid-throttle deadzone (i.e. below 40% or above 60%) the vehicle will descend or climb
            # depending upon the deflection of the stick.
            thrust = thrust * 0.9
            if thrust > 0:
                thrust += 0.1
            elif thrust < 0:
                thrust -= 0.1
        thrust_normalized = normalize_PWM_range(thrust)
        # print(f'{bcolors.OKBLUE}Set thrust: {thrust=}\t{thrust_normalized}{bcolors.ENDC}')
        # self.set_rc_channel_pwm(self.CHANNEL_THROTTLE, thrust_normalized)
        self.attitude_command_queue.put({'thrust': thrust_normalized})

    @property
    def moving_to_target(self):
        return self._moving_to_target

    def do_hover(self):
        self._thrust = 0
        self._yaw = 0
        self._roll = 0
        self._pitch = 0

    def _manual_thrust_series(self, thrust_pairs):
        """

        Args:
            thrust_pairs: ((thrust, time),(thrust,time),(etc...)) (thurst: float 0..1, time_in_sec)

        Returns:

        """
        for thrust, duration in thrust_pairs:
            # print(f'_manual_thrust_series: {thrust}\t{duration}')
            self.thrust = thrust
            sleep(duration)

    def _takeoff_cmd_nav_takeoff(self, takeoff_altitude):
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

    def _takeoff_manual(self, thrust_pairs=((0.7, 3.5), (0.1, 0.1))):
        # self.mode_guided()
        # self.mode_position_hold() # working, go to near ground.
        self.mode_alt_hold()
        # self.mode_stabilize()
        sleep(0.2)
        self.arm()
        sleep(2)
        self._manual_thrust_series(thrust_pairs)
        self.mode_position_hold()

    def takeoff(self, takeoff_altitude):
        # self.mode_alt_hold()
        # self._set_mode('GUIDED_NOGPS')
        # self._set_mode('AUTO')
        # self.mode_guided()
        # self.mode_position_hold()
        # self.arm()
        # Command Takeoff
        self.mode_guided()
        if not self.arm():
            if not self.arm_2989():
                self.arm_21196()
        self._takeoff_cmd_nav_takeoff(takeoff_altitude)
        sleep(3)
        self.mode_position_hold()

    def land(self, ):
        self.mode_land()
        # self.mode_return_to_land()

        # self._manual_thrust_series(thrust_pairs=((-0.1, 1), (-0.2, 0.5), (-1, 0)))
        # self.disarm()

    def to_target(self, safety=True):
        self._to_target_mode = True
        self.pitch = 1
        if safety:
            sleep(1)  # for safety
            self.pitch = -1  # for safety
            sleep(1)  # for safety
            self.pitch = 0  # for safety

    def send_empty(self):
        self.attitude_command_queue.put({'send empty': 1})



class AttitudePWMControlThread(threading.Thread):
    CHANNEL_PITCH = 1
    CHANNEL_ROLL = 2
    CHANNEL_THROTTLE = 3
    CHANNEL_YAW = 4

    def __init__(self, queue, connection, delay=1/10):
        super().__init__()
        self.queue = queue
        self.connection = connection
        self.delay = delay

        # Initialize initial values
        self.pitch = normalize_PWM_range(0)
        self.roll = normalize_PWM_range(0)
        self.thrust = normalize_PWM_range(0)
        self.yaw = normalize_PWM_range(0)
        self.send_empty = 0

    def run(self):
        while True:
            # Check for new attitude command in the queue
            if not self.queue.empty():
                command = self.queue.get()
                # print(f'{command=}')

                # Update attitude values from the command
                self.thrust = command.get('thrust', self.thrust)
                self.pitch = command.get('pitch', self.pitch)
                self.roll = command.get('roll', self.roll)
                self.yaw = command.get('yaw', self.yaw)
                self.send_empty = command.get('send empty', self.yaw)
                # print(f'{bcolors.OKBLUE}Thread:{bcolors.ENDC} {self.thrust=}\t{self.pitch=}\t{self.roll=}\t{self.yaw=}')

            rc_channel_values = [65535 for _ in range(18)]
            if self.send_empty == 0:
                # Send the current attitude command to the drone
                rc_channel_values[self.CHANNEL_THROTTLE - 1] = self.thrust
                rc_channel_values[self.CHANNEL_PITCH - 1] = self.pitch
                rc_channel_values[self.CHANNEL_ROLL - 1] = self.roll
                rc_channel_values[self.CHANNEL_YAW - 1] = self.yaw

            self.connection.mav.rc_channels_override_send(
                self.connection.target_system, self.connection.target_component,
                *rc_channel_values)

            sleep(self.delay)

    def stop(self):
        self.queue.put({'thrust': 0})  # 500 neutral
        self.queue.put({'pitch': 0})
        self.queue.put({'roll': 0})
        self.queue.put({'yaw': 0})
        # Wait for the thread to finish (if needed)
        self.join(timeout=3)


if __name__ == "__main__":
    pass

