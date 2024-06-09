from pymavlink import mavutil
from pymavlink_iq_utilites import *
from time import sleep
import threading
import queue
from bcolors import bcolors

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
        self.autopilot_info = get_autopilot_info(self.connection, self.connection.target_system)
        print(f'Connected to {self.autopilot_info["autopilot"]} autopilot')

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

    def wait_heartbeat(self):
        self.connection.wait_heartbeat()

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

    def _arm(self, arm_command=1):
        self.connection.wait_heartbeat()
        self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component,
                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_command, 0, 0, 0, 0,
                                              0, 0)
        arm_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Arm ACK: {arm_msg}")
        return arm_msg

    def arm(self):
        print('Arming')
        return self._arm(1)

    def disarm(self):
        print('Disarming')
        return self._arm(0)

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
        print(f"Change Mode to {mode} ACK:  {ack_msg}")

    def mode_land(self):
        """
        Reduces altitude to ground level, attempts to go straight down
        """
        self._set_mode('LAND')

    def mode_return_to_land(self):
        """
        Returns above takeoff location, may also include landing
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
        in test: only rotation via RC
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

    def emergency_stop(self):
        self.mode_land()
        self.disarm()
        self.mode_brake()

    def set_yaw_mavlink(self, yaw: float, yaw_rate: float = 15, abs_rel_flag: int = 1,):
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
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, yaw, yaw_rate, direction, abs_rel_flag, 0, 0, 0)
        set_yaw_ack = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Set Yaw ACK:  {set_yaw_ack}")
        # return set_yaw_ack.result

    def make_movement_mavlink(self, rel_x=0, rel_y=0, rel_z=0):

        # from mavlink_iq
        # self.connection.mav.send(
        #     mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        #         10, self.connection.target_system,
        #         self.connection.target_component,
        #         mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        #         int(0b010111111000), 40, 0, -10, 0, 0, 0, 0,
        #         0, 0, 1.57, 0.5))

        type_mask = int(0b010111111000)
        # x, y, z = 0, 0, -1
        time_boot_ms = 10  # just for definition
        velocity_x, velocity_y, velocity_z = 0, 0, 0
        axel_x, axel_y, axel_z = 0, 0, 0
        yaw = 0  # radians
        yaw_rate = 0

        # from mavlink_iq
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

    # self.connection.mav.send(
        #     mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        #         10, self.connection.target_system,
        #         self.connection.target_component,
        #         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        #         int(0b110111111000),
        #         int(-35.3629849 * 10 ** 7),
        #         int(149.1649185 * 10 ** 7), 10, 0, 0, 0, 0,
        #         0, 0, 1.57, 0.5))

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

    def takeoff_manual(self, thrust_pairs=((0.7, 3.5), (0.1, 0.1))):
        # self.mode_guided()
        # self.mode_position_hold() # working, go to near ground.
        self.mode_alt_hold()
        # self.mode_stabilize()

        sleep(0.2)
        self.arm()
        sleep(2)
        self._manual_thrust_series(thrust_pairs)
        self.mode_position_hold()

    def takeoff_via_mavlink(self, takeoff_altitude):
        # self.mode_alt_hold()
        # self._set_mode('GUIDED_NOGPS')
        # self._set_mode('AUTO')
        # self.mode_guided()
        # self.mode_position_hold()
        # self.arm()
        # Command Takeoff
        takeoff_params = [0, 0, 0, 0, 0, 0, takeoff_altitude]

        self.connection.mav.command_long_send(
            self.connection.target_system,  # target_system
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            takeoff_params[0], takeoff_params[1], takeoff_params[2], takeoff_params[3],
            takeoff_params[4], takeoff_params[5], takeoff_params[6])

        takeoff_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Takeoff ACK:  {takeoff_msg}")

    def manual_land(self, thrust_pairs=((-0.1, 1), (-0.2, 0.5), (-1, 0))):
        self.mode_land()
        # self._manual_thrust_series(thrust_pairs)
        # self.disarm()

    def to_target(self, safety=True):
        self.pitch = 1
        if safety==True:
            sleep(0.5)  # for safety
            self.pitch = -1  # for safety
            sleep(0.2)  # for safety
            self.pitch = 0  # for safety

    def move_NED(self, rel_x=0, rel_y=0, rel_z=0, yaw=0):
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
                # self.connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, # seems to be working
                # self.connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # best
                # self.connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
                type_mask,
                rel_x, rel_y, rel_z,
                velocity_x, velocity_y, velocity_z,
                axel_x, axel_y, axel_z,
                yaw, yaw_rate
            )
        )


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
                # print(f'{bcolors.OKBLUE}Thread:{bcolors.ENDC} {self.thrust=}\t{self.pitch=}\t{self.roll=}\t{self.yaw=}')

            # Send the current attitude command to the drone
            rc_channel_values = [65535 for _ in range(18)]
            rc_channel_values[self.CHANNEL_PITCH - 1] = self.pitch
            rc_channel_values[self.CHANNEL_ROLL - 1] = self.roll
            rc_channel_values[self.CHANNEL_THROTTLE - 1] = self.thrust
            rc_channel_values[self.CHANNEL_YAW - 1] = self.yaw
            self.connection.mav.rc_channels_override_send(
                self.connection.target_system, self.connection.target_component,
                *rc_channel_values)

            sleep(self.delay)

    def stop(self):
        self.queue.put({'thrust': 0})  # 500 neutral
        # self.queue.put({'pitch': 0})
        # self.queue.put({'roll': 0})
        # self.queue.put({'yaw': 0})
        # Wait for the thread to finish (if needed)
        self.join(timeout=3)


if __name__ == "__main__":
    # # Replace with your actual connection method
    # connection = mavutil.mavlink.MAVLinkConnection('udp://:14550')
    #
    # # Create a shared queue to pass attitude commands
    # attitude_command_queue = queue.Queue()
    #
    # # Create and start the attitude control thread
    # attitude_control_thread = AttitudeControlThread(attitude_command_queue, connection)
    # attitude_control_thread.start()
    #
    # # Example usage: Send some attitude commands to the queue
    # attitude_command_queue.put({'thrust': 0.5})  # Set thrust to 50%
    # attitude_command_queue.put({'pitch': -0.1})  # Tilt nose down slightly
    # attitude_command_queue.put({'yaw': 0.1})  # Yaw right slightly
    pass

