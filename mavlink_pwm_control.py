from pymavlink import mavutil
from pymavlink_iq_utilites import *
from time import sleep
import threading
from bcolors import bcolors

def normalize_value(value: float, min_norm=-1000, max_norm=1000):
    raise Warning('Sholudn be used in this module')
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

    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        """ Set RC channel pwm value
        Args:
            channel_id (TYPE): Channel ID
            pwm (int, optional): Channel pwm value 1100-1900
        """
        if channel_id < 1 or channel_id > 18:
            print("Channel does not exist.")
            return

        # Mavlink 2 supports up to 18 channels:
        # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel_id - 1] = pwm
        self.connection.mav.rc_channels_override_send(
            self.connection.target_system,  # target_system
            self.connection.target_component,  # target_component
            *rc_channel_values)  # RC channel list, in microseconds.
        # print(f'{bcolors.WARNING}\tSended PWM:\t{channel_id=}\t{pwm=} {bcolors.ENDC}')

    def _arm(self, arm_command=1):
        self.connection.wait_heartbeat()
        self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component,
                                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_command, 0, 0, 0, 0,
                                              0, 0)
        msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)
        return msg

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
        self.connection.set_mode(mode)

    def mode_land(self):
        self._set_mode('LAND')

    def mode_return_to_land(self):
        # self._set_mode('RTL')
        self._set_mode('SMART_RTL')

    def mode_alt_hold(self):
        self._set_mode('ALT_HOLD')

    def mode_guided(self):
        self._set_mode('GUIDED')

    def mode_stabilize(self):
        self._set_mode('STABILIZE')

    def mode_brake(self):
        self._set_mode('BRAKE')

    def emergency_stop(self):
        self.disarm()
        self.mode_land()
        self.mode_brake()

    @property
    def pitch(self):
        return self._pitch

    @pitch.setter
    def pitch(self, pitch: float):
        self._pitch = pitch
        normalized_pitch = normalize_PWM_range(pitch)
        self.set_rc_channel_pwm(self.CHANNEL_PITCH, normalized_pitch)

    @property
    def roll(self):
        return self._roll

    @roll.setter
    def roll(self, roll: float):
        self._roll = roll
        norm_roll = normalize_PWM_range(roll)
        self.set_rc_channel_pwm(self.CHANNEL_ROLL, norm_roll)

    @property
    def yaw(self):
        return self._yaw

    @yaw.setter
    def yaw(self, yaw: float):
        self._yaw = yaw
        norm_yaw = normalize_PWM_range(yaw)
        self.set_rc_channel_pwm(self.CHANNEL_YAW, norm_yaw)

    @property
    def thrust(self):
        return self._thrust

    @thrust.setter
    def thrust(self, thrust: float):
        self._thrust = thrust
        thrust_normalized = normalize_PWM_range(thrust)
        print(f'{bcolors.OKBLUE}Set thrust: {thrust=}\t{thrust_normalized}{bcolors.ENDC}')
        self.set_rc_channel_pwm(self.CHANNEL_THROTTLE, thrust_normalized)

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

    def manual_takeoff(self, thrust_pairs=((0.5, 3), (0, 0.1))):
        self.mode_alt_hold()
        # self.mode_guided()
        # self.mode_stabilize()
        sleep(0.2)
        self.arm()
        sleep(2)
        self._manual_thrust_series(thrust_pairs)

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


class AttitudeControlThread(threading.Thread):
    def __init__(self, queue, connection, delay=1/20):
        super().__init__()
        self.queue = queue
        self.connection = connection
        self.delay = delay

        # Initialize initial values
        self.pitch = 0
        self.roll = 0
        self.thrust = 500
        self.yaw = 0

    def run(self):
        while True:
            # Check for new attitude command in the queue
            if not self.queue.empty():
                command = self.queue.get()
                print(f'{command=}')

                # Update attitude values from the command
                self.thrust = command.get('thrust', self.thrust)
                self.pitch = command.get('pitch', self.pitch)
                self.roll = command.get('roll', self.roll)
                self.yaw = command.get('yaw', self.yaw)
                # print(f'{self.thrust=}\t{self.pitch=}\t{self.roll=}\t{self.yaw=}')

            # Send the current attitude command to the drone
            self.connection.mav.manual_control_send(
                self.connection.target_system,
                self.pitch,
                -self.roll,  # inversed
                self.thrust,
                self.yaw,
                0)

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

