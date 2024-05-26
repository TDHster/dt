from pymavlink_iq import mavutil
# import queue
# import threading
from time import sleep, time, time_ns
from sys import exit



def normalize_value(value: float, min_norm=-1000, max_norm=1000):
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
        raise ValueError("Input value must be between -1 and 1")
    # Normalize to range 0-1
    normalized_value = (value + 1) / 2
    # Scale to desired range
    return int(min_norm + normalized_value * (max_norm - min_norm))


class MavlinkJoystickControl:
    PITCH_CHANNEL_ID = 1
    ROLL_CHANNEL_ID = 2
    THROTTLE_CHANNEL_ID = 3
    YAW_CHANNEL_ID = 4
    FORWARD_CHANNEL_ID = 5
    _pitch, _roll, _yaw, _throttle = 0, 0, 0, 0  # for be defined

    def __init__(self, connection_string='udpout:127.0.0.1:14550', yaw_pid=1, throttle_pid=1, pitch_pid=1, roll_pid=1):
        """
        udpin:0.0.0.0:14550 or udpout:127.0.0.1:14550
        Linux computer connected to the vehicle via USB	/dev/ttyUSB0
        Linux computer connected to the vehicle via serial port (RaspberryPi example)	/dev/ttyAMA0 (also set baud=57600)
        MAVLink API listening for SITL connection via UDP	udpin:localhost:14540 (or udp:localhost:14540, 127.0.0.1:14540,etc.)
        MAVLink API initiating a connection to SITL via UDP	udpout:localhost:14540 (or udpout:127.0.0.1:14540)
        GCS connected to the vehicle via UDP	127.0.0.1:14550 or udp:localhost:14550
        SITL connected to the vehicle via TCP	tcp:127.0.0.1:5760 (ArduPilot only, PX4 does not support TCP)
        OSX computer connected to the vehicle via USB	dev/cu.usbmodem1
        Windows computer connected to the vehicle via USB (in this case on COM14)	com14
        Windows computer connected to the vehicle using a 3DR Telemetry Radio on COM14	com14 (also set baud=57600)
        """
        print('Trying make mavlink connection...')
        self._master = mavutil.mavlink_connection(connection_string)
        print(f'Connected to {connection_string} {self._master}')
        # print(f'Heartbeat: {self.master.wait_heartbeat()}')

        # self.control_commands = queue.Queue()

        self.yaw_pid = yaw_pid
        self.roll_pid = roll_pid
        self.pitch_pid = pitch_pid
        self.throttle_pid = throttle_pid
        self.PWM_neutral = 1500
        self.PWN_min = 1100
        self.PWM_max = 1900

    # def arm(self):
    #     try:
    #         # worked:
    #         # self.master.mav.command_long_send(
    #         #     self.master.target_system,
    #         #     self.master.target_component,
    #         #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0 )
    #         self.set_mode(mode)
    #
    #         self._master.arducopter_arm()
    #         # wait until arming confirmed (can manually check with self.master.motors_armed())
    #         print("Waiting for the vehicle to arm")
    #         self._master.motors_armed_wait()
    #         print('Armed!')
    #         print(f'Arm status check: {self._master.motors_armed()}')
    #         return True
    #     except Exception as e:
    #         print(f'Problem with arming: {e}')
    #         return False

    def arm(self, arm_command=1):
        '''
        Args:
            arm_command: 1 - arm
            0 - disarm

        Returns:
        '''
        # Wait for the first heartbeat
        # This sets the system and component ID of remote system for the link
        self._master.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" %
              (self._master.target_system, self._master.target_component))

        self._master.mav.command_long_send(self._master.target_system, self._master.target_component,
                                           mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_command, 0, 0, 0, 0,
                                           0, 0)

        msg = self._master.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

        # return the result of the ACK message
        return msg.result

    def disarm(self):
        try:
            # not worked
            # self.master.mav.command_long_send(
            #     self.master.target_system,
            #     self.master.target_component,
            #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            #     0,
            #     0, 0, 0, 0, 0, 0, 0)
            self._master.arducopter_disarm()
            print('Sent disarm command!')
            # wait until disarming confirmed
            print(f'Disarm wait: {self._master.motors_disarmed_wait()}')
            return True
        except Exception as e:
            print(f'Problem with disarming: {e}')
            return False

    def _set_rc_channel_pwm(self, channel_id, pwm=1500):
        # Create a function to send RC values
        # More information about Joystick channels
        # here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
        # 1500 neutral
        """ Set RC channel pwm value
        Args:
            channel_id (TYPE): Channel ID
            pwm (int, optional): Channel pwm value 1100-1900

        # Channel Meaning
        # 1        Pitch
        # 2        Roll
        # 3        Throttle
        # 4        Yaw
        # 5        Forward
        """
        if channel_id < 1 or channel_id > 18:
            print("Channel does not exist.")
            return

        # Mavlink 2 supports up to 18 channels:
        # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel_id - 1] = pwm
        print(f'Debug _set_rc_channel_pwm:\r\t {self._master.target_system=} {self._master.target_component=} {rc_channel_values=}')
        self._master.mav.rc_channels_override_send(
            self._master.target_system,  # target_system
            self._master.target_component,  # target_component
            # *rc_channel_values)  # RC channel list, in microseconds.
            *rc_channel_values[:12])  # RC channel list, in microseconds.

    def _set_rc_4_channels_pwm(self):
        # Create a function to send RC values
        # More information about Joystick channels
        # here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
        # 1500 neutral
        """ Set RC channel pwm value
        Args:
            channel_id (TYPE): Channel ID
            pwm (int, optional): Channel pwm value 1100-1900

        # Channel Meaning
        # 1        Pitch
        # 2        Roll
        # 3        Throttle
        # 4        Yaw
        # 5        Forward
        """
        # if channel_id < 1 or channel_id > 18:
        #     print("Channel does not exist.")
        #     return

        # Mavlink 2 supports up to 18 channels:
        # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[self.THROTTLE_CHANNEL_ID - 1] = normalize_value(self._throttle, min_norm=self.PWN_min, max_norm=self.PWM_max)
        rc_channel_values[self.YAW_CHANNEL_ID - 1] = normalize_value(self._yaw , min_norm=self.PWN_min, max_norm=self.PWM_max)
        rc_channel_values[self.PITCH_CHANNEL_ID - 1] = normalize_value(self._pitch , min_norm=self.PWN_min, max_norm=self.PWM_max)
        rc_channel_values[self.ROLL_CHANNEL_ID - 1] = normalize_value(self._roll , min_norm=self.PWN_min, max_norm=self.PWM_max)

        # print(f'Debug _set_rc_channel_pwm:\r\t {self._master.target_system=} {self._master.target_component=} {rc_channel_values=}')
        self._master.mav.rc_channels_override_send(
            self._master.target_system,  # target_system
            self._master.target_component,  # target_component
            # *rc_channel_values)  # RC channel list, in microseconds.
            *rc_channel_values)  # RC channel list, in microseconds.


    @property
    def yaw(self):
        return self._yaw

    @yaw.setter
    def yaw(self, yaw: float):
        self._yaw = yaw * self.yaw_pid
        # self._set_rc_channel_pwm(self.YAW_CHANNEL_ID,
        #                          normalize_value(yaw, min_norm=self.PWN_min, max_norm=self.PWM_max))

        self._set_rc_4_channels_pwm()

    @property
    def throttle(self):
        return self._throttle

    @throttle.setter
    def throttle(self, throttle: float):
        self._throttle = throttle * self.throttle_pid
        # self._set_rc_channel_pwm(self.THROTTLE_CHANNEL_ID,
        #                          normalize_value(throttle, min_norm=self.PWN_min, max_norm=self.PWM_max))
        self._set_rc_4_channels_pwm()

    def to_target(self):
        self._pitch = 1  # maximum
        # self._set_rc_channel_pwm(self.PITCH_CHANNEL_ID,
        #                          normalize_value(self._pitch, min_norm=self.PWN_min, max_norm=self.PWM_max))
        self._set_rc_4_channels_pwm()

    def get_modes_list(self):
        return list(self._master.mode_mapping().keys())

    def set_mode(self, mode='LAND'):
        '''
        Modes: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'LOITER', 'RTL', 'CIRCLE', 'POSITION', 'LAND',
                'OF_LOITER', 'DRIFT', 'SPORT', 'FLIP', 'AUTOTUNE', 'POSHOLD', 'BRAKE', 'THROW', 'AVOID_ADSB',
                'GUIDED_NOGPS', 'SMART_RTL', 'FLOWHOLD', 'FOLLOW', 'ZIGZAG', 'SYSTEMID', 'AUTOROTATE', 'AUTO_RTL']
        '''
        self._master.set_mode(mode)


class MavlinkManualControl:
    _pitch, _roll, _yaw, _throttle = 0, 0, 0, 0  # for be defined
    _buttons = 0
    THROTTLE_NEUTRAL = 500
    THROTTLE_NEUTRAL_FLOAT = 0.5
    MAXIMUM_PITCH = 1

    def __init__(self, connection_string='udpout:127.0.0.1:14550'):
        """
        udpin:0.0.0.0:14550 or udpout:127.0.0.1:14550
        Linux computer connected to the vehicle via USB	/dev/ttyUSB0
        Linux computer connected to the vehicle via serial port (RaspberryPi example)	/dev/ttyAMA0 (also set baud=57600)
        MAVLink API listening for SITL connection via UDP	udpin:localhost:14540 (or udp:localhost:14540, 127.0.0.1:14540,etc.)
        MAVLink API initiating a connection to SITL via UDP	udpout:localhost:14540 (or udpout:127.0.0.1:14540)
        GCS connected to the vehicle via UDP	127.0.0.1:14550 or udp:localhost:14550
        SITL connected to the vehicle via TCP	tcp:127.0.0.1:5760 (ArduPilot only, PX4 does not support TCP)
        OSX computer connected to the vehicle via USB	dev/cu.usbmodem1
        Windows computer connected to the vehicle via USB (in this case on COM14)	com14
        Windows computer connected to the vehicle using a 3DR Telemetry Radio on COM14	com14 (also set baud=57600)
        """
        # Create the connection
        print('Trying make mavlink connection...')
        self.master = mavutil.mavlink_connection(connection_string)
        print(f'Connected to {connection_string} {self.master}')
        # print(f'Heartbeat: {self.master.wait_heartbeat()}')

    def arm(self):
        # Arm
        try:
            # worked:
            # self.master.mav.command_long_send(
            #     self.master.target_system,
            #     self.master.target_component,
            #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            #     0, 1, 0, 0, 0, 0, 0, 0
            # )
            self.master.arducopter_arm()
            # wait until arming confirmed (can manually check with self.master.motors_armed())
            print("Waiting for the vehicle to arm")
            self.master.motors_armed_wait()
            print('Armed!')
            print(f'Arm check: {self.master.motors_armed()}')
        except Exception as e:
            print(f'Problem with arm: {e}')

    def disarm(self):
        # Disarm
        # master.arducopter_disarm() or:
        try:
            # not worked
            # self.master.mav.command_long_send(
            #     self.master.target_system,
            #     self.master.target_component,
            #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            #     0,
            #     0, 0, 0, 0, 0, 0, 0)
            self.master.arducopter_disarm()
            print('Sent disarm command!')
            # wait until disarming confirmed
            print(f'Disarm wait: {self.master.motors_disarmed_wait()}')
        except Exception as e:
            print(f'Problem with disarm: {e}')

    def _make_movement(self):
        """
        Send a positive x value, negative y, negative z,
        positive rotation and no button.
        https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
        Warning: Because of some legacy workaround, z will work between [0-1000]
        where 0 is full reverse, 500 is no output and 1000 is full throttle.
        x,y and r will be between [-1000 and 1000].
        x,y,z,r | pitch, roll, thottle, yaw
        x : X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle. (type:int16_t)
        y : Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle. (type:int16_t)
        z : Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000
            on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative
            thrust. (type:int16_t)
        r : R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid.
            Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle. (type:int16_t)
        buttons : A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released.
            The lowest bit corresponds to Button 1. (type:uint16_t)

        """

        print(f'Send command: pitch={self._pitch}, roll={self._roll}, yaw={self._yaw}, throttle={self._throttle}')
        self.master.mav.manual_control_send(
            self.master.target_system,
            normalize_value(self._pitch),
            normalize_value(self._roll),
            normalize_value(self._throttle, min_norm=0, max_norm=1000),
            normalize_value(self._yaw),
            self._buttons
        )

    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        # Create a function to send RC values
        # More information about Joystick channels
        # here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
        """ Set RC channel pwm value
        Args:
            channel_id (TYPE): Channel ID
            pwm (int, optional): Channel pwm value 1100-1900

        # Channel Meaning
        # 1        Pitch
        # 2        Roll
        # 3        Throttle
        # 4        Yaw
        # 5        Forward
        """
        if channel_id < 1 or channel_id > 18:
            print("Channel does not exist.")
            return

        # Mavlink 2 supports up to 18 channels:
        # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel_id - 1] = pwm
        self.master.mav.rc_channels_override_send(
            self.master.target_system,  # target_system
            self.master.target_component,  # target_component
            *rc_channel_values)  # RC channel list, in microseconds.

    @property
    def yaw(self):
        return self._yaw

    @yaw.setter
    def yaw(self, value: float):
        """
        :param value: -1,1 float
        :return: None
        """
        self._yaw = value
        self._make_movement()
        self._yaw = 0


    @property
    def pitch(self):
        return self._pitch

    @pitch.setter
    def pitch(self, value: float):
        """
        :param value: -1..1 float
        :return:
        """
        self._pitch = value
        self._make_movement()
        self._pitch = 0


    def to_target(self):
        self._pitch = self.MAXIMUM_PITCH
        self._make_movement()

    @property
    def throttle_yaw(self):
        return self._throttle, self._yaw

    @throttle_yaw.setter
    def throttle_yaw(self, throttle_and_yaw):
        _throttle, _yaw = throttle_and_yaw
        self._make_movement()
        _throttle, _yaw = self.THROTTLE_NEUTRAL_FLOAT, 0

    @property
    def throttle(self):
        return self._throttle

    @throttle.setter
    def throttle(self, value: float):
        """
        :param value: -1..1 float
        :return:
        """
        self._throttle = value
        self._make_movement()
        self._throttle = 0

    def ___control_with_buttons(self):
        # To active button 0 (first button), 3 (fourth button) and 7 (eighth button)
        # It's possible to check and configure this buttons in the Joystick menu of QGC
        buttons = 1 + 1 << 3 + 1 << 7
        self.master.mav.manual_control_send(
            self.master.target_system,
            0,
            0,
            500,  # 500 means neutral throttle
            0,
            buttons)

    def _read_parameters(self):
        # # Create the connection
        # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        # Wait a heartbeat before sending commands
        self.master.wait_heartbeat()

        # Request all parameters
        self.master.mav.param_request_list_send(
            self.master.target_system, self.master.target_component
        )
        while True:
            sleep(0.01)
            try:
                message = self.master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
                print('name: %s\tvalue: %d' % (message['param_id'],
                                               message['param_value']))
            except Exception as error:
                print(error)
                exit(0)


class CameraGimble:
    def __init__(self, pitch=0, roll=0, yaw=0):
        self._pitch = pitch
        self._roll = roll
        self._yaw = yaw

    @property
    def pitch(self):
        return self._pitch

    @pitch.setter
    def pitch(self, value):
        # Validate pitch value (adjust limits as needed)
        if -180 <= value <= 180:
            self._pitch = value
        else:
            raise ValueError("Pitch value must be between -180 and 180 degrees")

    @property
    def roll(self):
        return self._roll

    @roll.setter
    def roll(self, value):
        # Validate roll value (adjust limits as needed)
        if -180 <= value <= 180:
            self._roll = value
        else:
            raise ValueError("Roll value must be between -180 and 180 degrees")

    @property
    def yaw(self):
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        # Validate yaw value (adjust limits as needed)
        if -180 <= value <= 180:
            self._yaw = value
        else:
            raise ValueError("Yaw value must be between -180 and 180 degrees")



class link_RPi_GPIO:
    def __init__(self, GPIO_IN=[33, 34], GPIO_OUT=[36, 37]):
        try:
            import RPi.GPIO as GPIO
            print(f"Running on: {GPIO.RPI_INFO}")
        except RuntimeError:
            print("Error importing RPi.GPIO!  Are your root? Use sudo")

        try:
            #Settig up RPi board
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)

            GPIO.setup(GPIO_IN, GPIO.IN)
            GPIO.setup(GPIO_OUT, GPIO.OUT)
        except Exception as e:
            print(f'RPi pin setup error: {e}')



if __name__ == '__main__':
    # for getting data
    # run: sudo mavproxy.py --master=/dev/ttyACM0 --out=udpout:0.0.0.0:14540
    # dron_control = MavlinkControl('udpin:127.0.0.1:14540')
    # while True:
    #     dron_control._read_parameters()

    connection_string = 'udpout:127.0.0.1:14540'
    print(f'Trying to connect: {connection_string}')
    dron = MavlinkJoystickControl(connection_string)
    print('Modes:', list(dron._master.mode_mapping().keys()))
    '''
    Modes: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'LOITER', 'RTL', 'CIRCLE', 'POSITION', 'LAND',
            'OF_LOITER', 'DRIFT', 'SPORT', 'FLIP', 'AUTOTUNE', 'POSHOLD', 'BRAKE', 'THROW', 'AVOID_ADSB',
            'GUIDED_NOGPS', 'SMART_RTL', 'FLOWHOLD', 'FOLLOW', 'ZIGZAG', 'SYSTEMID', 'AUTOROTATE', 'AUTO_RTL']
    '''
    dron.arm()
    # dron.set_mode('ALT_HOLD')
    # dron.set_mode('STABILIZE')
    dron.set_mode('GUIDED')  # <- this work

    from math import sin

    dron.throttle = 0.7
    sleep(0.5)
    dron.throttle = 0.5

    while True:
        try:
            # dron_control.yaw = sin(time_ns()/10)
            # dron_control.throttle = sin(time_ns()/10)
            # print(f' {sin(time_ns())}:0.2f')
            # sleep(1/30)
            # dron_control.master.flightmode_list() - give error
           # Set some roll
            # set_rc_channel_pwm(2, 1600)
            # Set some yaw
            # dron.set_rc_channel_pwm(4, 1700)
            # dron.throttle = 0.1
            dron.yaw = 0
            # dron_control.throttle = 0
            sleep(2)
            # dron.set_rc_channel_pwm(4, 1200)
            # dron.throttle = 0.5
            dron.yaw = 0.5
            sleep(2)
            # dron.set_rc_channel_pwm(4, 1500)
            # dron.throttle = 1
            dron.yaw = 1
            sleep(2)
            # dron.throttle = 0
            dron.yaw = 0.5
            sleep(2)


        except KeyboardInterrupt:
            dron.disarm()
            dron.set_mode('LAND')
            exit(0)

