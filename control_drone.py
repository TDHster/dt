from pymavlink import mavutil

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


class MavlinkControl:
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
        self.master = mavutil.mavlink_connection(connection_string)
        print(f'Connected to {connection_string} {self.master}')
        # print(f'Heartbeat: {self.master.wait_heartbeat()}')

    def arm(self):
        # Arm
        # master.arducopter_arm() or:
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )

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
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 0, 0, 0, 0, 0, 0)
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
            500, # 500 means neutral throttle
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


    dron_control = MavlinkControl('udpout:127.0.0.1:14550')
    dron_control.arm()

    from math import sin
    while True:
        try:
            # dron_control.yaw = sin(time_ns()/10)
            dron_control.throttle = sin(time_ns()/10)
            print(f' {sin(time_ns())}:0.2f')
            sleep(1/30)
        except KeyboardInterrupt:
            dron_control.disarm()
            break

