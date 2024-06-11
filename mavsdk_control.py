from pymavlink import mavutil
from pymavlink_iq_utilites import *
import bcolors

import asyncio
from mavsdk import System

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

    def __init__(self, connection_string='udp://127.0.0.1:14550'):
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
        self.drone = System()

        # await drone.connect(system_address=connection_string)

        self.attitude_command_queue = queue.Queue()
        self.attitude_control_thread = AttitudeMavSDKControlThread(self.attitude_command_queue, self.drone)

    async def get_connection_status(self, connection_string):
        await self.drone.connect(system_address=connection_string)

        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"-- {bcolors.OKGREEN}Connected{bcolors.ENDC} to drone!")
                break

        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print(f"-- {bcolors.OKGREEN}GPS{bcolors.ENDC} is good enough for flying.")
                break
    def start_control_thread(self):
        self.attitude_control_thread.start()
        print('Attitude command thread started.')

    # def wait_heartbeat(self):
    #     self.connection.wait_heartbeat()

    async def arm(self):
        print("-- Arming")
        await self.drone.action.arm()
        return True

    async def arm_force(self):
        print("-- Arming force")
        await self.drone.action.arm_force()
        return True

    async def get_takeoff_altitude(self):
        print("-- Arming force")
        await self.drone.action.get_takeoff_altitude()
        return True

    async def set_takeoff_altitude(self, altitude=1):
        print("-- Arming force")
        await self.drone.action.set_takeoff_altitude(altitude)
        return True

    async def disarm(self):
        print('Disarming')
        await self.drone.action.disarm()
        return True

    @property
    def pitch(self):
        return self._pitch

    @pitch.setter
    def pitch(self, pitch: float):
        self._pitch = pitch
        self.attitude_command_queue.put({'pitch': pitch})

    @property
    def roll(self):
        return self._roll

    @roll.setter
    def roll(self, roll: float):
        self._roll = roll
        self.attitude_command_queue.put({'roll': roll})

    @property
    def yaw(self):
        return self._yaw

    @yaw.setter
    def yaw(self, yaw: float):
        self._yaw = yaw
        self.attitude_command_queue.put({'yaw': yaw})

    @property
    def thrust(self):
        return self._thrust

    @thrust.setter
    def thrust(self, thrust: float):
        self._thrust = thrust
        # if self._mode == 'ALT_HOLD':
        #     # Outside of the mid-throttle deadzone (i.e. below 40% or above 60%) the vehicle will descend or climb
        #     # depending upon the deflection of the stick.
        #     thrust = thrust * 0.9
        #     if thrust > 0:
        #         thrust += 0.1
        #     elif thrust < 0:
        #         thrust -= 0.1

        # thrust_normalized = (thrust + 1) / 2
        thrust_normalized = thrust
        self.attitude_command_queue.put({'thrust': thrust_normalized})

    async def takeoff(self):
        await self.drone.action.takeoff()

    async def start_position_control(self):
        print("-- start_position_control")
        await self.drone.manual_control.start_position_control()

    async def start_attitude_control(self):
        print("-- start_position_control")
        await self.drone.manual_control.start_altitude_control()

    async def land(self):
        await self.drone.action.land()

    async def kill(self):
        await self.drone.action.kill()

    def to_target(self, safety=True):
        self.pitch = 1
        if safety==True:
            sleep(0.5)  # for safety
            self.pitch = -1  # for safety
            sleep(0.2)  # for safety
            self.pitch = 0  # for safety


class AttitudeMavSDKControlThread(threading.Thread):

    def __init__(self, queue, drone, delay=1/10):
        super().__init__()
        self.queue = queue
        self.drone = drone
        self.delay = delay

        # Initialize initial values
        self.pitch = 0
        self.roll = 0
        self.thrust = 0
        self.yaw = 0

    def run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        # Ensure loop is closed even if exceptions occur
        try:
            loop.run_until_complete(self._async_run())
        finally:
            loop.close()

    async def _async_run(self):
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
                print(f'{bcolors.OKBLUE}Thread:{bcolors.ENDC} {self.thrust=}\t{self.pitch=}\t{self.roll=}\t{self.yaw=}')
            throttle_norm = (self.thrust + 1) / 2
            await self.drone.manual_control.set_manual_control_input(
                self.pitch, self.roll,
                throttle_norm,
                self.yaw
            )
            await asyncio.sleep(self.delay)


    def stop(self):
        self.queue.put({'thrust': 0})  #
        self.queue.put({'pitch': 0})
        self.queue.put({'roll': 0})
        self.queue.put({'yaw': 0})
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

