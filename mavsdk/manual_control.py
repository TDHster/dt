#!/usr/bin/env python3

"""
This example shows how to use the manual controls plugin.

Note: Manual inputs are taken from a test set in this example to decrease
complexity. Manual inputs can be received from devices such as a joystick
using third-party python extensions.

Note: Taking off the drone is not necessary before enabling manual inputs.
It is acceptable to send positive throttle input to leave the ground.
Takeoff is used in this example to decrease complexity
"""

import asyncio
import random
from mavsdk import System

# Test set of manual inputs. Format: [roll, pitch, throttle, yaw]
value = 0.5
throttle_value = 0.5
manual_inputs = [
    [0, 0, throttle_value, 0],  # no movement
    [-value, 0, throttle_value, 0],  # minimum roll
    [value, 0, throttle_value, 0],  # maximum roll
    [0, -value, throttle_value, 0],  # minimum pitch
    [0, value, throttle_value, 0],  # maximum pitch
    [0, 0, throttle_value, -value],  # minimum yaw
    [0, 0, throttle_value, value],  # maximum yaw
    [0, 0, throttle_value, 0],  # max throttle
    [0, 0, throttle_value, 0],  # minimum throttle
    # [0, 0, value, 0],  # max throttle
    # [0, 0, 0, 0],  # minimum throttle
]


async def manual_controls():
    """Main function to connect to the drone and input manual controls"""
    # Connect to the Simulation
    drone = System()

    await drone.connect(system_address="udp://127.0.0.1:14550")

    # This waits till a mavlink based drone is connected
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    # Checking if Global Position Estimate is ok
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

    # set the manual control input after arming
    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    # Arming the drone
    print("-- Arming")
    await drone.action.arm()

    await asyncio.sleep(1)
    await drone.action.set_takeoff_altitude(2)
    # Takeoff the vehicle
    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(5)

    # set the manual control input after arming
    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    # start manual control
    print("-- Starting manual control")
    await drone.manual_control.start_position_control()

    status_text_task = asyncio.ensure_future(print_status_text(drone))

    try:
        while True:
            # grabs a random input from the test list
            # WARNING - your simulation vehicle may crash if its unlucky enough
            # input_index = random.randint(0, len(manual_inputs) - 1)
            # input_list = manual_inputs[input_index]

            # # get current state of roll axis (between -1 and 1)
            # roll = float(input_list[0])
            # # get current state of pitch axis (between -1 and 1)
            # pitch = float(input_list[1])
            # # get current state of throttle axis
            # # (between -1 and 1, but between 0 and 1 is expected)
            # throttle = float(input_list[2])
            # # get current state of yaw axis (between -1 and 1)
            # yaw = float(input_list[3])

            # await drone.manual_control.set_manual_control_input(pitch, roll, throttle, yaw)
            pitch, roll, throttle, yaw = 1, 0, 0.6, 0
            await drone.manual_control.set_manual_control_input(pitch, roll, throttle_value, yaw)
            print(f'{pitch=}, {roll=}, {throttle=}, {yaw=}')
            await asyncio.sleep(0.1)
            # pitch, roll, throttle, yaw = 0, 0, 0.5, 0
            # await drone.manual_control.set_manual_control_input(pitch, roll, throttle, yaw)
            # await asyncio.sleep(2)
            # pitch, roll, throttle, yaw = 0, 0, 0.5, 0.5
            # await drone.manual_control.set_manual_control_input(pitch, roll, throttle, yaw)
            # await asyncio.sleep(3)
            # pitch, roll, throttle, yaw = 0, 0, 0.5, -0.5
            # await drone.manual_control.set_manual_control_input(pitch, roll, throttle, yaw)
            # await asyncio.sleep(3)

    except Exception as e:
        print(f'Interrupt by user keyboard {e}')

    print("-- Landing")
    await drone.action.land()
    status_text_task.cancel()


async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(manual_controls())
