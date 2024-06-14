#!/usr/bin/env python3

"""
Caveat when attempting to run the examples in non-gps environments:

`drone.offboard.stop()` will return a `COMMAND_DENIED` result because it
requires a mode switch to HOLD, something that is currently not supported in a
non-gps environment.
"""

import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)


move_distance = 1  # meter

async def run():
    """ Does Offboard control using position NED coordinates. """

    drone = System()
    await drone.connect(system_address="udp://127.0.0.1:14550")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    #------
    await asyncio.sleep(1)
    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(3)

    #------

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed \
                with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # print(f"-- Go 0m North, 0m East, {move_distance}m Down within local coordinate system")
    # await drone.offboard.set_position_ned(
    #         PositionNedYaw(0.0, 0.0, -move_distance, 0.0))
    # await asyncio.sleep(10)

    print(f"-- Go {move_distance}m North, 0m East, 0m Down within local coordinate system, turn to face East")
    await drone.offboard.set_position_ned(
            PositionNedYaw(move_distance, 0.0, 0, 90.0))
    await asyncio.sleep(10)

    print(f"-- Go 0m North, {move_distance}m East, 0m Down within local coordinate system")
    await drone.offboard.set_position_ned(
            PositionNedYaw(0, move_distance, 0, 90.0))
    await asyncio.sleep(5)

    print(f"-- Go 0m North, -{move_distance}m East, 0m Down \
            within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, move_distance, 0.0, 180.0))
    await asyncio.sleep(5)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed \
                with error code: {error._result.result}")


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
