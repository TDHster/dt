#!/usr/bin/env python3

import asyncio
from mavsdk import System


async def run():

    drone = System()
    print('Trying to connect...')
    await drone.connect(system_address="udp://127.0.0.1:14550")
    status_text_task = asyncio.ensure_future(print_status_text(drone))

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

    await drone.action.set_takeoff_altitude(3)
    await asyncio.sleep(1)

    print("-- Arming")
    await drone.action.arm()
    await asyncio.sleep(1)

    await drone.action.set_takeoff_altitude(2)

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(10)

    print("-- Hold")
    await drone.action.hold()
    await asyncio.sleep(20)

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
    asyncio.run(run())
