import mavsdk.offboard
from questionary import Separator, prompt
import argparse
# from mavlink_th_control import MavlinkDrone # was tested - ok
# from mavlink_pwm_control import MavlinkDrone
# from mavsdk_control import MavlinkDrone
from time import sleep, time

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)



parser = argparse.ArgumentParser(description="Mavlink сontrol")

parser.add_argument(
    "-c",
    "--connectionstring",
    type=str,
    default="udp://127.0.0.1:14550",
    help="Specify path for mavlink connection",
)

args = parser.parse_args()
connection_string = args.connectionstring
print(f"Using mavlink connection string: {connection_string}")




async def run():

    drone = System()
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

    print("-- Arming")
    await drone.action.arm()
    # await drone.action.arm_force()

    await drone.manual_control.set_manual_control_input(0,0,0,0)

    takeoff_altitude = 5  # not in meters
    await drone.action.set_takeoff_altitude(takeoff_altitude)
    takeoff_altitude = await drone.action.get_takeoff_altitude()
    print(f'Current set takeoff altitude: {takeoff_altitude}')

    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(5)

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed \
                    with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # takeoff_altitude = await drone.action.get_takeoff_altitude()
    # print(f'Current setted takeoff altitude: {takeoff_altitude}')


    # await drone.manual_control.start_position_control()
    # await drone.manual_control.start_altitude_control()




    # await drone.manual_control.set_manual_control_input(0,0,0,1)

    # await drone.offb  oard.set_position_ned(mavsdk.offboard.PositionNedYaw(north_m=0, east_m=-3, down_m=0, yaw_deg=0)) # rotate CW 30 deg

    # await drone.offboard.set_position_ned(mavsdk.offboard.PositionNedYaw(north_m=0, east_m=0, down_m=-2, yaw_deg=0))
    # await drone.offboard.set_attitude(mavsdk.offboard.Attitude(roll_deg=0, pitch_deg=0, yaw_deg=30,thrust_value=0.5))

    await asyncio.sleep(10)

    # await drone.manual_control.start_position_control()
    # duration_seconds = 10
    # yaw = -1
    # start_time = time()
    # while time() - start_time < duration_seconds:
    #     await drone.manual_control.set_manual_control_input(0, 0, 0, yaw)
    #     print(f'Sent {yaw=}')
    #     sleep(1/10)

    # await asyncio.sleep(10)

    # print("-- Stopping offboard")
    # try:
    #     await drone.offboard.stop()
    # except OffboardError as error:
    #     print(f"Stopping offboard mode failed \
    #             with error code: {error._result.result}")


    print("-- Landing")
    await drone.action.land()

    status_text_task.cancel()


async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return


def ask_dictstyle(**kwargs):
    questions = [
        {
            "type": "select",
            "name": "drone_command",
            "message": "What do you want to do?",
            "choices": [
                "arm",
                "arm_force",
                "disarm",
                "get_takeoff_altitude",
                "set_takeoff_altitude",
                "takeoff",
                Separator(),
                "land",
                "kill",
                Separator(),
                "start_attitude_control",
                "start_position_control",
                # Separator(),
                # "manual CW",
                # "manual CCW",
                # "manual left",
                # "manual right",
                # "manual forward",
                # "manual backward",
                Separator()
            ]
        }
    ]
    return prompt(questions, **kwargs)


if __name__ == "__main__":

    # Run the asyncio loop
    asyncio.run(run())


    # EXPOSURE_TIME = 3
    # EXPOSURE_XY_VALUE = 0.4
    # #working config:
    # # udpin:127.0.0.1:14550 =  udpin:127.0.0.1:14550
    # # pi@raspberrypi:~ $ mavproxy.py --master=/dev/ttyACM0 --out=udpout:0.0.0.0:14550
    # # arm uncheck all
    #
    # print(f'Trying to connect: {connection_string}')
    # drone = MavlinkDrone(connection_string)
    # drone.connect()
    # print('Connected.')
    #
    # while True:
    #     try:
    #         drone_command = ask_dictstyle()["drone_command"]
    #     except KeyError:
    #         print('Emergency off')
    #         drone.kill()
    #         exit(0)
    #     print(drone_command)
    #     match drone_command:
    #         case "arm":
    #             drone.arm()
    #         case "arm_force":
    #             drone.arm_force()
    #         case "disarm":
    #             drone.disarm()
    #         case "get_takeoff_altitude":
    #             takeoff_altitude = drone.get_takeoff_altitude()
    #             print(takeoff_altitude)
    #             sleep(3)
    #         case "set_takeoff_altitude":
    #             drone.set_takeoff_altitude(2)
    #         case "takeoff":
    #             drone.takeoff()
    #         case "land":
    #             drone.land()
    #         case "kill":
    #             drone.kill()
    #         case "start_attitude_control":
    #             drone.start_attitude_control()
    #         case "start_position_control":
    #             drone.start_position_control()
