from questionary import Separator, prompt
import argparse
from mavlink_th_control import MavlinkDrone
from time import sleep


parser = argparse.ArgumentParser(description="Mavlink сontrol")

parser.add_argument(
    "-c",
    "--connectionstring",
    type=str,
    default="udpin:127.0.0.1:14550",
    help="Specify path for mavlink connection",
)

args = parser.parse_args()
connection_string = args.connectionstring
print(f"Using mavlink connection string: {connection_string}")

def ask_dictstyle(**kwargs):
    questions = [
        {
            "type": "select",
            "name": "drone_command",
            "message": "What do you want to do?",
            "choices": [
                "mode brake",
                "mode land",
                "mode alt_hold",
                "arm",
                "disarm",
                Separator(),
                "attitude takeoff",
                "attitude yaw left",
                "attitude yaw right",
                "attitude roll left",
                "attitude roll right",
                "attitude pitch back",
                "attitude pitch forward",
                "attitude land"
            ]
        }
    ]
    return prompt(questions, **kwargs)

if __name__ == "__main__":
    EXPOSURE_TIME = 3
    EXPOSURE_XY_VALUE = 0.4
    #working config:
    # udpin:127.0.0.1:14550 =  udpin:127.0.0.1:14550
    # pi@raspberrypi:~ $ mavproxy.py --master=/dev/ttyACM0 --out=udpout:0.0.0.0:14550
    # arm uncheck all

    print(f'Trying to connect: {connection_string}')
    drone = MavlinkDrone(connection_string)
    print('Connected.')

    while True:
        try:
            drone_command = ask_dictstyle()["drone_command"]
        except KeyError:
            print('Emergency off')
            drone.emergency_stop()
            exit(0)
        print(drone_command)
        match drone_command:
            case "mode brake":
                drone.mode_brake()
            case "mode land":
                drone.mode_land()
            case "mode alt_hold":
                drone.mode_alt_hold()
            case "arm":
                drone.arm()
            case "disarm":
                drone.disarm()

            case "attitude takeoff":
                # drone.manual_takeoff()
                drone.thrust(0.5)
                sleep(EXPOSURE_TIME)
                drone.thrust(0.6)
                sleep(EXPOSURE_TIME)
                drone.thrust(0.5)
            case "attitude yaw left":
                drone.yaw(-0.3)
                sleep(EXPOSURE_TIME)
                drone.yaw(0)
            case "attitude yaw right":
                drone.yaw(0.3)
                sleep(EXPOSURE_TIME)
                drone.yaw(0)
            case "attitude roll left":
                drone.roll(EXPOSURE_XY_VALUE)
                sleep(EXPOSURE_TIME)
                drone.roll(0)
            case "attitude roll right":
                drone.roll(-EXPOSURE_XY_VALUE)
                sleep(EXPOSURE_TIME)
                drone.roll(0)
            case "attitude pitch back":
                drone.pitch(-EXPOSURE_XY_VALUE)
                sleep(EXPOSURE_TIME)
                drone.pitch(0)
            case "attitude pitch forward":
                drone.pitch(EXPOSURE_XY_VALUE)
                sleep(EXPOSURE_TIME)
                drone.pitch(0)
            case "attitude land":
                # drone.manual_land()
                drone.thrust(0.4)
                sleep(EXPOSURE_TIME)
                drone.thrust(0.3)
                sleep(EXPOSURE_TIME)
                drone.thrust(0.2)
                sleep(EXPOSURE_TIME)
                drone.thrust(0)

