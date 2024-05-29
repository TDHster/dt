import questionary
from mavlink_control import MavlinkDrone
from pprint import pprint
import questionary
from questionary import Choice
from questionary import Separator, prompt
from math import pi

import argparse

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
                "arm",
                "takeoff",
                "disarm",
                "mode_land",
                "mode_quided",
                Separator(),
                "yaw_left",
                "yaw_right",
                Separator(),
                "move_left",
                "move_right",
                "move_forward",
                "move_back",
            ],
        }
    ]

    # return prompt(questions, style=custom_style_dope, **kwargs)
    return prompt(questions, **kwargs)


if __name__ == "__main__":
    #working config:
    # udpin:127.0.0.1:14550 =  udpin:127.0.0.1:14550
    # pi@raspberrypi:~ $ mavproxy.py --master=/dev/ttyACM0 --out=udpout:0.0.0.0:14550
    # arm uncheck all
    print(f'Trying to connect: {connection_string}')
    drone = MavlinkDrone(connection_string)
    print('Connected.')
    # pprint(ask_dictstyle()["drone_command"])
    while True:
        # print(drone.get_message_local_position_ned())
        drone_command = ask_dictstyle()["drone_command"]
        if drone_command is None:
            drone.emergency_stop()
        match drone_command:
            case "arm":
                print("Selected option: arm")
                drone.arm()
            case "takeoff":
                print('Taking off')
                drone.takeoff2(1)
            case "disarm":
                drone.disarm()
            case "mode_land":
                drone.mode_land()
            case "mode_quided":
                drone.mode_guided()
            case "yaw_left":
                drone.yaw(15)
            case "yaw_right":
                drone.yaw(15)
            case "move_left":
                drone.move_test(-1, 0, 0)
            case "move_right":
                drone.move_test(1, 0, 0)
            case "move_forward":
                drone.move_test(0, 1, 0)
            case "move_back":
                drone.move_test(0, -1, 0)

        # print(drone.get_message_local_position_ned())


