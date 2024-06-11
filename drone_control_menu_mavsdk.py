from questionary import Separator, prompt
import argparse
from mavlink_th_control import MavlinkDrone # was tested - ok
# from mavlink_pwm_control import MavlinkDrone
from mavsdk_control import MavlinkDrone
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
    EXPOSURE_TIME = 3
    EXPOSURE_XY_VALUE = 0.4
    #working config:
    # udpin:127.0.0.1:14550 =  udpin:127.0.0.1:14550
    # pi@raspberrypi:~ $ mavproxy.py --master=/dev/ttyACM0 --out=udpout:0.0.0.0:14550
    # arm uncheck all

    print(f'Trying to connect: {connection_string}')
    drone = MavlinkDrone(connection_string)
    drone.connect()
    print('Connected.')

    while True:
        try:
            drone_command = ask_dictstyle()["drone_command"]
        except KeyError:
            print('Emergency off')
            drone.kill()
            exit(0)
        print(drone_command)
        match drone_command:
            case "arm":
                drone.arm()
            case "arm_force":
                drone.arm_force()
            case "disarm":
                drone.disarm()
            case "get_takeoff_altitude":
                takeoff_altitude = drone.get_takeoff_altitude()
                print(takeoff_altitude)
                sleep(3)
            case "set_takeoff_altitude":
                drone.set_takeoff_altitude(2)
            case "takeoff":
                drone.takeoff()
            case "land":
                drone.land()
            case "kill":
                drone.kill()
            case "start_attitude_control":
                drone.start_attitude_control()
            case "start_position_control":
                drone.start_position_control()
