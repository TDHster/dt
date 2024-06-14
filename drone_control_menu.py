from questionary import Separator, prompt
import argparse
from mavlink_th_control import MavlinkDrone # was tested - ok
# from mavlink_pwm_control import MavlinkDrone
from mavlink_drone import MavlinkDrone as Drone
from time import sleep


parser = argparse.ArgumentParser(description="Mavlink сontrol")

parser.add_argument(
    "-c",
    "--connectionstring",
    type=str,
    # default="tcp:192.168.0.177:5762",
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
                "emergency",
                "brake",
                "mode land",
                "mode guided",
                "arm",
                "disarm",
                "takeoff",
                Separator(),
                "nav rotate CW",
                "nav rotate CCW",
                "nav left",
                "nav right",
                "nav forward",
                "nav backward",
                Separator(),
                "exit"
            ]
        }
    ]
    return prompt(questions, **kwargs)

if __name__ == "__main__":
    MOVE_VALUE = 0.5
    YAW_VALUE = 30
    #working config:
    # udpin:127.0.0.1:14550 =  udpin:127.0.0.1:14550
    # pi@raspberrypi:~ $ mavproxy.py --master=/dev/ttyACM0 --out=udpout:0.0.0.0:14550
    # arm uncheck all

    print(f'Trying to connect: {connection_string}')
    drone = Drone(connection_string)
    print('Connected.')

    while True:
        drone_command = ask_dictstyle()["drone_command"]

        # try:
        #     drone_command = ask_dictstyle()["drone_command"]
        # except KeyError:
        #     # print('Emergency off')
        #     # drone.emergency_stop()
        #     # drone.mode_land()
        #     break
        #     exit(0)
        print(drone_command)
        match drone_command:
            case "emergency":
                drone.emergency_stop()
            case "break":
                drone.set_mode_brake()
            case "mode brake":
                drone.set_mode_land()
            case "mode guided":
                drone.set_mode_guided()
            case "mode RTL":
                drone.set_mode_return_to_land()
            case "arm":
                drone.arm()
            case "disarm":
                drone.disarm()
            case "takeoff":
                drone._takeoff(2)

            case "nav left":
                drone.change_position(dy=-MOVE_VALUE)
            case "nav right":
                drone.change_position(dy=MOVE_VALUE)
            case "nav forward":
                drone.change_position(dx=MOVE_VALUE)
            case "nav backward":
                drone.change_position(dx=-MOVE_VALUE)
            case "nav rotate CW":
                drone.yaw(yaw=YAW_VALUE)
            case "nav rotate CCW":
                drone.yaw(yaw=YAW_VALUE)

            case "exit":
                exit(0)


