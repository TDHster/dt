from questionary import Separator, prompt
import argparse
from mavlink_th_control import MavlinkDrone # was tested - ok
from mavlink_pwm_control import MavlinkDrone as Drone
# from mavlink_drone import MavlinkDrone as Drone
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
                # "emergency",
                "mode brake",
                "mode land",
                "mode guided",
                "mode poshold",
                "arm",
                "arm_2989",
                "arm_21196",
                "disarm",
                "disarm_2989",
                "disarm_21196",
                "takeoff",
                "takeoff manual",
                Separator(),
                "hover",
                "up",
                "down",
                "rotate CW",
                "rotate CCW",
                "left",
                "right",
                "forward",
                "backward",
                Separator(),
                "send empty",
                "exit"
            ]
        }
    ]
    return prompt(questions, **kwargs)

if __name__ == "__main__":
    CONTROL_STEP = 0.3

    # MOVE_VALUE = 0.5
    # YAW_VALUE = 30
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
            case "mode land":
                drone.mode_land()
            case "mode guided":
                drone.mode_guided()
            case "mode poshold":
                drone.mode_position_hold()
            case "arm":
                drone.arm()
            case "arm_2989":
                drone.arm_21196()
            case "arm_21196":
                drone.arm_21196()
            case "disarm":
                drone.disarm()
            case "disarm_2989":
                drone.disarm_2989()
            case "disarm_21196":
                drone.disarm_21196()
            case "takeoff":
                drone.takeoff(12)
            case "takeoff manual":
                # drone.takeoff(12)
                drone._takeoff_manual()
            case "hover":
                drone.do_hover()
            case "up":
                drone.thrust += CONTROL_STEP
            case "down":
                drone.thrust -= CONTROL_STEP
            case "left":
                drone.roll += CONTROL_STEP
            case "right":
                drone.roll -= CONTROL_STEP
            case "forward":
                drone.pitch += CONTROL_STEP
            case "backward":
                drone.pitch += CONTROL_STEP
            case "rotate CW":
                drone.yaw -= CONTROL_STEP
            case "rotate CCW":
                drone.yaw += CONTROL_STEP
            case "send empty":
                drone.send_empty()

            case "exit":
                exit(0)


