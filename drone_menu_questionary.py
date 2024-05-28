import questionary

from pprint import pprint

import questionary
from questionary import Choice
from questionary import Separator
from questionary import prompt

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
                "mode_acro",
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
    # pprint(ask_dictstyle()["drone_command"])
    while True:
        drone_command = ask_dictstyle()["drone_command"]

        match drone_command:
            case "arm":
                print("Selected option: arm")
