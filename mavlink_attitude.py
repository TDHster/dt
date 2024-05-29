from enum import IntEnum
from pymavlink import mavutil
from time import sleep
import argparse

parser = argparse.ArgumentParser(description="Mavlink сontrol")

parser.add_argument(
    "-c", "--connectionstring",
    type=str, default="udpin:127.0.0.1:14550", help="Specify path for mavlink connection",
)

args = parser.parse_args()
connection_string = args.connectionstring
print(f"Using mavlink connection string: {connection_string}")


# Define target attitude message class
class TargetAttitude:
    def __init__(self, thrust, roll, pitch, yaw):
        self.thrust = thrust  # Normalized thrust value (0-1)
        self.roll = roll  # Desired roll angle (radians)
        self.pitch = pitch  # Desired pitch angle (radians)
        self.yaw = yaw  # Desired yaw angle (radians)


# Function to send SET_ATTITUDE_TARGET message
def send_attitude_target(mav, target_attitude):
    msg = mavutil.mavlink.MAVLink_message_set_attitude_target_pack(
        mav.target_system, mav.target_component, 0,  # Target system, target component, ignore rate
        0,  # Time since system boot (ms)
        target_attitude.thrust,
        target_attitude.roll, target_attitude.pitch,
        target_attitude.yaw, 0, 0)  # Include yaw, ignore yaw rate and body_roll_rate
    mav.send(msg)


# Initialize MAVLink connection (replace with your connection method)
mav = mavutil.mavlink_connection(connection_string)

# Define initial target attitude (hovering)
initial_attitude = TargetAttitude(0.5, 0, 0, 0)  # 50% thrust, level attitude, 0 yaw

# Send initial hover command
send_attitude_target(mav, initial_attitude)

# Gradually increase target thrust for ascent
for i in range(1, 11):  # Adjust loop iterations for desired climb rate
    thrust = i * 0.05  # Increase thrust in steps of 5%
    send_attitude_target(mav, TargetAttitude(thrust, 0, 0, initial_attitude.yaw))
    # Implement logic to monitor altitude and adjust thrust/pitch as needed
    # (This part is not included in this simplified example)
    sleep(0.5)  # Adjust sleep time for desired ascent speed

# Implement logic for maintaining altitude or transitioning to other flight modes

# Close MAVLink connection
mav.close()
