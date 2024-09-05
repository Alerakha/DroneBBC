import dronekit
from pymavlink import mavutil
import time
import math
from pymavlink import mavutil
import time

# Setup connection to the MAVLink
connection_string = '/dev/ttyUSB0'
mavlink_connection = mavutil.mavlink_connection(connection_string, baud=57600)
mavlink_connection.wait_heartbeat()
print("MAVLink connection established")

# Lidar addresses (replace with actual addresses)
lidar_addresses = {
    "front": 0x11,
    "right": 0x12,
    "left": 0x10
}

# Function to read Lidar sensor
def read_lidar(sensor_id):
    while True:
        msg = mavlink_connection.recv_match(type='DISTANCE_SENSOR', blocking=True)
        if msg.id == sensor_id:
            return msg.current_distance / 1000.0  # Convert from mm to meters
        time.sleep(0.5)

# Function to check if drone is armed
def is_drone_armed():
    mavlink_connection.mav.command_long_send(
        1, 1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    msg = mavlink_connection.recv_match(type='HEARTBEAT', blocking=True)
    return msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

# Main loop - sensor reading while disarmed
while not is_drone_armed():
    print("Drone is DISARMED")
    for position, address in lidar_addresses.items():
        distance = read_lidar(address)
        if distance is not None:
            print(f"{position.capitalize()} Lidar distance: {distance:.2f} meters")
            if position == "front" and distance < 0.8:
                print("Detect tembok depan!")
            elif position == "right" and distance < 0.8:
                print("Detect tembok kanan!")
            elif position == "left" and distance < 0.8:
                print("Detect tembok kiri!")
            else:
                print(f"Error reading {position} lidar sensor")
            time.sleep(1)
    time.sleep(2)

#=================================================
print("Awas sikil awas ndasnya drone arming")
time.sleep(1)
print("take off")

# Arm the drone and take off
mavlink_connection.mav.command_long_send(
    1, 1,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)

time.sleep(5)

# Take off to desired altitude
mavlink_connection.mav.command_long_send(
    1, 1,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 0
)

while True:
    msg = mavlink_connection.recv_match(type='ALTITUDE', blocking=True)
    altitude = msg.altitude_monotonic / 1000.0  # Convert from mm to meters

    if altitude >= 0.8:
        print('ketinggian sudah {0:.2f}m'.format(altitude))
        break
    else:
        print("naik ketinggian: {0:.2f}m".format(altitude))
    time.sleep(0.25)

# Hover at altitude
i = 0
while i < 20:
    print('hovering rn on {0:.2f}m'.format(altitude))
    mavlink_connection.mav.manual_control_send(
        0, 0, 0, 1450, 1500
    )
    time.sleep(0.5)
    if i % 1 == 0:
        print(i / 4)
    i += 1

print('hovering rn on {0:.2f}m'.format(altitude))
mavlink_connection.mav.manual_control_send(
    0, 0, 0, 1450, 1350
)
time.sleep(3)

# Landing and disarming
print("Landing...")
mavlink_connection.mav.command_long_send(
    1, 1,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 0, 0, 0, 0, 0, 0, 0
)

# Disarm the drone
time.sleep(5)
mavlink_connection.mav.command_long_send(
    1, 1,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 0, 0, 0, 0, 0, 0, 0
)

print("Drone disarmed")

