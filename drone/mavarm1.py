# no dronekit
from pymavlink import mavutil
import time

usbser = '/dev/ttyUSB0'
vehicle = mavutil.mavlink_connection(usbser, baud=57600)
vehicle.wait_heartbeat()
print("Connected to ", end="")	
print(usbser)
print("heartbeat from (system %u component %u)" % (vehicle.target_system, vehicle.target_component))

def arm(armstate): # 1 or 0
	vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM #send cmd
	, 0 #confirm
	, armstate #arm 1 dis 0
	, 0
	, 0
	, 0
	, 0
	, 0
	, 0)
	
	msg = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
	if msg.result == 0 and armstate == 1:
		print("Drone is ARMED")
	elif msg.result == 0 and armstate == 0:
		print("Drone is DISARMED")
	else:
		print("sumting wong")
		
def takeoff(alt):
	vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF #send cmd
	, 0 #confirm
	, 0 #arm 1 dis 0
	, 0
	, 0
	, 0
	, 0
	, 0
	, alt)
	
	msg = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
	if msg.result == 0:
		print("taking off to {alt} meter")
	else:
		print("DRONE CANT ARM, CHECK MODE")
		
def changemode(modeval):
	modeflag = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
	vehicle.mav.set_mode_send(
		vehicle.target_system,
		modeflag,
		modeval
	)
	print(f"mode is : {modeval}")

def landing():
	vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND, 
	0, 0, 0, 0, 0, 0, 0, 0)
	print("Landing..")
#========================================================================
def get_distance(sensor_id):
    while True:
        msg = mavlink_connection.recv_match(type='DISTANCE_SENSOR', blocking=True)
        if msg is not None:
            print(f"Received message from sensor with ID: {msg.id}")  # Print the received sensor ID
           # return msg.current_distance
            if msg.id == sensor_id:
                print(f"Matched sensor ID: {msg.id}")  # Print if it matches the sensor_id you're looking for
                return msg.current_distance  # Return the current distance in meters
                time.sleep(0.1)

#========================================================================

arm(1)
time.sleep(2)
changemode(5) # 5 is loiter
time.sleep(5)
takeoff(0.8)
# Main loop
while True:
    front_distance = get_distance(10)  # Replace 1 with the actual sensor ID
    right_distance = get_distance(12)  # Replace 2 with the actual sensor ID
    left_distance = get_distance(16)  # Replace 3 with the actual sensor ID
    print("Depan : ", end="")
    print(front_distance)
    print("Kanan : ", end="")
    print(right_distance)
    print("Kiri : ", end="")
    print(left_distance)
    time.sleep(0.2)
time.sleep(7)
landing()
time.sleep(3)
arm(0)
