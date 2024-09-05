from pymavlink import mavutil
import time
import math

i = 0

print("program wilayah V0.01")
#Connect ke telemetri
usbser = '/dev/ttyUSB0'	

#Koneksi vehicle
#vehicle = dronekit.connect(connection_string, baud=57600, wait_ready = True, timeout = 150)
print("Connected to ")	
print(usbser)
vehicle = mavutil.mavlink_connection(usbser, baud=57600)
vehicle.wait_heartbeat()

def arm(armstate): # 1 or 0
	vehicle.mav.command_long_send(
	vehicle.target_system, 
	vehicle.target_component, 
	mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, #send cmd
		0, #confirm
		armstate, #arm 1 dis 0
		0,
		0,
		0,
		0,
		0,
		0
	)
	
	msg = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
	if msg.result == 0 and armstate == 1:
		print("Drone is ARMED")
	elif msg.result == 0 and armstate == 0:
		print("Drone is DISARMED")
	else:
		print("Sumting Wong")

def changemode(modeval):
	modeflag = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
	vehicle.mav.set_mode_send(
		vehicle.target_system,
		modeflag,
		modeval
	)

	# Adding mode name to print
	mode_values = {
		0: "stabilize",
		5: "loiter",
		9: "land"
	}
	# ========================
	mode_name = mode_values.get(modeval, "mode unknown")
	print(f"Mode is : {mode_name}")
	
	

def rcover(ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8): # rc minimum is 1000, max is 2000
	vehicle.mav.rc_channels_override_send(
		vehicle.target_system,
		vehicle.target_component,
		ch1,
		ch2,
		ch3,
		ch4,
		ch5,
		ch6,
		ch7,
		ch8
		)
			
#=================================================
# Pymavlink
# Define a function to read distance from a sensor
def get_distance(sensor_id):
    while True:
        msg = vehicle.recv_match(type='DISTANCE_SENSOR', blocking=True)
        if msg is not None:
            print(f"Received message from sensor with ID: {msg.id}")
           # return msg.current_distance
            if msg.id == sensor_id:
                print(f"Matched sensor ID: {msg.id}")
                return msg.current_distance  # Return the current distance in meters
                time.sleep(0.1)


def program3():
	changemode(5)
	x = 0
	front_distance = get_distance(10)
	right_distance = get_distance(12)
	left_distance = get_distance(16)
	bottom_distance = get_distance(0)
	time.sleep(1)
	while bottom_distance is not None and front_distance is not None:
		arm(1)
		print("VROOOM VROOOMM NIGGERRSS")
		bottom_distance = get_distance(0)
		time.sleep(0.02)
		while bottom_distance <= 60:
			print(f"taking off @ {bottom_distance}")
			rcover(1500,1500,1620,1500,0,0,0,0)
			bottom_distance = get_distance(0)
		print(f"keluar while takeoff, alt = {bottom_distance}")
		print("HOVERING")
		rcover(1500,1500,1500,1500,65535,65535,65535,65535)
		time.sleep(3)
		front_distance = get_distance(10)
		time.sleep(0.02)
		#YANG TERJADI KETIKA DRONE MENDEKATI TEMBOK DEPAN
		while front_distance <= 130:
			front_distance = get_distance(10)
			time.sleep(0.02)
			rcover(65535,1420,1500,1500,65535,65535,65535,65535) 
			print(f"maju, {front_distance}")
			right_distance = get_distance(12)
			time.sleep(0.02)
			left_distance = get_distance(16)
			time.sleep(0.02)
			#ADJUST
			while left_distance < 80:
				kiriint = 100 - left_distance
				rcover(1500 - kiriint,1420,1500,65535,65535,65535,65535,65535)
				print("ke ngiri nen")
				left_distance = get_distance(16)
				time.sleep(0.02)
			while right_distance < 80:
				kananint = 100 - right_distance
				rcover(1500 + kananint,1420,1500,65535,65535,65535,65535,65535)
				print("nganan bgt bro")
				right_distance = get_distance(12)
				time.sleep(0.2)
		print("keluar while front maju")
		rcover(1500,1650,1450,1500,0,0,0,0) #rem
		print("stoppppp")
		#time.sleep(0.8) #IKI OFF
		#changemode(9)	#========
		#arm(0)		#========
		print("Program keluar")
		
		#Atau bawah iki off
		while left_distance > 350:
			time.sleep(0.02)
			left_distance = get_distance(16)
			print("Jalan ke kiri!")
			time.sleep(0.02)
			rcover(1500,1440,1500,1500,0,0,0,0)
		#DRONE UP
		print(f"taking off @ {bottom_distance}")
		rcover(1500,1500,1620,1500,0,0,0,0) #naik
		left_distance = get_distance(16)
		
		if left_distance > 400:
			rcover(1640,1500,1500,0,0,0,0)
			time.sleep(8)
		print("program3 done")
		changemode(9)
		arm(0)
	else :
		print("init sensor.....")
'''    
#Set mode
def set_mode(mode):
    if mode == "LOITER":
        mode_id = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
        print("Mode set to LOITER")
    else:
        raise ValueError(f"Mode {mode} not supported in this example.")

    # Send command to change mode
    mavlink_connection.mav.command_long_send(
        1,  # target_system (usually 1 for the main system)
        1,  # target_component (usually 1 for the autopilot)
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # Command ID
        0,  # Confirmation
        0,  # param1: Custom mode
        mode_id,  # param2: Mode ID
        0, 0, 0, 0, 0  # Other parameters are unused
    )

    # Confirm mode change by checking the heartbeat
    while True:
        msg = mavlink_connection.recv_match(type='HEARTBEAT', blocking=True)
        if msg.custom_mode == mode_id:
            print(f"Mode changed to {mode}")
            break
        time.sleep(1)

# Change to LOITER mode
set_mode("LOITER")

#=================================================
#Dronekit
print("Awas sikil awas ndasnya drone arming")
time.sleep(1)
print("take off")
while True:
	vehicle.channels.overrides[3] = 1620
	vehicle.channels.overrides[1] = 1500
	vehicle.channels.overrides[2] = 1500
	if vehicle.rangefinder.distance >=0.8:
		print('ketinggian sudah {0:.2f}m'.format(vehicle.rangefinder.distance))
		break
	else:
		print("naik ketinggian: {0:.2f}m".format(vehicle.rangefinder.distance))
	time.sleep(0.25)

while i<20:
	print('hovering rn on {0:.2f}m'.format(vehicle.rangefinder.distance))
	vehicle.channels.overrides[3] = 1450 # throttle
	vehicle.channels.overrides[1] = 1500 # kanan+ kiri-
	vehicle.channels.overrides[2] = 1500 # maju- mundur+
	time.sleep(0.5)
	if i%1==0:
		print(i/4)
	i=i+1

print('hovering rn on {0:.2f}m'.format(vehicle.rangefinder.distance))
vehicle.channels.overrides[3] = 1450
vehicle.channels.overrides[1] = 1500
vehicle.channels.overrides[2] = 1350
time.sleep(3)

print("Landing...")
vehicle.mode = dronekit.VehicleMode('LAND')
if vehicle.rangefinder.distance < 0.15:
	print("disarming...")
	vehicle.armed = False 
#=================================================
#=================================================
#Mavlink



#Try landing
print("Landing...")
mavlink_connection.mav.set_mode_send(
    mavlink_connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mavutil.mavlink.MAV_MODE_LAND
)
time.sleep(1)


# Disarming the drone
mavlink_connection.mav.command_long_send(
    mavlink_connection.target_system,
    mavlink_connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0
)
print("disarming...")
'''
# Main loop
while True:
    front_distance = get_distance(10)  # Replace 1 with the actual sensor ID
    right_distance = get_distance(12)  # Replace 2 with the actual sensor ID
    left_distance = get_distance(16)  # Replace 3 with the actual sensor ID
    bottom_distance = get_distance(0)
    print("Depan : ", end="")
    print(front_distance)
    print("Kanan : ", end="")
    print(right_distance)
    print("Kiri : ", end="")
    print(left_distance)
    print("bottom : ", end="")
    print(bottom_distance)
    time.sleep(0.2)
    program3()
    break
    
