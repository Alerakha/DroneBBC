from pymavlink import mavutil
import time 
import threading
# =======================================================================================||
usbser = '/dev/ttyUSB0'
vehicle = mavutil.mavlink_connection(usbser, baud=57600)
vehicle.wait_heartbeat()
print(f"Connected to {usbser}")	
print("OPENING : mavarm3_run.py....")
time.sleep(3)

# Checking Heartbeat
print("Heartbeat from (system %u component %u)" % (vehicle.target_system, vehicle.target_component))

# ===========================================
# =======================================================================================|| GET DISTANCE
# LIDAR Sensor Function
def get_distance(sensor_id):
    while True:
        msg = vehicle.recv_match(type='DISTANCE_SENSOR', blocking=True)
        if msg is not None:
            print(f"Received message from sensor with ID: {msg.id}")
            
            if msg.id == sensor_id:
                print(f"Matched sensor ID: {msg.id}")
                time.sleep(0.1)
                return msg.current_distance /100 # Return the current distance in meters
        else:
        	print("Error sensor not detected, {msg.id}")
        	time.sleep(0.1)


# ===========================================
# =======================================================================================|| GLOBAL SENSOR VARIABLES

# Global variables to hold sensor readings
front_distance = 0.0
right_distance = 0.0
left_distance = 0.0
bottom_distance = 0.0

def read_sensor(sensor_ID):
    if sensor_ID == 10:
        return get_distance(10)
    elif sensor_ID == 12:
        return get_distance(12)
    elif sensor_ID == 16:
        return get_distance(16)
    elif sensor_ID == 0:
        return get_distance(0)


# ============================================|| UPDATE SENSOR

def update_sensors():
    global front_distance, left_distance, right_distance, bottom_distance
    while True:
        front_distance = read_sensor(10)
        left_distance = read_sensor(16)
        right_distance = read_sensor(12)
        bottom_distance = read_sensor(0)
        time.sleep(0.2)  # Delay to reduce CPU usage

# Function to continuously read sensor data
# Start the sensor reading in a separate thread
sensor_thread = threading.Thread(target=update_sensors)
sensor_thread.daemon = True  # Ensures the thread will close when the main program exits
sensor_thread.start()

# ===========================================
# =======================================================================================|| ARM STATE
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
		
# ============================================
# =======================================================================================|| TAKE OFF
# Take Off
def takeoff(alt):
	bottom_distance = get_distance(0)
	print(f"Taking off to altitude {alt} m")
	while bottom_distance < alt: #Timer 3 detik dengan time.sleep(3)	
		print (f"Drone Altitude: {bottom_distance:.2f} meters")
		rcover(1500, 1500, 1620, 0, 0, 0, 0, 0)
		bottom_distance = get_distance(0)
		if bottom_distance >= alt:
			print("Target heigth reached. Hovering...")
			rcover(1500, 1500, 1450, 0, 0, 0, 0, 0) 
			bottom_distance = get_distance(0)
			
		

# ============================================
# =======================================================================================|| LANDING THEN DISARM
# Landing and Disarm
def landing():
	bottom_distance = get_distance(0)
	print (f"Drone Altitude: {bottom_distance:.2f} meters")
	
	if bottom_distance > 0.15:
		print("Approaching land..")
		rcover(1500, 1500, 1450, 0, 0, 0, 0, 0)
		time.sleep(0.5)
		bottom_distance = get_distance(0)
	elif bottom_distance < 0.15:
		print("Reached altitude < 0.15")
		print(f"DISARMING...")
		arm(0)
		
# ============================================
# =======================================================================================|| CHANGE MODE
# Change Mode
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
	

# ============================================
# =======================================================================================|| READ SENSOR

def read_sensor(durasi):
	i = 1
	while i <= durasi:
	    '''
	    front_distance = get_distance(10)  # Replace 1 with the actual sensor ID
	    right_distance = get_distance(12)  # Replace 2 with the actual sensor ID
	    left_distance = get_distance(16)  # Replace 3 with the actual sensor ID
	    bottom_distance = get_distance(0)
	    '''
	    print("Depan : ", end="")
	    print(front_distance)
	    print("Kanan : ", end="")
	    print(right_distance)
	    print("Kiri : ", end="")
	    print(left_distance)
	    print("bottom : ", end="")
	    print(bottom_distance)
	    time.sleep(0.2)
	    i +=1


# ============================================
# =======================================================================================|| ADJUST POSISI

def adjust():
#Adjust posisi tengah
		if left_distance < 0.7:
			nilaikiri = 150 - (left_distance*10)
			rcover(1500 + nilaikiri, 1450, 1450,0,0,0,0,0) #rodok nganan
			print("Drone rodok nganan")
		elif right_distance < 0.7:
			nilaikanan = 150 - (right_distance*10)
			rcover(1500 - nilaikanan,1450,1500,0,0,0,0) #rodok ngiri
			print("Drone rodok ngiri")

# ============================================
# =======================================================================================|| RCOVER

def rcover(ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8): # rc minimum is 1000, max is 2000
	vehicle.mav.rc_channels_override_send(
		vehicle.target_system,
		vehicle.target_component,
		ch1,	# Kiri - Kanan + 
		ch2,	# Maju - Mundur +
		ch3,	# Throttle
		ch4,	#
		ch5,
		ch6,
		ch7,
		ch8
		)
			
# ============================================
# =======================================================================================|| MAJU
def maju():
	while front_distance >= 1:
		print("Drone maju")
		rcover(65535,1420,1500,0,0,0,0,0) 
		#Adjust posisi tengah
		front_distance = get_distance(10)
		right_distance = get_distance(12)
		left_distance = get_distance(16)
		if left_distance < 0.7:
			nilaikiri = 150 + (left_distance*10)
			rcover(1500 + nilaikiri, 65535, 65535,0,0,0,0,0) #rodok nganan
			print("Drone rodok nganan")
		elif right_distance < 0.7:
			nilaikanan = 150 - (right_distance*10)
			rcover(1500 - nilaikanan,65535,65535,0,0,0,0) #rodok ngiri
			print("Drone rodok ngiri")
			
	else:
		print("Depan ada hambatan, Drone Hovering!")
		rcover(1500,1500,1520,0,0,0,0,0)

# ============================================
# =======================================================================================|| BELOK
def belok():
	if left_distance < 1 and front_distance < 1 :
		print("Kanan ada hambatan, belok kiri!")
		rcover(0,1500,1500,1430,0,0,0,0)
		if front_distance > 3:
			rcover(65535,1420,1500,0,0,0,0,0)
			time.sleep(1)
			continue
	elif right_distance < 1 and front_distance < 1:
		print("Kiri ada hambatan, belok kanan!")
		rcover(0,1500,1500,1580,0,0,0,0)
		if front_distance > 3:
			rcover(65535,1420,1500,0,0,0,0,0)
			time.sleep(1)
			continue
	else :
		print("Else Belok, Hovering!")
		rcover(1500,1500,1520,0,0,0,0,0)
 
# ============================================
# =======================================================================================|| PROGRAM 1

# PROGRAM 1 TAKEOFF AND HOVERING AND LANDING
def program1():
	print("Running Program 1")
	bottom_distance = get_distance(0)
	front_distance = get_distance(10)
	print("Program1 will do : TAKE OFF, HOVER then LANDING 'WITHOUT ARMING and DISARMING the Drone'")
	while vehicle.motors_armed() == True and bottom_distance is not None and front_distance is not None:
		changemode(5)
		time.sleep(2)
		print ("Try 'Autonomous Take Off'")
		print ("Remote ready cek drone e gak liar")
		time.sleep(2)
		takeoff(0.3)
		print ("Taking off engaged")
		
		while bottom_distance <= 0.8:
			rcover(0,0,1620,0,0,0,0,0)
			print(f"bottom distance : {bottom_distance}")
		else:
			rcover(0,0,1450,0,0,0,0,0)
			print("hover")
			time.sleep(2)
			changemode(9)
			print("Harus e wes landing, Disarm manual sek ngge remote!")
	else :
		print("Motor is disarmed")

# ============================================
# =======================================================================================|| PROGRAM 3

def leron():
	print("=-===-==-==-=-=-=-=-===-=-=-=-=-===-=-=-=-=-=-=-==-=-====-==-==-===-=-=--=-=-")
	arm(1)
	print("DRONE IS ARMING!!!")
	changemode(5)
	takeoff(0.6)
	print("=============================================================================",  end="")
	print("IF THIS IS PRINTED THEN TAKE OFF ALREADY DONE")
	time.sleep(2)
	if bottom_distance > 0.4:
		maju()
		belok()
		maju()
		if front_distance >= 1 and left_distance > 2 and right_distance > 2:
			rcover(1500, 1500, 1620, 0, 0, 0, 0, 0)
			print("Program Done, Please Override with Remote!")
			changemode(9) #jaga-jaga landing
			if front_distance > 2:
				maju()
				time.sleep(2)
				changemode(9)
	else:
		print("Drone in low altitude!!!")
		changemode(9) #Landing mode
		arm(0)
			
# ============================================
# =======================================================================================|| TIMER CODE
def timer():
	print("TRY CODE TIMED DRONE")
	changemode(5)
	print("=============================================================================", end="")
	print("=============================================================================")
	time.sleep(2)
	print("Timer Mode Engaged")
	rcover(1500, 1500, 1620, 0, 0, 0, 0, 0)
	time.sleep(2)
	print("Start maju")
	rcover(65535,1430,1500,0,0,0,0,0) 
	time.sleep(3)
	changemode(9)
	#print("OTW YAW KIRI")
	#rcover(0,1500,1500,1430,0,0,0,0)
	time.sleep(1)
	changemode(9)

# =======================================================================================|| NOTES
# =======================================================================================|| NOTES
# notes : 
# ch1 = miring kiri kanan
# ch2 = miring maju mundur (reverse value)
# ch3 = throttle
# ch4 = yaw
# nilai min 1000, nilai max 2000, tengah 1500
# modes :
# 0 = stabilize
# 5 = loiter
# 9 = land
# sensors :
# front_distance = read_sensor(10)
# left_distance = read_sensor(16)
# right_distance = read_sensor(12)
# bottom_distance = read_sensor(0)
# =======================================================================================|| NOTES
# =======================================================================================|| NOTES



# =======================================================================================
# ===================|| CODE THE DRONE OPERATION BELOW HERE||============================

print("TRY CODE AUTO DRONE")
time.sleep(2)
#changemode(5)
#print("TESTING SENSOR")
#read_sensor(3)
print("=============================================================================", end="")
print("=============================================================================")

leron()
#timer()
print("PROGRAM FINISHED!! PLEASE AUTO CONTROLL THE DRONE NOW!!")

# =======================================================================================
# =======================================================================================
