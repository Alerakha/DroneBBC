# no dronekit
from pymavlink import mavutil
import time, threading

usbser = '/dev/ttyUSB0'

vehicle = mavutil.mavlink_connection(usbser, baud=57600)
vehicle.wait_heartbeat()
print(f"Connected to {usbser}")	
print("Operating mavarm3.py....")

# ===========================================
# =======================================================================================|| ARM STATE
# Checking Heartbeat
print("heartbeat from (system %u component %u)" % (vehicle.target_system, vehicle.target_component))

#Arming state
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
	print(f"Taking off to altitude {alt} m")
	i = 0 #Untuk timer
		while i < 15: #Timer 3 detik dengan time.sleep(3)	
			bottom_distance = get_distance(0)
			
			if bottom_distance >= alt:
				print("Target heigth reached. Hovering...")
				rcover(0, 0, 1500, 0, 0, 0, 0, 0) 
				time.sleep(3)
				
			else:
				print (f"Drone Altitude: {bottom_distance:.2f} meters")
				rcover(0, 0, 1620, 0, 0, 0, 0, 0)
			time.sleep(0.2)
			i += 1
		if i >= 15:
			print("Sensor not reading in 3 second, Move to Timed hover function")
			rcover(0, 0, 1500, 0, 0, 0, 0, 0) 
			time.sleep(3)

# ============================================
# =======================================================================================|| LANDING THEN DISARM
# Landing and Disarm
def landing():
	while True:
		bottom_distance = get_distance(0)
		print (f"Drone Altitude: {bottom_distance:.2f} meters")
		
		if bottom_distance > 0.15:
			print("Approaching land..")
			rcover(0, 0, 1450, 0, 0, 0, 0, 0)
			time.sleep(0.5)
		else:
			print("Reached altitude < 0.15")
			arm(0)
			print(f"DISARMING...")
			break
		time.sleep(0.2)

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
# =======================================================================================|| RCOVER2(DON'T USE THIS)
# Remote Controll Override	
def rcover2(ch, pwm=1500): # rc minimum is 1000, max is 2000
	if ch<1 or ch>8:
		print("not avail")
		return
	rcval = [65535 for _ in range(18)]
	for i in range(6):
		rcval[i] = 1500
	rcval[ch - 1] = pwm
	vehicle.mav.rc_channels_override_send(
		vehicle.target_system,
		vehicle.target_component,
		*rcval
		)

# ===========================================
# =======================================================================================|| GET DISTANCE
# LIDAR Sensor Function
def get_distance(sensor_id):
    while True:
        msg = vehicle.recv_match(type='DISTANCE_SENSOR', blocking=True)
        if msg is not None:
            print(f"Received message from sensor with ID: {msg.id}")
            # return msg.current_distance
            if msg.id == sensor_id:
                print(f"Matched sensor ID: {msg.id}")
                return msg.current_distance /100 # Return the current distance in meters
                time.sleep(0.1)
        else:
        	print("Error sensor not detected, {msg.id}")
        	time.sleep(0.1)

# ============================================
# =======================================================================================|| INDOOR (STILL DEVELOP)
# Try mode indoor
def indoor():
	while True:
		front_distance = get_distance(10)
		right_distance = get_distance(12)  
		left_distance = get_distance(16)  
		bottom_distance = get_distance(0)
		
		if front_distance > 0.7: # If untuk maju
			rcover(0,1420,1500,0,0,0,0,0) 
			
			# Adjust posisi tengah
			if left_distance < 0.7:
				rcover(1590,1480,1500,0,0,0,0,0) #rodok nganan
			elif right_distance < 0.7:
				rcover(1410,14800,1500,0,0,0,0) #rodok ngiri
			
		elif front_distance < 0.7: # Jika detect tembok/obstacle
			rcover(0,1500,1500,0,0,0,0,0)
			time.sleep(3) # Stop diluk
			
			# Cek kiri & kanan
			if left_distance < 0.7:
				rcover(0,1500,1500,1570,0,0,0,0) # Setting yaw to turn right
				time.sleep(2) #Waktu untuk muter (moga bener 90)
			elif right_distance < 0.7:
				rcover(0,1500,1500,0,1430,0,0,0) # Setting yaw to turn left
				time.sleep(2)
		time.sleep(3)
	time.sleep(0.1)

# ============================================
# =======================================================================================|| RCOVER

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
			
# ============================================
# =======================================================================================|| READ SENSOR

def read_sensor(time):
	i = 1
	while i <= time:
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
	    i +=1
	    
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
		time.sleep(5)
		#takeoff(0.3)
		print ("Taking off engaged")
		
		while bottom_distance <= 0.8:
			rcover(0,0,1620,0,0,0,0,0)
			print(f"bottom distance : {bottom_distance}")
		else:
			rcover(0,0,1450,0,0,0,0,0)
			print("hover")
			time.sleep(5)
			changemode(9)
			print("Harus e wes landing, Disarm manual sek ngge remote!")
	else :
		print("Motor is disarmed")
# ============================================
# =======================================================================================|| PROGRAM 2

# PROGRAM 2 (Cek kanan kiri) check sensor first before applying rcover 
'''
note :
Program 2 sementara hanya melakukan cek lidar sensor tanpa terbang cek aman
Lek wes di tes ternyata function sensor jalan & mengeluarkan print yg benar
Maka bisa hilangkan "#" pada sistem yg bekerja (takeoff, rcover, changemode) untuk menjalankan fungsi auto terbang
'''
def program2():
	print ("Running Program 2, Checking lidar distance for indoor mission")
	while True:
			front_distance = get_distance(10)
			right_distance = get_distance(12)  
			left_distance = get_distance(16)  
			bottom_distance = get_distance(0)
			
			if bottom_distance < 0.5:
				print ("Drone is too close to land, trying to operate auto take off")
				#takeoff(0.8) #uncommand untuk real takeoff, Lihat Function takeoff
				
			if front_distance > 1: # If 1 meter untuk maju, ngko onk if 0.7 ge mandek
				print("Drone maju")
				#rcover(0,1420,1500,0,0,0,0,0) 
				
				#Adjust posisi tengah
				if left_distance < 0.7:
					nilaikiri = 150 - (left_distance*10)
					#rcover(1500 + nilaikiri, 1450, 1450,0,0,0,0,0) #rodok nganan
					print("Drone rodok nganan")
				elif right_distance < 0.7:
					nilaikanan = 150 - (right_distance*10)
					#rcover(1500 - nilaikanan,1450,1500,0,0,0,0) #rodok ngiri
					print("Drone rodok ngiri")
				
			elif front_distance < 0.7: #Jika detect tembok/obstacle
				#rcover(1500,1700,1480,0,0,0,0,0)
				print("HOOP Depan onok hambatan")
				time.sleep(0.2) # Stop diluk
				
				#Cek kiri & kanan
				if left_distance < 1:
					print("Kiri tembok! drone bakal muter nganan")
					#rcover(0,1500,1500,1570,0,0,0,0) # Setting yaw to turn right
					
					#cek jarak tembok depan dan kanan sebelum lurus neh
					if front_distance > 1 and right_distance > 1:
						print("Depan Sudah kosong, kanan kosong. Drone maju!")
						#rcover(0,1420,1500,0,0,0,0,0)
						
						if front_distance < 0.7:
							print("Depan ada hambatan, landing!")
							#landing() #Read landing function first or change to changemode(9)
							
					elif front_distance < 1 or rigth_distance < 1:
						print("Depan atau Kanan masih ada hambatan")
						#rcover(0,1500,1500,1570,0,0,0,0)
					else:
						print("Sensor error cannot read collision, operating landing mode")
						#landing()
					
				elif right_distance < 1:
					print("Kanan tembok! drone bakal muter ngiri")
					#rcover(0,1500,1500,1430,0,0,0,0) # Setting yaw to turn left
					
					#cek jarak tembok depan dan kiri sebelum lurus neh
					if front_distance > 1 and left_distance > 1:
						print("Depan Sudah kosong, kiri kosong. Drone maju!")
						#rcover(0,1420,1500,0,0,0,0,0)
						
						if front_distance < 0.7:
							print("Depan ada hambatan, landing!")
							#landing() #Read landing function first
						
					elif front_distance < 1 or left_distance < 1:
						print("Depan atau Kiri masih ada hambatan")
						#rcover(0,1500,1500,1430,0,0,0,0)
						
					else:
						print("Sensor error cannot read collision, operating landing mode")
						#landing()
						#break
					
			time.sleep(3)
	time.sleep(0.1)
	
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
# =======================================================================================|| NOTES
# =======================================================================================|| NOTES



# =======================================================================================
# ===================|| CODE THE DRONE OPERATION BELOW HERE||============================

print("Operating read sensor..")
print("Cek sensor ngeread sek")

read_sensor(5)
print("Checking sensor done..")
time.sleep(3)
program1()
#Check program 2, uji coba baca sensor dulu baru buka code rcover 
program2()
time.sleep(2)
#in case drone belum disarm
arm(0)

# =======================================================================================
# =======================================================================================
