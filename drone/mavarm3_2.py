from pymavlink import mavutil
import time 
import threading
import coba_uwaw

# =======================================================================================||
usbser = '/dev/ttyUSB0'
# vehicle = mavutil.mavlink_connection(usbser, baud=57600)
# Run in terminal before code : mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600 --out=udp:127.0.0.1:14550
vehicle = mavutil.mavlink_connection('udp:127.0.0.1:14550')
vehicle.wait_heartbeat()
print(f"Connected to {usbser}")	
print("OPENING : mavarm3_2.py....")
time.sleep(2)

# Checking Heartbeat
print("Heartbeat from (system %u component %u)" % (vehicle.target_system, vehicle.target_component))
time.sleep(2)

# ===========================================
# =======================================================================================|| GET LIDAR DISTANCE
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


# ===========================================(ERROR)
# =======================================================================================|| GLOBAL SENSOR VARIABLES
front_distance = 0.0
right_distance = 0.0
left_distance = 0.0
bottom_distance = 0.0

# ============================================|| UPDATE SENSOR DENGAN THREAD
def update_sensors():
    global front_distance, left_distance, right_distance, bottom_distance
    while True:
        front_distance = get_distance(10)
        left_distance = get_distance(16)
        right_distance = get_distance(12)
        bottom_distance = get_distance(0)
        time.sleep(0.1)  # Delay to reduce CPU usage

# ============================================
# =======================================================================================|| CHANGE MODE
def changemode(modeval):
	modeflag = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
	vehicle.mav.set_mode_send(
		vehicle.target_system,
		modeflag,
		modeval
	)

	mode_values = {
		0: "stabilize",
		5: "loiter",
		9: "land"
	}
	mode_name = mode_values.get(modeval, "mode unknown")
	print(f"Mode is : {mode_name}")	

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
    changemode(5)
    bottom_distance = get_distance(0)
    print(f"TAKE OFF TO ALTITUDE {alt} m")
    while bottom_distance < alt:
        print (f"Drone Altitude: {bottom_distance:.2f} meters")
        rcover(1500, 1500, 1620, 0, 0, 0, 0, 0)
        bottom_distance = get_distance(0)
        if bottom_distance >= alt:
            print(f"Target height reached{alt}. Hovering...")
            rcover(1500, 1500, 1450, 0, 0, 0, 0, 0)
            return print("Take Off step already done")
			
# ============================================
# =======================================================================================|| READ SENSOR
def read_sensor(perulangan):
    i = 1
    while i <= perulangan:
        front_distance = get_distance(10)
        bottom_distance = get_distance(0)
        left_distance = get_distance(16)
        right_distance = get_distance(12)
        
        print("Depan : ", end="")
        print(front_distance)
        print("Kanan : ", end="")
        print(right_distance)
        print("Kiri : ", end="")
        print(left_distance)
        print("bottom : ", end="")
        print(bottom_distance)
        time.sleep(0.3)
        i +=1

# ============================================
# =======================================================================================|| ADJUST POSISI
'''
def adjust():
    right_distance = get_distance(12)
    left_distance = get_distance(16)
    while left_distance < 0.7:
        nilaikiri = 150 - (left_distance*10)
        rcover(1500 + nilaikiri, 1450, 1450,0,0,0,0,0) #rodok nganan
        print("Drone rodok nganan")
        left_distance = get_distance(16)
        
    while right_distance < 0.7:
        nilaikanan = 150 - (right_distance*10)
        rcover(1500 - nilaikanan,1450,1500,0,0,0,0) #rodok ngiri
        print("Drone rodok ngiri")
        right_distance = get_distance(12)
'''
def adjust():
    while True:
        right_distance = get_distance(12)
        left_distance = get_distance(16)
        
        if left_distance < 0.7:
            nilaikiri = 150 - (left_distance*10)
            rcover(1500 + nilaikiri, 1450, 1450, 0, 0, 0, 0, 0)  # rodok nganan
            print("Drone rodok nganan")
            
        elif right_distance < 0.7:
            nilaikanan = 150 - (right_distance*10)
            rcover(1500 - nilaikanan, 1450, 1450, 0, 0, 0, 0, 0)  # rodok ngiri
            print("Drone rodok ngiri")
        else:
            print("Drone stabil")

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
		ch8		# Obstacle Avoidance
		)
			
# ============================================
# =======================================================================================|| MAJU
def maju():
    front_distance = get_distance(10)
    while front_distance >= 1.2:
        front_distance = get_distance(10)
        left_distance = get_distance(16)
        right_distance = get_distance(12)
        print("Drone maju")
        rcover(1500,1420,1500,0,0,0,0,0) 
        adjust()
            
    else:
        print("Depan ada hambatan, Drone Hovering!")
        rcover(1500, 1500, 1520, 0, 0, 0, 0, 0)
        adjust()
        front_distance = get_distance(10)
        left_distance = get_distance(16)
        right_distance = get_distance(12)
  
# ============================================
# =======================================================================================|| BELOK
def belok():
    while left_distance < 1 and front_distance <= 2 :
        print("Kanan ada hambatan, belok kiri!")
        rcover(1500,1500,1500,1430,0,0,0,0)
        front_distance = get_distance(10)
        left_distance = get_distance(16)
        while front_distance > 2:
            rcover(1500,1420,1500,0,0,0,0,0) #Maju
            front_distance = get_distance(10)
    
    while right_distance < 1 and front_distance <= 2:
        print("Kiri ada hambatan, belok kanan!")
        rcover(1500,1500,1500,1580,0,0,0,0)
        front_distance = get_distance(10)
        right_distance = get_distance(16)
        while front_distance > 2:
            rcover(1500,1420,1500,0,0,0,0,0) #Maju
            front_distance = get_distance(10)
    
    else :
        print("Can't detect left and right obstacle, Hovering enabled!")
        rcover(1500,1500,1520,0,0,0,0,0)
 
# ============================================
# =======================================================================================|| PROGRAM 
# ===== MUST DO CONDITION ::
        # 1 pastikan drone ARMING, MODE LOITER, dan TAKEOFF sebelum lanjut bawah
        # 2 pastikan semua LIDAR dan kamera terbaca
        # 3 if drone detect DEPAN TEMBOK maka dia HARUS HOVER
        # 4 drone harus cek lidar kanan dan kiri, pakai 'OR' sebagai kondisi
        # 5 jika DEPAN ADA TEMBOK #3 dan KANAN 'OR' KIRI ADA TEMBOK maka drone HARUS YAW sesuai kondisi SAMPAI DEPAN KOSONG
        # 6 jika DEPAN KOSONG dan KANAN 'OR' KIRI ADA TEMBOK maka drone HARUS MAJU
        # 7 jika DEPAN ADA TEMBOK tapi KANAN 'OR' KIRI KOSONG maka drone HARUS NAIK sampai depan kosong (SITUASI GATE)
        # 8 setelah #7 maka drone HARUS MAJU
        
def leron():
    arm(1) #1
    print("DRONE IS ARMING!!!")
    read_sensor(2) #2
    takeoff(1) #1
    
    sensor_thread = threading.Thread(target=update_sensors)
    sensor_thread.daemon = True
    sensor_thread.start()

    webcam = threading.Thread(target=coba_uwaw.main)
    webcam.daemon = True
    webcam.start()
    
    adjust = threading.Thread(target=adjust)
    adjust.daemon = True
    adjust.start()
    
    while bottom_distance > 0.4:
        maju() #3 hover in maju function
        
        if front_distance <= 1 and left_distance > 2 and right_distance > 2:
            while front_distance <=1 :
                rcover(1500, 1500, 1620, 0, 0, 0, 0, 0)
                front_distance = get_distance(10)
                if front_distance > 2:
                    maju()
                    time.sleep(5)
                    changemode(9)
                    
        elif front_distance <= 1 and (left_distance < 1 or right_distance < 1):
            belok() 
            
    else:
        print("Drone in very low altitude!!!")
        changemode(9) #Landing mode
        arm(0)


# JUST TAKE OFF LANDING
def JTL():
    changemode(5)
    read_sensor(5)
    arm(1)
    takeoff(1) #Jarak meter ketinggian
    rcover(1500,1420,1500,0,0,0,0,0)
    time.sleep(5)
    #test yaw
    rcover(1500,1500,1520,1580,0,0,0,0)
    time.sleep(3)
    changemode(9)
    arm(0)
    
    
			
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
# front_distance = get_distance(10)
# left_distance = get_distance(16)
# right_distance = get_distance(12)
# bottom_distance = get_distance(0)
# =======================================================================================|| NOTES
# =======================================================================================|| NOTES



# =======================================================================================
# ===================|| CODE THE DRONE OPERATION BELOW HERE||============================
def main():
    print("TRY CODE AUTO DRONE")
    time.sleep(2)
    print("=============================================================================", end="")
    print("=============================================================================")

    # leron()
    JTL()

    print("PROGRAM FINISHED!! PLEASE MANUALLY CONTROLL OVER THE DRONE")
    
# =======================================================================================|| CODE RUN IN MAIN
if __name__ == "__main__":
    main()
# ==========================================================================================================
# ==========================================================================================================
