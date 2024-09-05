# no dronekit
from pymavlink import mavutil
import time

usbser = '/dev/ttyUSB0'
vehicle = mavutil.mavlink_connection(usbser, baud=57600)
vehicle.wait_heartbeat()
print(f"Connected to {usbser}")	
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
# ============================================

changemode(5)
time.sleep(1)	
arm(1)
rcover(1500,1500,1620,1500,0,0,0,0)
time.sleep(2)
rcover(1500,1500,1500,1500,0,0,0,0)
time.sleep(5)
changemode(9)
time.sleep(5)
arm(0)

