import dronekit
from pymavlink import mavutil
import smbus
import time
import math

i = 0

print("program wilayah V0.01")
#Connect ke telemetri
connection_string = 'udp:192.168.1.253:14550'	

#Koneksi vehicle
vehicle = dronekit.connect(connection_string, baud=1500000, wait_ready = True, timeout = 150)
print("Connected to ")	
print(connection_string)

#=================================================

#Vehicle mode
vehicle.mode = dronekit.VehicleMode('LOITER')
print("mode is LOITER")

while not vehicle.armed:
	time.sleep(2)
	print("Drone is DISARMED")
	
#=================================================
print("Awas sikil awas ndasnya drone arming")
time.sleep(1)
print("take off")
while True:
	vehicle.channels.overrides[3] = 1634
	vehicle.channels.overrides[1] = 1500
	vehicle.channels.overrides[2] = 1500
	if vehicle.rangefinder.distance >=0.8:
		print('ketinggian sudah {0:.2f}m'.format(vehicle.rangefinder.distance))
		break
	else:
		print("naik ketinggian: {0:.2f}m".format(vehicle.rangefinder.distance))
	time.sleep(0.25)

while i<1:
	print('hovering rn on {0:.2f}m'.format(vehicle.rangefinder.distance))
	vehicle.channels.overrides[3] = 1465 # throttle
	vehicle.channels.overrides[1] = 1500 # kanan+ kiri-
	vehicle.channels.overrides[2] = 1500 # maju- mundur+
	vehicle.channels.overrides[8] = 1900
	time.sleep(0.1)
	#vehicle.channels.overrides[8] = 1100
	time.sleep(1)
	i=i+1

#maju
print('Move forward rn on {0:.2f}m'.format(vehicle.rangefinder.distance))
vehicle.channels.overrides[3] = 1450
vehicle.channels.overrides[1] = 1500
vehicle.channels.overrides[2] = 1420
time.sleep(3)

print("Landing...")
vehicle.mode = dronekit.VehicleMode('LAND')
if vehicle.rangefinder.distance < 0.15:
	print("disarming...")
	vehicle.armed = False 

