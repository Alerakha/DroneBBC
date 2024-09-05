import dronekit
from pymavlink import mavutil
import smbus
import time
import math

i = 0

print("program wilayah V0.01")
#Connect ke telemetri
connection_string = '/dev/ttyUSB0'	

#Koneksi vehicle
vehicle = dronekit.connect(connection_string, baud=57600, wait_ready = True, timeout = 150)
print("Connected to ")	
print(connection_string)
'''
#=================================================
bus = smbus.SMBus(1)
def scan_i2c_bus():
    print("Scanning I2C bus...")
    for address in range(0x03, 0x77):
        try:
            bus.write_quick(address)
            print(f"Found device at address {hex(address)}")
        except IOError:
            pass

scan_i2c_bus()
time.sleep(2)
'''
'''
lidar_addresses = {
    "front": 0x11,  # Replace with the actual address for the front sensor
    "left": 0x09,   # Replace with the actual address for the left sensor
    "right": 0x12,  # Replace with the actual address for the right sensor
    "bottom": 0x8,  # Replace with the actual address for the right sensor
}

def read_lidar(address):
    try:
        data = bus.read_i2c_block_data(address, 0x00, 2)
        distance = data[0] | (data[1] << 8)
        distance /= 1
        return distance
    except Exception as e:
        print(f"Error reading from sensor at address {hex(address)}: {e}")
        return None
'''
#=================================================

#Vehicle mode
vehicle.mode = dronekit.VehicleMode('LOITER')
print("mode is LOITER")

while not vehicle.armed:
	time.sleep(2)
	print("Drone is DISARMED")
	'''
	for position, address in lidar_addresses.items():
		distance = read_lidar(address)
		if distance is not None:
			print(f"{position.capitalize()} Lidar distance: {distance:.2f} meters")
			if position == "front" and distance < 0.8:
				print("Detect tembok depan!")
			elif position == "right" and distance < 0.8:
				print("Detect tembok kanan!")
			elif position == "left" and distance <0.8:
				print("Detect tembok kiri!")
			else:
				print(f"Error reading {position} lidar sensor")
			time.sleep(1)
	'''

#Coba initialize I2C
#Coba detect
	
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

