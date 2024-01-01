from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14551')
args = parser.parse_args()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

  print("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt )
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print("Reached target altitude")
      break
    time.sleep(1)

def temp_land():
    print("Vehicle in LAND mode")
    vehicle.mode = VehicleMode("LAND")
    while not vehicle.location.global_relative_frame.alt==0:
        if vehicle.location.global_relative_frame.alt < 2:
            set_velocity_body(vehicle,0,0,0.1)
        print ("Vehicle in AUTO mode")
    vehicle.mode = VehicleMode("AUTO")


def rtl():
    print("Vehicle Returning to LAND mode")
    vehicle.mode = VehicleMode("RTL")

# Initialize the takeoff sequence to 5m
arm_and_takeoff(15)

print("Take off complete")

# Hover for 10 seconds
time.sleep(10)

print("Now let's land")
#vehicle.mode = VehicleMode("RTL")
temp_land()
rtl()

# Close vehicle object
vehicle.close()

