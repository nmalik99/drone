from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil  # Needed for command message definitions
import time
import math

# Set up option parsing to get connection string
import argparse

parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',
                help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl

    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
#  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)




def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
 
    Bitmask to indicate which dimensions should be ignored by the vehicle
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that
    none of the setpoint dimensions should be ignored). Mapping:
    bit 1: x,  bit 2: y,  bit 3: z,
    bit 4: vx, bit 5: vy, bit 6: vz,
    bit 7: ax, bit 8: ay, bit 9:
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0,
    0, 0,
    mavutil.mavlink.MAV_FRAME_BODY_NED,
    0b0000111111000111,  # -- BITMASK -> Consider only the velocities
    0, 0, 0,  # -- POSITION
    vx, vy, vz,  # -- VELOCITY
    0, 0, 0,  # -- ACCELERATIONS
    0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def goto_location(waypoint):
    vehicle.simple_goto(waypoint)
    time.sleep(2)
    reached = 0
    while(not reached):
        time.sleep(1)
        a = vehicle.velocity
        if (abs(a[1])< 0.2 and abs(a[2])< 0.2 and abs(a[0])< 0.2):
            reached = 1
    print("Waypoint reached!")


def battery_check():
    if(vehicle.battery.level*-1 < 9.9):
        print("Battery Low. Landing")
        print("Battery Level: %s and Voltage is %s" % (vehicle.battery.level , vehicle.battery.voltage))
        land()
    else:
        print("Battery: %s" % vehicle.battery.level)

def land():
    print("Vehicle in LAND mode")
    vehicle.mode = VehicleMode("LAND")
    while not vehicle.location.global_relative_frame.alt==0:
        if vehicle.location.global_relative_frame.alt < 2:
            set_velocity_body(vehicle,0,0,0.1)
    vehicle.armed = False
    vehicle.close()

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

def delay(sec):
        print("Hover for %s Seconds" % sec)
        time.sleep(sec)



###################################################################################
################################ START CODE #######################################
###################################################################################

############# POINTS ###############
p1 = LocationGlobalRelative(24.830125, 67.097387, 15)


############# TAKE OFF #############
arm_and_takeoff(15)                                                                                                 # Vehicle takeoff
home = vehicle.location.global_frame                                                                                #HOME
print("Reached Target Altitude")
print("Altitude: ", vehicle.location.global_relative_frame.alt)
print("Home Location: %s" % home)
delay(1)
battery_check()

############# POINT 1 ##############
print("Going to Point 1")
goto_location(p1)
print("Reached Point 1")
print("Location: %s" % vehicle.location.global_frame)
delay(1)
battery_check()

############### LAND ################
temp_land()

############ RETURN TO HOME ##########
print("Going to Home")
goto_location(home)
print("Reached Home")
print("Location: %s" % vehicle.location.global_frame)
delay(1)
battery_check()


############# LAND #################    
rtl()                                                                                                               # Land vehicle once mission is over
vehicle.flush()
vehicle.close()
print("Exiting Script")
