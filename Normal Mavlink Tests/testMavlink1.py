from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time

aTargetAltitude = 5


# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description="Commands vehicle using vehicle.simple_goto.")
parser.add_argument("--connect",
                    help="First vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect


'''
print("Start simulator (SITL)")
import dronekit_sitl
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()
'''

''' -------------------------------------------------------------------
                          Connect and Takeoff
---------------------------------------------------------------------'''

# Connect to the Vehicle (in this case a simulator running the same computer)
#vehicle = connect(connection_string, wait_ready=True)
vehicle = connect(connection_string, wait_ready=False)

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
vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

# Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
#  after Vehicle.simple_takeoff will execute immediately).
while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
        print("Reached target altitude")
        break
    time.sleep(1)


''' ------------------------------------------------------------------
                            MAVLINK Message
--------------------------------------------------------------------'''

msg = vehicle.message_factory.set_position_target_global_int_encode(
    0,       # time_boot_ms (not used)
    0, 0,    # target system, target component
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
    0b0000111111111000, # type_mask (only speeds enabled)
    -35.361883*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
    149.165116*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
    20, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
    0, # X velocity in NED frame in m/s
    0, # Y velocity in NED frame in m/s
    0, # Z velocity in NED frame in m/s
    0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
    0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

# send command to vehicle
vehicle.send_mavlink(msg)
print("Mavlink sent")
time.sleep(100)


''' ------------------------------------------------------------------
                           LAND AND CLOSE SITL
--------------------------------------------------------------------'''

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")



#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")
