from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description="Commands vehicle using vehicle.simple_goto.")
parser.add_argument("--connect",
                    help="First vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect

# Connect to vehicle
vehicle = connect(connection_string, wait_ready=False)

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
