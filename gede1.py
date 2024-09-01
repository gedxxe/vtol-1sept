from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
from pymavlink import mavutil

# Connect to the vehicle
vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)  # Adjust as per your connection

def arm_and_takeoff(target_altitude):
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(4)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def set_groundspeed(speed):
    """
    Set the ground speed of the drone.
    """
    vehicle.groundspeed = speed
    print(f"Groundspeed set to {speed} m/s")

def move_to_position(target_location):
    """
    Move the drone to a specific position using the ground speed.
    """
    vehicle.simple_goto(target_location)
    # Wait until the vehicle reaches the target location
    while True:
        distance = get_distance_metres(vehicle.location.global_relative_frame, target_location)
        if distance < 0.5:
            print("Reached target location")
            break
        time.sleep(1)

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobalRelative object containing the latitude/longitude `dNorth` and `dEast` metres from the `original_location`.
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobalRelative objects.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlon = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlon*dlon)) * 1.113195e5

def condition_yaw(heading, clockwise=True, relative=False):
    """
    Yaw the drone to a specific heading or yaw right.
    """
    is_relative = 1 if relative else 0
    direction = 1 if clockwise else -1
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
        0,       # confirmation
        heading, # param 1: Yaw angle
        0,       # param 2: Yaw speed
        direction, # param 3: Direction -1 ccw, 1 cw
        is_relative, # param 4: Relative offset (1) or absolute angle (0)
        0, 0, 0) # param 5-7 (not used)
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Mission starts here
arm_and_takeoff(1)  # Take off to 3 meters

time.sleep(3)  # Hold position for 3 seconds

# Set ground speed to 0.25 m/s
set_groundspeed(0.25)

# Calculate the target position 6.25 meters forward (0.25 m/s * 25 s)
current_location = vehicle.location.global_relative_frame
target_location_forward = get_location_metres(current_location, 6.25, 0)
move_to_position(target_location_forward)

time.sleep(5)  # Hold position for 5 seconds

# Yaw the drone to the right by 90 degrees
condition_yaw(90, clockwise=True, relative=True)
time.sleep(2)  # Wait for yaw completion

time.sleep(5)  # Hold position for 5 seconds

# Calculate the target position 6.25 meters to the right (relative to the starting point)
target_location_right = get_location_metres(target_location_forward, 0, 6.25)
move_to_position(target_location_right)

time.sleep(5)  # Hold position for 5 seconds

print("Landing")
vehicle.mode = VehicleMode("LAND")

while vehicle.location.global_relative_frame.alt > 0:
    print(f"Altitude: {vehicle.location.global_relative_frame.alt}")
    time.sleep(1)

print("Mission completed")
vehicle.close()
