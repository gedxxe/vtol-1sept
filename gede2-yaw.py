from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
from pymavlink import mavutil

# Connect to the vehicle
vehicle = connect('/dev/ttyUSB0', baud=57600, wait_ready=True)  # Adjust as per your connection

def arm_and_takeoff(target_altitude):
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def move_forward(duration, speed):
    """
    Move the drone forward based on local position.
    """
    print(f"Moving forward for {duration} seconds at {speed} m/s")
    velocity_x = speed
    velocity_y = 0
    velocity_z = 0

    send_velocity(velocity_x, velocity_y, velocity_z)
    time.sleep(duration)
    send_velocity(0, 0, 0)  # Stop

def move_right(duration, speed):
    """
    Move the drone to the right relative to its current orientation.
    """
    print(f"Moving right for {duration} seconds at {speed} m/s")
    velocity_x = 0
    velocity_y = speed
    velocity_z = 0

    send_velocity(velocity_x, velocity_y, velocity_z)
    time.sleep(duration)
    send_velocity(0, 0, 0)  # Stop

def send_velocity(velocity_x, velocity_y, velocity_z):
    """
    Send velocity command to the drone.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not used)
        0, 0)    # yaw, yaw_rate (not used)
    vehicle.send_mavlink(msg)
    vehicle.flush()

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
arm_and_takeoff(3)  # Take off to 3 meters

time.sleep(3)  # Hold position for 3 seconds

move_forward(25, 0.25)  # Move forward for 25 seconds at 0.25 m/s

time.sleep(5)  # Hold position for 5 seconds

condition_yaw(90, clockwise=True, relative=True)  # Yaw to the right by 90 degrees
time.sleep(2)  # Wait for yaw completion

time.sleep(5)  # Hold position for 5 seconds

move_right(25, 0.25)  # Move to the right for 25 seconds at 0.25 m/s

time.sleep(5)  # Hold position for 5 seconds

print("Landing")
vehicle.mode = VehicleMode("LAND")

while vehicle.location.global_relative_frame.alt > 0:
    print(f"Altitude: {vehicle.location.global_relative_frame.alt}")
    time.sleep(1)

print("Mission completed")
vehicle.close()
