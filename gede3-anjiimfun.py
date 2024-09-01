from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil

# Connect to the Vehicle
# Make sure to replace '/dev/ttyUSB0' with the correct USB port
vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)

def arm_and_takeoff(target_altitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(5)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# 1. Auto takeoff
arm_and_takeoff(1.4)  # Take off to 1 meter

# 2. Hold position for 5 seconds
print("Hold position for 5 seconds")
time.sleep(3)

# 3. Move forward (to the north) for 3 seconds
print("Moving forward to the north")
duration = 30
north_velocity = 0.2  # m/s
east_velocity = 0
down_velocity = 0

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111,
        0, 0, 0, velocity_x, velocity_y, velocity_z, 0, 0, 0, 0, 0)
    
    for _ in range(0, duration * 10):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

send_ned_velocity(north_velocity, east_velocity, down_velocity, duration)

# 4. Stop and hold steady position for 5 seconds
print("Stopping and holding position for 5 seconds")
send_ned_velocity(0, 0, 0, 1)  # Send zero velocity to stop
time.sleep(5)

# 5. Move forward (to the east) for 3 seconds
print("Moving forward to the east")
duration = 27
north_velocity = 0  # m/s
east_velocity = 0.2
down_velocity = 0

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111,
        0, 0, 0, velocity_x, velocity_y, velocity_z, 0, 0, 0, 0, 0)
    
    for _ in range(0, duration * 10):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

send_ned_velocity(north_velocity, east_velocity, down_velocity, duration)

# 4. Stop and hold steady position for 5 seconds
print("Stopping and holding position for 5 seconds")
send_ned_velocity(0, 0, 0, 1)  # Send zero velocity to stop
time.sleep(5)

# exit gate naik
print("Moving forward to the east")
duration = 5
north_velocity = 0  # m/s
east_velocity = 0
down_velocity = -0.2

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111,
        0, 0, 0, velocity_x, velocity_y, velocity_z, 0, 0, 0, 0, 0)
    
    for _ in range(0, duration * 10):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

send_ned_velocity(north_velocity, east_velocity, down_velocity, duration)

# 4. Stop and hold steady position for 5 seconds
print("Stopping and holding position for 5 seconds")
send_ned_velocity(0, 0, 0, 1)  # Send zero velocity to stop
time.sleep(5)

# 5. Move forward (to the east) for 3 seconds
print("Moving forward to the east")
duration = 25
north_velocity = 0  # m/s
east_velocity = 0.2
down_velocity = 0

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111,
        0, 0, 0, velocity_x, velocity_y, velocity_z, 0, 0, 0, 0, 0)
    
    for _ in range(0, duration * 10):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

send_ned_velocity(north_velocity, east_velocity, down_velocity, duration)

# 4. Stop and hold steady position for 5 seconds
print("Stopping and holding position for 5 seconds")
send_ned_velocity(0, 0, 0, 1)  # Send zero velocity to stop
time.sleep(5)

# 8. Fun yaw for showing off
def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to set the yaw of the drone.

    :param heading: Desired yaw angle in degrees (0 is North, 90 is East, etc.)
    :param relative: If True, yaw is relative to current heading. If False, yaw is absolute.
    """
    is_relative = 1 if relative else 0
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1: yaw angle in degrees
        0,  # param 2: yaw speed (0 for default)
        1,  # param 3: direction -1 ccw, 1 cw
        is_relative,  # param 4: relative (1) or absolute (0)
        0, 0, 0)  # param 5-7 (unused)
    vehicle.send_mavlink(msg)

# Assuming you have already connected to the vehicle
# Yaw to 90 degrees to the right (clockwise)
condition_yaw(720, relative=True)
time.sleep(5)  # Rotate for 3 seconds

# 9. Auto land
print("Landing")
vehicle.mode = VehicleMode("LAND")

while vehicle.armed:
    print(" Waiting for disarming...")
    time.sleep(5)

print("Landed and disarmed")

# Close vehicle object before exiting script
vehicle.close()
