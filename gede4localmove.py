from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

# Connect to the Vehicle
# Replace '/dev/ttyUSB0' with the correct USB port
vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)

def arm_and_takeoff(target_altitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(10)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(2)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def send_velocity_body(vx, vy, vz, duration):
    """
    Move the drone with respect to its current orientation (relative movement).
    
    :param vx: Velocity in the X direction (forward/backward) in m/s
    :param vy: Velocity in the Y direction (right/left) in m/s
    :param vz: Velocity in the Z direction (up/down) in m/s
    :param duration: Duration to send this command in seconds
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, 
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # Use body frame (relative to drone's orientation)
        0b0000111111000111,  # Consider only velocity components
        0, 0, 0,  # Position
        vx, vy, vz,  # Velocity in m/s
        0, 0, 0,  # Acceleration
        0, 0)  # Yaw, yaw rate
    
    for _ in range(0, int(duration * 10)):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

# 1. Auto takeoff
arm_and_takeoff(3)  # Take off to 10 meters

# 2. Hold position for 5 seconds
print("Holding position for 5 seconds")
time.sleep(5)

# Move forward (relative to current heading) for 3 seconds
print("Moving forward relative to current heading")
send_velocity_body(2.5, 0, 0, 3)  # Move forward at 2.5 m/s for 3 seconds

# Hold steady position for 1 seconds
print("Holding position for 1 seconds")
send_velocity_body(0, 0, 0, 1)  # Send zero velocity to stop
time.sleep(1)

# Move kanan (relative to current heading) for 3 seconds
print("Moving right relative to current heading")
send_velocity_body(0, 2.5, 0, 3)  # Move kanan at 2.5 m/s for 3 seconds

# Hold steady position for 1 seconds
print("Holding position for 1 seconds")
send_velocity_body(0, 0, 0, 1)  # Send zero velocity to stop
time.sleep(1)

# Move backward (relative to current heading) for 3 seconds
print("Moving backward relative to current heading")
send_velocity_body(-2.5, 0, 0, 3)  # Move forward at 2.5 m/s for 3 seconds

# Hold steady position for 1 seconds
print("Holding position for 1 seconds")
send_velocity_body(0, 0, 0, 1)  # Send zero velocity to stop
time.sleep(1)

# Move kiri (relative to current heading) for 3 seconds
print("Moving left relative to current heading")
send_velocity_body(0, -2.5, 0, 3)  # Move left at 2.5 m/s for 3 seconds

# Hold steady position for 1 seconds
print("Holding position for 1 seconds")
send_velocity_body(0, 0, 0, 1)  # Send zero velocity to stop
time.sleep(1)

# Move forward (relative to current heading) for 3 seconds
print("Moving forward relative to current heading")
send_velocity_body(2.5, 0, 0, 3)  # Move forward at 2.5 m/s for 3 seconds

# Hold steady position for 1 seconds
print("Holding position for 1 seconds")
send_velocity_body(0, 0, 0, 1)  # Send zero velocity to stop
time.sleep(5)



# 3. Auto land
print("Landing")
vehicle.mode = VehicleMode("LAND")

while vehicle.armed:
    print(" Waiting for disarming...")
    time.sleep(1)

print("Landed and disarmed")

# Close vehicle object before exiting script
vehicle.close()
