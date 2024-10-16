import time
import numpy as np
from mbot_bridge.api import MBot

def find_min_dist(ranges, thetas):
    """Finds the length and angle of the minimum ray in the scan."""
    min_dist, min_angle = None, None

    # Convert ranges to a numpy array for easy filtering.
    valid_ranges = np.array(ranges)
    valid_thetas = np.array(thetas)

    # Ignore rays with zero range (invalid rays).
    valid_idx = (valid_ranges > 0).nonzero()[0]
    if len(valid_idx) == 0:
        return None, None

    # Get the minimum valid range and corresponding angle.
    min_idx = valid_ranges[valid_idx].argmin()
    min_dist = valid_ranges[valid_idx][min_idx]
    min_angle = valid_thetas[valid_idx][min_idx]

    return min_dist, min_angle

def cross_product(v1, v2):
    """Compute the Cross Product between two vectors."""
    res = np.zeros(3)
    res[0] = v1[1] * v2[2] - v1[2] * v2[1]
    res[1] = v1[2] * v2[0] - v1[0] * v2[2]
    res[2] = v1[0] * v2[1] - v1[1] * v2[0]
    return res

def compute_velocity_vector(angle_to_wall, drive_velocity):
    """Compute the forward velocity vector perpendicular to the wall."""
    # Create the v_to_wall vector based on the angle.
    v_to_wall = [np.cos(angle_to_wall), np.sin(angle_to_wall), 0]
    
    # The second vector is the "up" vector in 3D.
    v_up = [0, 0, 1]
    
    # Compute the cross product to get the forward velocity vector.
    v_forward = cross_product(v_to_wall, v_up)
    
    # Normalize the forward vector to have magnitude 1 and multiply by desired drive velocity.
    v_forward = (v_forward / np.linalg.norm(v_forward)) * drive_velocity
    
    return v_forward[:2]  # Return the x, y components (ignore z).

# Initialize the robot object
robot = MBot()
setpoint = 0.5  # Desired distance from the wall (in meters).
forward_speed = 0.2  # Speed to move forward.
threshold = 0.05  # Allowable error in distance (tolerance).

# Allow the robot to initialize
time.sleep(2)

try:
    while True:
        # Read the latest lidar scan with retries
        for attempt in range(3):
            ranges, thetas = robot.read_lidar()
            if ranges and thetas:
                break  # Exit the loop if data is successfully retrieved.
            time.sleep(0.1)
        else:
            print("Failed to get LIDAR data after 3 attempts. Stopping robot.")
            robot.stop()
            break

        # Find the minimum distance and its corresponding angle
        min_dist, min_angle = find_min_dist(ranges, thetas)

        if min_dist is None or min_angle is None:
            print("No valid distance data available from LIDAR.")
            robot.stop()
            continue

        print(f"Min dist: {min_dist:.2f} m at angle {min_angle:.2f} radians")

        # Compute the forward velocity vector (perpendicular to the wall)
        v_forward = compute_velocity_vector(min_angle, forward_speed)
        
        # Compute the correction to maintain the setpoint distance
        error = min_dist - setpoint
        correction_speed = 0
        if error > threshold:
            correction_speed = forward_speed  # Move forward
        elif error < -threshold:
            correction_speed = -forward_speed  # Move backward
        
        # Combine the forward velocity and correction for final control
        final_velocity_x = v_forward[0] + correction_speed
        final_velocity_y = v_forward[1]

        # Send the control command to the robot (moving along the wall)
        robot.drive(final_velocity_x, final_velocity_y, 0)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Control + C detected. Stopping the robot...")
    robot.stop()

except Exception as e:
    print(f"An error occurred: {str(e)}")
    robot.stop()
