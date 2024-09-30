import time
import numpy as np
from mbot_bridge.api import MBot

def find_min_dist(ranges, thetas):
    """Finds the length and angle of the minimum ray in the scan.

    Make sure you ignore any rays with length 0! Those are invalid.

    Args:
        ranges (list): The length of each ray in the Lidar scan.
        thetas (list): The angle of each ray in the Lidar scan.

    Returns:
        tuple: The length and angle of the shortest ray in the Lidar scan.
    """
    min_dist, min_angle = None, None

    # Convert ranges to a numpy array for easy filtering.
    valid_ranges = np.array(ranges)
    valid_thetas = np.array(thetas)

    # Ignore rays with zero range (invalid rays).
    valid_idx = (valid_ranges > 0).nonzero()[0]  # Get indices where range > 0
    if len(valid_idx) == 0:
        return None, None  # Return None if no valid data is available.

    # Get the minimum valid range and corresponding angle.
    min_idx = valid_ranges[valid_idx].argmin()  # Index of minimum range
    min_dist = valid_ranges[valid_idx][min_idx]  # Minimum distance
    min_angle = valid_thetas[valid_idx][min_idx]  # Corresponding angle

    return min_dist, min_angle


def cross_product(v1, v2):
    """Compute the Cross Product between two vectors.

    Args:
        v1 (list): First vector of length 3.
        v2 (list): Second vector of length 3.

    Returns:
        list: The result of the cross product operation.
    """
    res = np.zeros(3)
    res[0] = v1[1] * v2[2] - v1[2] * v2[1]
    res[1] = v1[2] * v2[0] - v1[0] * v2[2]
    res[2] = v1[0] * v2[1] - v1[1] * v2[0]
    return res


# Initialize the robot object
robot = MBot()
setpoint = 0.5  # Desired distance from the wall (in meters).
forward_speed = 0.2  # Speed to move forward.
backward_speed = -0.2  # Speed to move backward.
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
            time.sleep(0.1)  # Wait before retrying
        else:
            print("Failed to get LIDAR data after 3 attempts. Stopping robot.")
            robot.stop()
            break  # Exit the main loop if data couldn't be retrieved.

        # Find the minimum distance and its corresponding angle
        min_dist, min_angle = find_min_dist(ranges, thetas)

        if min_dist is None or min_angle is None:
            print("No valid distance data available from LIDAR.")
            robot.stop()  # Stop the robot if no valid data is available.
            continue

        print(f"Min dist: {min_dist:.2f} m at angle {min_angle:.2f} radians")

        # Bang-Bang control logic
        if min_dist > setpoint + threshold:
            robot.drive(forward_speed, 0, 0)  # Move forward
        elif min_dist < setpoint - threshold:
            robot.drive(backward_speed, 0, 0)  # Move backward
        else:
            robot.drive(0, 0, 0)  # Stop the robot

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Control + C detected. Stopping the robot...")
    robot.stop()

except Exception as e:
    print(f"An error occurred: {str(e)}")
    robot.stop()
