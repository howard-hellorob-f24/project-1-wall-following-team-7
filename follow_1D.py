import time
import numpy as np
from mbot_bridge.api import MBot

# Get distance to wall
def find_fwd_dist(ranges, thetas, window=5):
    """Find the distance to the nearest object in front of the robot.

    Args:
        ranges (list): The ranges from the Lidar scan.
        thetas (list): The angles from the Lidar scan.
        window (int, optional): The window to average ranges over. Defaults to 5.

    Returns:
        float: The distance to the nearest obstacle in front of the robot.
    """
    # Grab the rays near the front of the scan.
    fwd_ranges = np.array(ranges[:window] + ranges[-window:])
    fwd_thetas = np.array(thetas[:window] + thetas[-window:])
    
    # Grab just the positive values.
    valid_idx = (fwd_ranges > 0).nonzero()
    fwd_ranges = fwd_ranges[valid_idx]
    fwd_thetas = fwd_thetas[valid_idx]

    # Compute forward distances.
    fwd_dists = fwd_ranges * np.cos(fwd_thetas)
    
    if len(fwd_dists) == 0:
        return np.nan  # Handle cases where no valid data is available.
    
    return np.mean(fwd_dists)  # Return the mean.

# Initialize a robot object.
robot = MBot()
setpoint = 0.5  # Setpoint is the desired distance from the object (in meters).
forward_speed = 0.2  # Speed to move forward.
backward_speed = -0.2  # Speed to move backward.
threshold = 0.05  # Allowable error in distance (tolerance).

# Allow the robot to initialize.
time.sleep(2)

try:
    # Loop forever.
    while True:
        # Retry reading the LIDAR with a maximum of 3 attempts.
        for attempt in range(3):
            ranges, thetas = robot.read_lidar()
            if ranges and thetas:
                break  # Exit loop if data is successfully retrieved.
            time.sleep(0.1)  # Wait a bit before retrying.
        else:
            print("Failed to get LIDAR data after 3 attempts. Stopping robot.")
            robot.stop()
            break  # Exit the main loop if data couldn't be retrieved.

        dist_to_wall = find_fwd_dist(ranges, thetas)

        if np.isnan(dist_to_wall):
            print("No valid distance data available from LIDAR.")
            robot.stop()  # Stop the robot if no valid data is available.
            continue

        # Bang-Bang controller logic
        if dist_to_wall > setpoint + threshold:
            robot.drive(forward_speed, 0, 0)  # Move forward.
        elif dist_to_wall < setpoint - threshold:
            robot.drive(backward_speed, 0, 0)  # Move backward.
        else:
            robot.drive(0, 0, 0)  # Stop the robot.

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Control + C detected. Stopping the robot...")
    robot.stop()

except Exception as e:
    print(f"An error occurred: {str(e)}")
    robot.stop()
