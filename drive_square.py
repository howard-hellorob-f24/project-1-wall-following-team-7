import time
from mbot_bridge.api import MBot

robot = MBot()

"""
TODO: (P1.1) Write code to make the robot drive in a square. Then,
modify your code so that the robot drives in a square 3 times.

HINT: A function to send velocity commands to the robot is provided. To
use it, use the following code:

    robot.drive(vx, vy, wz);

Replace vx, vy, and wz with the velocity in the x direction (vx), y
direction (vy), and the angular velocity (wz). You can also use this code:

    time.sleep(secs);

to sleep for "secs" seconds (replace with desired number of seconds).
"""

# robot.stop()
def drive_square():
    for _ in range(4):
        # Move forward 1 meter
        speed = 0.5  # Set a slower speed, e.g., 0.1 m/s
        distance = 1.3  # Target distance of 1 meter
        time_to_move = distance / speed  # Calculate time to move 1 meter
        
        robot.drive(speed, 0, 0)  # Move forward at the specified speed (e.g., 0.1 m/s)
        time.sleep(time_to_move)  # Sleep for the time needed to move 1 meter (e.g., 10 seconds)
        
        # Stop before turning
        robot.drive(0, 0, 0)
        time.sleep(1)

        # Turn 90 degrees (keep this part unchanged)
        robot.drive(0, 0, 1.57)  # Turn with angular velocity of 1.57 rad/s (90 degrees in 1 second)
        time.sleep(1)  # Sleep for 1 second to complete the turn

        # Stop after turn
        robot.drive(0, 0, 0)
        time.sleep(1)

try:
    for _ in range(1):  # Repeat the square driving 3 times
        drive_square()

except KeyboardInterrupt:
    print("Program interrupted by user. Stopping the robot...")
    robot.stop()  # Stop the robot when Ctrl + C is pressed

except Exception as e:
    print(f"Error: {e}")
    # Catch any exception, including the user quitting, and stop the robot.
    robot.stop()
