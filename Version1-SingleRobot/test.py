import time
import argparse
import math
from uwb_reader import UWBReader
from Zumo import Zumo
from navigation import calculate_turn_angle, is_within_target, normalize_angle

# constants from calibration
DISTANCE_TO_ENCODER_DELTA = 10176
TURN_ANGLE_TO_ENCODER_DELTA = 432.2648

# Constants for movement
MOTOR_SPEED_FORWARD = 400
MOTOR_SPEED_TURN = 350
ANGLE_TOLERANCE = math.radians(10)
TARGET_TOLERANCE = 0.1 #lower value is harder to execute

# Proportional constant for straight-line corrections
Kp = 0.2


def turn_in_place(zumo, motor_speed, desired_turn_angle):
    """
    Turn the robot in place using encoder-based logic.
    """
    assert(motor_speed > 0 and motor_speed <= 400)

    # Reset encoders
    zumo.reset_encoders()
    right_count, left_count = 0, 0

    # Compute the desired encoder count for the turn
    desired_right_count = abs(desired_turn_angle) * TURN_ANGLE_TO_ENCODER_DELTA

    if desired_turn_angle > 0:
        # Turn right
        zumo.send_speeds(-motor_speed, motor_speed)
        while right_count < desired_right_count:
            left_count, right_count = zumo.get_encoders()
    else:
        # Turn left
        zumo.send_speeds(motor_speed, -motor_speed)
        while left_count < desired_right_count:
            left_count, right_count = zumo.get_encoders()

    # Stop the robot after turning
    zumo.send_speeds(0, 0)

def move_forward(zumo, distance, base_speed=MOTOR_SPEED_FORWARD):
    """
    Move the robot forward with proportional control to keep it straight.
    """
    # Reset encoders
    zumo.reset_encoders()
    left_count, right_count = 0, 0

    # Compute the desired encoder count for the distance
    desired_count = distance * DISTANCE_TO_ENCODER_DELTA

    while left_count < desired_count or right_count < desired_count:
        # Get current encoder counts
        left_count, right_count = zumo.get_encoders()

        # Calculate the error
        error = left_count - right_count

        # Adjust motor speeds using proportional control
        left_speed = base_speed - (Kp * error)
        right_speed = base_speed + (Kp * error)

        # Ensure motor speeds are within valid range
        left_speed = max(0, min(400, left_speed))
        right_speed = max(0, min(400, right_speed))

        # Send adjusted speeds to the motors
        zumo.send_speeds(left_speed, right_speed)

    # Stop the robot after moving
    zumo.send_speeds(0, 0)


def calculate_heading(current_pos, previous_pos):
    """
    Calculate the robot's heading based on the change in position.
    """
    if previous_pos == (None, None):
        return 0.0  # If we truly have no previous info, return a default heading.

    dx = current_pos[0] - previous_pos[0]
    dy = current_pos[1] - previous_pos[1]
    return math.atan2(dy, dx)

def main(target_x, target_y):
    # Target position (x, y)
    TARGET_POS = (target_x, target_y)

    # Initialize UWB reader
    uwb_reader = UWBReader()
    if not uwb_reader.start():
        return

    # Initialize Zumo robot
    zumo = Zumo()

    # --------------------
    # 1) Get an initial valid position and store it as previous_pos
    previous_pos = (None, None)
    while True:
        pos = uwb_reader.get_latest_position()
        if pos != (None, None):
            previous_pos = pos
            print(f"Initial position acquired: {previous_pos}")
            break
        time.sleep(0.1)

    # 2) Move forward a bit to establish a heading.
    initial_move_distance = 0.4  
    print(f"Performing an initial move of {initial_move_distance} m to establish heading.")
    move_forward(zumo, initial_move_distance)

    # Now read a new position and compute heading
    time.sleep(0.2)  # small pause to get a fresh UWB reading
    current_pos = uwb_reader.get_latest_position()
    if current_pos == (None, None):
        print("Could not get position after initial move.")
        uwb_reader.stop()
        return

    # 3) Compute initial heading from the two different positions
    zumo.heading = calculate_heading(current_pos, previous_pos)
    print(f"Initial heading: {math.degrees(zumo.heading):.2f} degrees")
    previous_pos = current_pos

    # 4) Main loop
    try:
        while True:
            current_pos = uwb_reader.get_latest_position()
            if current_pos == (None, None):
                print("No valid UWB data received. Retrying...")
                time.sleep(0.1)
                continue

            print(f"Current position: {current_pos}")
            print(f"Target position: {TARGET_POS}, Tolerance: {TARGET_TOLERANCE}")

            # Check if within target proximity
            if is_within_target(current_pos, TARGET_POS, TARGET_TOLERANCE):
                print("Target position reached!")
                break

            # Recompute heading from previous_pos -> current_pos
            #zumo.heading = calculate_heading(current_pos, previous_pos)
            #previous_pos = current_pos

            # Calculate target angle
            dx_target = TARGET_POS[0] - current_pos[0]
            dy_target = TARGET_POS[1] - current_pos[1]
            theta_target = math.atan2(dy_target, dx_target)

            # Calculate relative turning angle
            gamma = normalize_angle(theta_target - zumo.heading)

            print(f"Target angle: {math.degrees(theta_target):.2f} degrees")
            print(f"Relative turning angle (gamma): {math.degrees(gamma):.2f} degrees")

            # Turn if needed
            if abs(gamma) > ANGLE_TOLERANCE:
                print("Turning to face the target.")
                turn_in_place(zumo, MOTOR_SPEED_TURN, gamma)
                # Update heading after turn
                zumo.heading = normalize_angle(zumo.heading + gamma)
            else:
                # Move forward
                distance_to_target = math.sqrt(dx_target**2 + dy_target**2)
                print(f"Moving forward by {distance_to_target:.2f} meters.")
                move_forward(zumo, distance=distance_to_target)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        zumo.send_speeds(0, 0)
        uwb_reader.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Navigate Zumo robot to a target position.")
    parser.add_argument("target_x", type=float, help="Target X coordinate")
    parser.add_argument("target_y", type=float, help="Target Y coordinate")
    args = parser.parse_args()
    main(args.target_x, args.target_y)

