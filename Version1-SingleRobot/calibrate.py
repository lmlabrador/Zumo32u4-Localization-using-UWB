import time
import math
from Zumo import Zumo  

# Constants for calibration
MOTOR_SPEED = 350  # Speed for calibration 
CALIBRATION_DISTANCE = 1.0  # Distance to travel for calibration (in meters)
CALIBRATION_ANGLE = 90  # Angle to turn for calibration (in degrees)

def calibrate_distance(zumo, distance):
    """Calibrate the encoder counts for a given distance."""
    print(f"Calibrating for distance: {distance} meters")

    # Reset encoders
    zumo.reset_encoders()
    left_count, right_count = 0, 0

    # Move forward
    zumo.send_speeds(MOTOR_SPEED, MOTOR_SPEED)

    # Wait until the robot has traveled the desired distance
    while left_count < distance * 10000 or right_count < distance * 10000:  # Adjust scaling factor as needed
        left_count, right_count = zumo.get_encoders()
        print(f"Encoder counts: Left={left_count}, Right={right_count}")
        time.sleep(0.1)

    # Stop the robot
    zumo.send_speeds(0, 0)

    # Calculate encoder delta per meter
    encoder_delta = (left_count + right_count) / 2
    encoder_delta_per_meter = encoder_delta / distance

    print(f"Encoder delta for {distance} meters: {encoder_delta}")
    print(f"Encoder delta per meter: {encoder_delta_per_meter}")

    return encoder_delta_per_meter

def calibrate_turning(zumo, angle):
    """Calibrate the encoder counts for a given turning angle."""
    print(f"Calibrating for turning angle: {angle} degrees")

    # Reset encoders
    zumo.reset_encoders()
    left_count, right_count = 0, 0

    # Convert angle to radians
    angle_rad = math.radians(angle)

    # Turn in place
    if angle > 0:
        zumo.send_speeds(-MOTOR_SPEED, MOTOR_SPEED)  # Turn right
    else:
        zumo.send_speeds(MOTOR_SPEED, -MOTOR_SPEED)  # Turn left

    # Wait until the robot has turned the desired angle
    while abs(left_count) < abs(angle_rad * 300) or abs(right_count) < abs(angle_rad * 300):  # Adjust scaling factor as needed
        left_count, right_count = zumo.get_encoders()
        print(f"Encoder counts: Left={left_count}, Right={right_count}")
        time.sleep(0.1)

    # Stop the robot
    zumo.send_speeds(0, 0)

    # Calculate encoder delta per radian
    encoder_delta = (abs(left_count) + abs(right_count)) / 2
    encoder_delta_per_radian = encoder_delta / angle_rad

    print(f"Encoder delta for {angle} degrees: {encoder_delta}")
    print(f"Encoder delta per radian: {encoder_delta_per_radian}")

    return encoder_delta_per_radian

def main():
    # Initialize Zumo robot
    zumo = Zumo()

    # Calibrate distance
    distance_encoder_delta = calibrate_distance(zumo, CALIBRATION_DISTANCE)
    print(f"Distance calibration complete. Encoder delta per meter: {distance_encoder_delta}")

    # Calibrate turning
    turning_encoder_delta = calibrate_turning(zumo, CALIBRATION_ANGLE)
    print(f"Turning calibration complete. Encoder delta per radian: {turning_encoder_delta}")

    # Print calibration results
    print("\nCalibration Results:")
    print(f"Encoder delta per meter (DISTANCE_TO_ENCODER_DELTA): {distance_encoder_delta}")
    print(f"Encoder delta per radian (TURN_ANGLE_TO_ENCODER_DELTA): {turning_encoder_delta}")

if __name__ == "__main__":
    main()
