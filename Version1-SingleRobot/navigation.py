import math

def calculate_turn_angle(current_pos, target_pos, current_heading):
    """Calculate the angle to turn towards the target."""
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]
    target_angle = math.atan2(dy, dx)
    return normalize_angle(target_angle - current_heading)

def is_within_target(current_pos, target_pos, tolerance):
    """Check if the robot is within the target tolerance."""
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]
    distance = math.sqrt(dx**2 + dy**2)
    return distance <= tolerance

def normalize_angle(angle):
    """Normalize an angle to the range [-π, π]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
