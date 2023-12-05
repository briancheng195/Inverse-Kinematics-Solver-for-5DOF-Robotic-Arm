import math

def forward_kinematics(L, theta):
    """
    Calculate forward kinematics for the simplified Interbotix arm (5-DOF).

    Parameters:
    - L: List of link lengths
    - theta: List of joint angles in radians

    Returns:
    - joint_positions: List of joint positions
    """
    joint_positions = []
    x, y, z = 0, 0, 0  # Initial position

    for i in range(len(L)):
        x += L[i] * math.cos(sum(theta[:i+1]))
        y += L[i] * math.sin(sum(theta[:i+1]))
        z += L[i] * math.sin(sum(theta[:i+1]))

        joint_positions.append((x, y, z))

    return joint_positions

def interbotix_inverse_kinematics(x, y, z, L):
    """
    Calculate the inverse kinematics for the simplified Interbotix arm (5-DOF).

    Parameters:
    - x, y, z: Desired end-effector position
    - L: List of link lengths

    Returns:
    - theta: List of joint angles in radians
    """
    theta = [0] * len(L)

    for i in range(len(L)):
        r = math.sqrt(x**2 + y**2)

        # Check for division by zero
        if r == 0:
            raise ValueError("Invalid position, division by zero")

        alpha = math.atan2(y, x)

        # Check for invalid position (within link length from the base)
        if r < L[i]:
            raise ValueError("Invalid position, end-effector too close to the base")

        # Calculate denominator for cos_theta
        denominator = 2 * r * (z - L[i])

        # Check for division by zero
        if denominator == 0:
            raise ValueError("Invalid position, division by zero")

        # Ensure the argument is within the valid range for acos
        cos_theta = max(min((r**2 + (z - L[i])**2 - L[i]**2) / denominator, 1), -1)
        sin_theta = math.sqrt(1 - cos_theta**2)

        theta[i] = math.degrees(math.atan2(sin_theta, cos_theta) + alpha)

    return theta

# Example usage with specific dimensions and desired end-effector position
L = [113.25, 206.16, 200, 65+66.05, 43.1]  # Link lengths
x_desired, y_desired, z_desired = 300, 200, 0  # Desired end-effector position

# Calculate inverse kinematics
theta_solution = interbotix_inverse_kinematics(x_desired, y_desired, z_desired, L)
print("Joint Angles (degrees):", theta_solution)

