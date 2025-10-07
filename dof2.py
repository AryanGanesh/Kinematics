from math import *
import matplotlib.pyplot as plt
import numpy as np

class SetConstants:
    def __init__(self):
        # Get target coordinates (2D only for 2-DOF)
        while True:
            try:
                self.X1 = float(input("Set cartesian X coordinate to reach to: "))
                self.Y1 = float(input("Set cartesian Y coordinate to reach to: "))
                break
            except ValueError:
                print("Please enter valid numbers for coordinates.")

        # Number of DOFs (fixed at 2 for this code)
        self.num_dofs = 2
        print("Number of DOFs set to 2 (planar arm).")

        # Get link lengths 
        self.links = {}
        for i in range(1, self.num_dofs + 1):
            while True:
                try:
                    length = float(input(f"Enter length of link l{i}: "))
                    if length <= 0:
                        print("Link length must be positive.")
                    else:
                        self.links[f"l{i}"] = length
                        break
                except ValueError:
                    print("Please enter a valid positive number for link length.")

    def get_target(self):
        return (self.X1, self.Y1)

    def get_num_dofs(self):
        return self.num_dofs

    def get_links(self):
        return self.links

class Calculate2Dof:
    def __init__(self, constants: SetConstants):
        self.constants = constants

    def calculate_angles(self):
        x, y = self.constants.get_target()
        links = self.constants.get_links()
        l1 = links['l1']
        l2 = links['l2']

        # Check if target is reachable
        distance = sqrt(x**2 + y**2)
        if distance > (l1 + l2) or distance < abs(l1 - l2):
            print("Target position is out of reach for the arm.")
            return None

        try:
            cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
            # Clamp value to avoid domain error due to floating point
            cos_theta2 = min(1.0, max(-1.0, cos_theta2))
            theta2_up = acos(cos_theta2)
            theta2_down = -theta2_up

            # Elbow-up solution
            k1_up = l1 + l2 * cos(theta2_up)
            k2_up = l2 * sin(theta2_up)
            theta1_up = atan2(y, x) - atan2(k2_up, k1_up)

            # Elbow-down solution
            k1_down = l1 + l2 * cos(theta2_down)
            k2_down = l2 * sin(theta2_down)
            theta1_down = atan2(y, x) - atan2(k2_down, k1_down)

            # Convert to degrees
            shoulder_angle_up = degrees(theta1_up)
            elbow_angle_up = degrees(theta2_up)
            shoulder_angle_down = degrees(theta1_down)
            elbow_angle_down = degrees(theta2_down)

            print("\nCalculated Angles for 2-DOF Arm:")
            print(f"Shoulder (theta1): {shoulder_angle_up:.2f}° (elbow-up), {shoulder_angle_down:.2f}° (elbow-down)")
            print(f"Elbow    (theta2): {elbow_angle_up:.2f}° (elbow-up), {elbow_angle_down:.2f}° (elbow-down)")

            # Return both solutions in radians for further use
            return [(theta1_up, theta2_up), (theta1_down, theta2_down)]
        except ValueError:
            print("Calculation error: Check your input values.")
            return None

def forward_kinematics(theta1, theta2, l1, l2):
    x = l1 * cos(theta1) + l2 * cos(theta1 + theta2)
    y = l1 * sin(theta1) + l2 * sin(theta1 + theta2)
    return x, y

def test_inverse_forward_kinematics():
    print("\n--- Running Unit Test: Inverse + Forward Kinematics ---")
    l1, l2 = 2, 1
    theta1_deg, theta2_deg = 45, 45
    theta1, theta2 = radians(theta1_deg), radians(theta2_deg)
    x, y = forward_kinematics(theta1, theta2, l1, l2)
    print(f"Test position: ({x:.2f}, {y:.2f}) from angles {theta1_deg}°, {theta2_deg}°")

    # Now use inverse kinematics to get angles back
    # Use Calculate2Dof logic directly here
    distance = sqrt(x**2 + y**2)
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta2 = min(1.0, max(-1.0, cos_theta2))
    theta2 = acos(cos_theta2)
    k1 = l1 + l2 * cos(theta2)
    k2 = l2 * sin(theta2)
    theta1 = atan2(y, x) - atan2(k2, k1)
    x2, y2 = forward_kinematics(theta1, theta2, l1, l2)

    print(f"Recovered angles: {degrees(theta1):.2f}°, {degrees(theta2):.2f}°")
    print(f"Recovered position: ({x2:.2f}, {y2:.2f})")
    assert abs(x - x2) < 1e-6 and abs(y - y2) < 1e-6, "Test failed!"
    print("Unit test passed!")

def plot_workspace(l1, l2, resolution=200):
    theta1_range = np.linspace(-np.pi, np.pi, resolution)
    theta2_range = np.linspace(-np.pi, np.pi, resolution)
    X, Y = [], []
    for theta1 in theta1_range:
        for theta2 in theta2_range:
            x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
            y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
            X.append(x)
            Y.append(y)
    plt.figure(figsize=(6,6))
    plt.scatter(X, Y, s=1, color='blue')
    plt.title("Workspace of 2-DOF Planar Arm")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis('equal')
    plt.grid(True)
    plt.show()

def plot_arm(theta1, theta2, l1, l2, target=None):
    # Calculate joint positions
    x0, y0 = 0, 0
    x1 = l1 * cos(theta1)
    y1 = l1 * sin(theta1)
    x2 = x1 + l2 * cos(theta1 + theta2)
    y2 = y1 + l2 * sin(theta1 + theta2)
    plt.figure(figsize=(6,6))
    plt.plot([x0, x1], [y0, y1], 'ro-', linewidth=4)  # Link 1
    plt.plot([x1, x2], [y1, y2], 'go-', linewidth=4)  # Link 2
    plt.plot(x2, y2, 'bo', markersize=10, label='End Effector')
    if target:
        plt.plot(target[0], target[1], 'kx', markersize=12, label='Target')
    plt.title("2-DOF Arm Configuration")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

def main():
    # Run unit test
    test_inverse_forward_kinematics()

    # Gather user input for target and links
    constants = SetConstants()
    links = constants.get_links()
    l1, l2 = links['l1'], links['l2']

    # Show workspace
    print("\nVisualizing workspace...")
    plot_workspace(l1, l2)

    # Calculate inverse kinematics
    calculator = Calculate2Dof(constants)
    solutions = calculator.calculate_angles()
    if solutions:
        # Visualize both solutions
        for idx, (theta1, theta2) in enumerate(solutions):
            print(f"\nVisualizing {'elbow-up' if idx == 0 else 'elbow-down'} solution...")
            plot_arm(theta1, theta2, l1, l2, target=constants.get_target())

if __name__ == "__main__":
    main()
