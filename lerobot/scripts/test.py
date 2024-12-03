import time

import numpy as np

from lerobot.common.robot_devices.robots.factory import make_robot
from lerobot.common.utils.utils import init_hydra_config


def move_upright(arm, steps_per_degree):
    """
    Move the robot arm upright by adjusting specific motors by -5 degrees.
    Args:
        arm: The robot's follower arm object.
        steps_per_degree: Conversion factor from degrees to motor steps.
    """
    print("Moving the robot to an upright position...")

    # Define the motor adjustments for the upright position
    upright_adjustments = [
        (3, -6),  # Wrist Flex
        (2, -10),  # Elbow Flex
        (1, -6),  # Shoulder Lift
    ]

    # Move each motor sequentially to achieve the upright position
    for motor_idx, degrees in upright_adjustments:
        motor_name = arm.motor_names[motor_idx]
        print(f"Adjusting {motor_name} (Motor {motor_idx}) by {degrees} degrees...")

        # Convert degrees to steps
        steps_to_move = (degrees * steps_per_degree) * 1.2  # Slightly more than 1 degree

        # Read current positions and calculate target
        current_positions = arm.read("Present_Position")
        target_positions = np.copy(current_positions)
        target_positions[motor_idx] += steps_to_move

        # Command the movement
        arm.write("Goal_Position", target_positions)
        time.sleep(1)  # Sleep 1 second between motions

    print("Robot is now upright.")


def vary_motors_sequentially(arm, steps_per_degree, variation_degrees):
    """
    Vary each motor sequentially by a small degree, sleep 1 second, and return to the original position.
    Args:
        arm: The robot's follower arm object.
        steps_per_degree: Conversion factor from degrees to motor steps.
        variation_degrees: Degrees to vary each motor.
    """
    print("Starting sequential motor variations...")

    # Convert variation to steps
    step_variation = variation_degrees * steps_per_degree

    # Read current positions of all motors
    original_positions = arm.read("Present_Position")

    # Iterate over each motor
    for idx, motor_name in enumerate(arm.motor_names):
        print(f"Varying {motor_name} (Motor {idx}) by {variation_degrees} degrees...")

        # Create target positions
        target_positions = np.copy(original_positions)
        target_positions[idx] += step_variation

        # Move motor to varied position
        arm.write("Goal_Position", target_positions)
        time.sleep(1)  # Sleep 1 second

        # Return motor to original position
        print(f"Returning {motor_name} to original position...")
        arm.write("Goal_Position", original_positions)
        time.sleep(1)  # Sleep 1 second

    print("Completed one variation cycle.")


def move_upright_and_vary(robot_config_path):
    """
    Moves the robot upright and continuously varies all motors sequentially.
    Args:
        robot_config_path (str): Path to the robot configuration YAML file.
    """
    # Initialize the robot configuration with cameras excluded
    robot_config = init_hydra_config(robot_config_path, ["~cameras"])
    robot = make_robot(robot_config)

    try:
        # Connect to the robot
        robot.connect()

        # Ensure the robot's follower arm exists
        if not robot.follower_arms:
            raise ValueError("No follower arms detected in the robot configuration.")

        # Degree-to-step conversion factor
        steps_per_degree = 4096 / 360.0
        variation_degrees = 1  # Degrees to vary each motor

        # Iterate through all follower arms
        for arm_name, arm in robot.follower_arms.items():
            print(f"Operating on {arm_name}...")

            # Step 1: Move the robot to an upright position
            move_upright(arm, steps_per_degree)

            # Step 2: Continuously vary each motor sequentially
            while True:
                vary_motors_sequentially(arm, steps_per_degree, variation_degrees)

    except KeyboardInterrupt:
        print("\nStopping movement...")

    finally:
        # Safely disconnect the robot
        robot.disconnect()


# Usage
if __name__ == "__main__":
    robot_config_path = "lerobot/configs/robot/so100.yaml"  # Update with the correct path
    move_upright_and_vary(robot_config_path)
