from controller.robot import Robot
import math

# Initialize the Robot node
robot = Robot()

# Constants
WHEEL_RADIUS = 0.3  # Replace with the radius of your car's wheels
TIME_STEP = int(robot.getBasicTimeStep())

# Get the motor devices
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

# Enable the position sensors
left_motor.getPositionSensor().enable(TIME_STEP)
right_motor.getPositionSensor().enable(TIME_STEP)

# Start the car engine
left_motor.setVelocity(10)  # Set the velocity for the left motor
right_motor.setVelocity(10)  # Set the velocity for the right motor

# Main loop
target_distance = 100.0  # Target distance to travel in meters
total_distance = 0.0  # Total distance traveled

while robot.step(TIME_STEP) != -1:
    # Get the position sensor values (wheel encoder values)
    left_encoder = left_motor.getPositionSensor().getValue()
    right_encoder = right_motor.getPositionSensor().getValue()

    # Calculate the average distance traveled since the last step
    average_encoder = (left_encoder + right_encoder) / 2
    distance_traveled = average_encoder * WHEEL_RADIUS * 2 * math.pi

    # Update the total distance traveled
    total_distance += distance_traveled

    # Check if the target distance has been reached
    if total_distance >= target_distance:
        print(f"Reached the target distance of {target_distance} meters.")
        left_motor.setVelocity(0)  # Stop the left motor
        right_motor.setVelocity(0)  # Stop the right motor
        break
