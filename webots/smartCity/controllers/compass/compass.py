from vehicle.driver import Driver


import math

# time in [ms] of a simulation step
TIME_STEP = 64

# create the Robot instance.
robot = Driver()

# get the driver device
# driver = robot.getDriver('MyDriver')

# get the compass device
compass = robot.getDevice('compass')
# enable the compass
compass.enable(TIME_STEP)

# feedback loop: step simulation until an exit event is received
while robot.step() != -1:
    # read the sensors, e.g. using compass.getValues()
    compass_values = compass.getValues()

    # process sensor data here.
    # a simple steering algorithm would be to turn in the direction of north
    # when the car is not facing north.

    if compass_values[0] < 0:  # if car is facing south (north is in the negative x direction)
        robot.setSteeringAngle(0.1)  # turn left
    else:  # if car is facing north
        robot.setSteeringAngle(0)  # go straight

    # compute the orientation in degrees
    orientation = math.atan2(compass_values[0], compass_values[1])
    orientation_degrees = math.degrees(orientation)
    print(f"The orientation of the car is: {orientation_degrees} degrees.")

# Enter here exit cleanup code.
