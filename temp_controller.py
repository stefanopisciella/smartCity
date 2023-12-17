from controller import Robot, Camera, Motor

robot = Robot()
camera = Camera('camera')

# MOTORS
front_left_motor = Motor('front left wheel')
front_right_motor = Motor('front right wheel')
back_left_motor = Motor('back left wheel')
back_right_motor = Motor('back right wheel')

TIMESTAMP = int(robot.getBasicTimeStep())
MAX_SPEED = 6.28

front_left_motor.setPosition(float('inf'))
front_right_motor.setPosition(float('inf'))
back_left_motor.setPosition(float('inf'))
back_right_motor.setPosition(float('inf'))

front_left_motor.setVelocity(0.0)
front_right_motor.setVelocity(0.0)
back_left_motor.setVelocity(0.0)
back_right_motor.setVelocity(0.0)

while robot.step(TIMESTAMP) != -1:
    front_left_motor_speed = 0.5 * MAX_SPEED
    front_right_motor_speed = 0.5 * MAX_SPEED

    front_left_motor.setVelocity(front_left_motor_speed)
    front_right_motor.setVelocity(front_right_motor_speed)