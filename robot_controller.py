from controller import Robot, Camera, Motor
import aruco_detector as ad

import cv2


robot = Robot()
camera = Camera('camera')

# MOTORS
front_left_motor = Motor('front left wheel')
front_right_motor = Motor('front right wheel')
back_left_motor = Motor('back left wheel')
back_right_motor = Motor('back right wheel')

# CONSTANTS AND PARAMETERS
TIMESTAMP = int(robot.getBasicTimeStep())
MAX_SPEED = 6.28

front_left_motor.setPosition(float('inf'))
front_right_motor.setPosition(float('inf'))
back_left_motor.setPosition(float('inf'))
back_right_motor.setPosition(float('inf'))

"""
front_left_motor.setVelocity(0.0)
front_right_motor.setVelocity(0.0)
back_left_motor.setVelocity(0.0)
back_right_motor.setVelocity(0.0)
"""




###
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
arucoParams = cv2.aruco.DetectorParameters()
###

while robot.step(TIMESTAMP) != -1:
    camera.enable(TIMESTAMP)
    camera.getImage()
    camera.saveImage("camera.jpg", 100)

    img = cv2.imread("camera.jpg", cv2.IMREAD_UNCHANGED)

    ###
    h, w, _ = img.shape

    width = 1000
    height = int(width * (h / w))
    img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)

    corners, ids, rejected = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)

    detected_markers = ad.aruco_display(corners, ids, rejected, img)

    cv2.imshow("webots_camera", img)
    ###

    # START control the PIONEER
    front_left_motor_speed = 1 * MAX_SPEED
    front_right_motor_speed = 1 * MAX_SPEED

    front_left_motor.setVelocity(front_left_motor_speed)
    front_right_motor.setVelocity(front_right_motor_speed)
    # END

    if cv2.waitKey(1) == 27:
        # the "ESC" has been pressed => stop the execution of the robot_controller
        camera.disable()
        cv2.destroyWindow('webots_camera')
        break
