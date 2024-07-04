from vehicle.car import Car
from controller.supervisor import Supervisor

import cv2

import paho.mqtt.client as mqtt
from mqtt_publisher import MQTTClient

import queue
import math
from math import trunc

import constants as cns


class CarController(Car):
    def __init__(self, car_brand_name, max_steering_angle):
        super().__init__()

        # CHECK
        # self.supervisor = Supervisor()

        self.car_node = self.getFromDef("car")  # Replace CAR_DEF_NAME with the actual DEF name of your car
        self.car_brand_name = car_brand_name

        self.max_steering_angle = max_steering_angle
        self.required_angle_to_brake = 6  # expressed in degrees. Precisely 6,5520015705 degrees and 0.114354000000781 radians
        self.current_parking_phase = 0
        self.approach_to_the_parking_stall_completed = False
        self.car_initial_position_in_meters = None

        self.camera = self.getDevice("moving_car_camera")
        self.camera.enable(17)  # Enable the camera with a sampling period of 10ms
        self.cameraImgPath = cns.VIRTUAL_CAR_CAMERA_IMG_PATH

        self.compass = self.getDevice('compass')
        self.compass.enable(int(self.getBasicTimeStep()))

        self.message_queue = queue.Queue()

        self.mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqttc.on_message = self.on_message_received
        self.mqttc.connect(cns.MQTT_HOSTNAME, 1883, 60)  # connect to the broker
        self.mqttc.subscribe(cns.MQTT_TOPIC)

        self.mqttc.loop_start()  # start the loop in a separate thread

        self.mqtt = MQTTClient(cns.MQTT_HOSTNAME, cns.MQTT_TOPIC)

    def on_message_received(self, client, userdata, msg):
        self.message_queue.put(msg.payload.decode('utf-8'))

    def act(self):
        # CHECK
        """
        while self.step() != -1:
            # self.rotate(2, False)
            self.rotate_x_rad(5, False, 90, False)  # it rotates the car 90 degrees to the left
            # self.go_straight(1, 0.551)

            # CHECK
            print(self.get_car_orientation_in_degrees())
        """

        try:
            parking_entrance_crossed = False

            while self.step() != -1:
                if self.message_queue.empty():
                    if not parking_entrance_crossed:
                        self.go_straight(2)
                else:
                    last_received_message = self.message_queue.queue[0]
                    print(f"Last received message: {last_received_message}")

                    if last_received_message == cns.PARKING_ENTRANCE_CROSSED:
                        parking_entrance_crossed = True

                        self.message_queue.get()  # pop this message from the queue
                    elif last_received_message == cns.STOP:
                        self.stop()

                        self.message_queue.get()  # pop this message from the queue
                    elif (last_received_message[0:cns.GO_FORWARD_STRAIGHT_LENGTH] == cns.GO_FORWARD_STRAIGHT or
                          last_received_message[0:cns.GO_BACKWARD_STRAIGHT_LENGTH] == cns.GO_BACKWARD_STRAIGHT):

                        # CHECK
                        print(f"Car position: {self.get_car_current_position()} meters")

                        if last_received_message[0:cns.GO_FORWARD_STRAIGHT_LENGTH] == cns.GO_FORWARD_STRAIGHT:
                            # CHECK
                            print(f"str: {last_received_message[cns.GO_FORWARD_STRAIGHT_LENGTH:]}")

                            self.go_straight(10, last_received_message[cns.GO_FORWARD_STRAIGHT_LENGTH:])
                        elif last_received_message[0:cns.GO_BACKWARD_STRAIGHT_LENGTH] == cns.GO_BACKWARD_STRAIGHT:
                            self.go_straight(-10, last_received_message[cns.GO_BACKWARD_STRAIGHT_LENGTH:])
                    elif last_received_message == cns.ROTATE_90_DEGREES_TO_RIGHT:
                        self.rotate_x_rad(5, True, 0, False)  # it rotates the car 90 degrees to the right
                    elif last_received_message == cns.ROTATE_90_DEGREES_TO_LEFT:
                        self.rotate_x_rad(5, False, 177, False)  # it rotates the car 90 degrees to the left
                    elif last_received_message == cns.START_PARKING_PHASE_IN_A_LEFT_SQUARE or last_received_message == cns.START_PARKING_PHASE_IN_A_RIGHT_SQUARE:
                        if last_received_message == cns.START_PARKING_PHASE_IN_A_RIGHT_SQUARE:
                            if self.current_parking_phase == 0:
                                self.go_straight(1, 0.551, True)
                            if self.current_parking_phase == 1:
                                self.rotate_x_rad(2, False, 31, True)
                            elif self.current_parking_phase == 2:
                                self.rotate_x_rad(-2, True, 61, True)
                            elif self.current_parking_phase == 3:
                                self.rotate_x_rad(2, False, 71, True)
                            elif self.current_parking_phase == 4:
                                self.rotate_x_rad(-2, True, 85, True)
                            elif self.current_parking_phase == 5:
                                self.rotate_x_rad(1, False, 90, True)  # it was 89
                            elif self.current_parking_phase == 6:
                                self.approach_to_the_parking_stall_completed = True
                        else:
                            if self.current_parking_phase == 0:
                                self.go_straight(1, 0.551, True)
                            if self.current_parking_phase == 1:
                                self.rotate_x_rad(2, False, 210, True)
                            elif self.current_parking_phase == 2:
                                self.rotate_x_rad(-2, True, 239, True)
                            elif self.current_parking_phase == 3:
                                self.rotate_x_rad(2, False, 250, True)
                            elif self.current_parking_phase == 4:
                                self.rotate_x_rad(-2, True, 269, True)
                            elif self.current_parking_phase == 5:
                                self.approach_to_the_parking_stall_completed = True

                        if self.approach_to_the_parking_stall_completed:
                            # approach to the parking stall completed

                            self.mqtt.publish_message(cns.APPROACH_TO_THE_PARKING_STALL_COMPLETED)

                            self.message_queue.get()  # pop cns.START_PARKING_PHASE_IN_A_LEFT_SQUARE or cns.START_PARKING_PHASE_IN_A_RIGHT_SQUARE message from the queue
                            self.message_queue.get()  # pop cns.APPROACH_TO_THE_PARKING_STALL_COMPLETED message from the queue

                            self.current_parking_phase += 1  # jump to the next phase

                # START camera
                self.camera.getImage()
                self.camera.saveImage(self.cameraImgPath, 100)
                img = cv2.imread(self.cameraImgPath, cv2.IMREAD_UNCHANGED)
                if img is not None:
                    cv2.imshow(self.car_brand_name + " camera", img)
                # END camera

                if cv2.waitKey(1) == 27:
                    # the "ESC" has been pressed => stop the execution of the robot_controller
                    self.camera.disable()
                    cv2.destroyWindow(self.car_brand_name + " camera")

                    self.stop()

                    break
        except Exception as e:
            print(f"An error occurred: {e}")
            self.mqttc.loop_stop()
            raise

        # self.mqttc.loop_stop()  # stop the loop

    def stop(self):
        # CHECK
        # self.setCruisingSpeed(0)  # Set cruising speed to 0 m/s ==> stop the car

        self.setThrottle(0)  # stop the throttle
        self.setBrakeIntensity(1)  # apply full brakes

    def go_straight(self, speed, distance_between_car_and_target_in_meters=None, car_is_in_parking_phase=False):
        if distance_between_car_and_target_in_meters is None:
            self.setCruisingSpeed(speed)
            self.setSteeringAngle(0)  # Set steering angle to 0 (straight)
            return

        if self.car_initial_position_in_meters is None:
            self.car_initial_position_in_meters = self.get_car_current_position()

        car_current_position = self.get_car_current_position()
        distance_moved = CarController.calculate_distance(self.car_initial_position_in_meters, car_current_position)

        # CHECK
        print(f"distance: {distance_moved}")

        if distance_moved >= float(distance_between_car_and_target_in_meters):
            self.stop()

            self.car_initial_position_in_meters = None  # reset car_initial_position_in_meters

            if car_is_in_parking_phase:
                self.current_parking_phase += 1  # jump to the next phase
            else:
                self.message_queue.get()  # pop GO_FORWARD_STRAIGHT or GO_BACKWARD_STRAIGHT message from the queue

            # CHECK
            print("FINITO IL GO STRAIGHT")
        else:
            self.setCruisingSpeed(speed)
            self.setSteeringAngle(0)  # Set steering angle to 0 (straight)

    def rotate(self, speed, turn_to_the_right, steering_angle=None):
        if steering_angle is None:
            steering_angle = self.max_steering_angle if turn_to_the_right is True else -self.max_steering_angle

        self.setRightSteeringAngle(steering_angle)  # direct setting of the steering angle for the right wheel
        self.setLeftSteeringAngle(steering_angle)  # direct setting of the steering angle for the left wheel

        self.setCruisingSpeed(speed)

    def rotate_x_rad_old(self, speed, turn_to_the_right, target_angle, car_is_in_parking_phase, steering_angle=None):
        car_orientation_in_degrees = self.get_car_orientation_in_degrees()

        # CHECK
        print(f"car_orientation: {car_orientation_in_degrees}")

        # CHECK
        if car_orientation_in_degrees == target_angle:
            # the car has finished the maneuver of rotation

            self.stop()

            if car_is_in_parking_phase:
                self.current_parking_phase += 1  # jump to the next rotation
            else:
                self.message_queue.get()  # pop cns.ROTATE_90_DEGREES_TO_RIGHT or cns.ROTATE_90_DEGREES_TO_LEFT message from the queue

                self.mqtt.publish_message(
                    cns.MANEUVER_COMPLETED)  # inform the CCTV camera that the maneuver is completed
                print("MANEUVER COMPLETED")
                self.message_queue.get()  # pop cns.MANEUVER_COMPLETED message from the queue
        else:
            # the car hasn't finished yet the maneuver of rotation

            self.rotate(speed, turn_to_the_right, steering_angle)

    def rotate_x_rad(self, speed, turn_to_the_right, target_angle, car_is_in_parking_phase, steering_angle=None):
        car_orientation_in_degrees = self.get_car_orientation_in_degrees()

        # CHECK
        print(f"car_orientation: {car_orientation_in_degrees}")

        angle_tolerance = 1

        # CHECK
        if abs(car_orientation_in_degrees - target_angle) <= angle_tolerance:
            # the car has finished the maneuver of rotation

            self.stop()

            if car_is_in_parking_phase:
                self.current_parking_phase += 1  # jump to the next rotation
            else:
                self.message_queue.get()  # pop cns.ROTATE_90_DEGREES_TO_RIGHT or cns.ROTATE_90_DEGREES_TO_LEFT message from the queue

                self.mqtt.publish_message(
                    cns.MANEUVER_COMPLETED)  # inform the CCTV camera that the maneuver is completed
                print("MANEUVER COMPLETED")
                self.message_queue.get()  # pop cns.MANEUVER_COMPLETED message from the queue
        else:
            # the car hasn't finished yet the maneuver of rotation

            self.rotate(speed, turn_to_the_right, steering_angle)

    """
    def rotate_90_degrees(self, speed, turn_to_the_right, steering_angle=None):
        # CHECK
        print("SONO NELLA FUNZIONE")

        if self.get_car_orientation_in_degrees() == 0 + self.required_angle_to_brake:  # CHECK 0 for the moment, later I have to edit it
            # the car has finished the maneuver of rotation
            self.message_queue.get()  # pop the first value of the queue
            self.stop()
            self.mqtt.publish_message(cns.MANEUVER_COMPLETED)  # inform the CCTV camera that the maneuver is completed
            print("MANEUVER COMPLETED")
            self.message_queue.get()  # pop cns.MANEUVER_COMPLETED message from the queue

            # CHECK
            print("SONO NEL RAMO TRUE")
        else:
            # the car hasn't finished yet the maneuver of rotation
            self.rotate(speed, turn_to_the_right, steering_angle)
    """

    def get_car_orientation_in_degrees(self):
        orientation = self.get_car_orientation()
        orientation_in_deg = int(math.degrees(orientation))

        if orientation_in_deg == -179:
            orientation_in_deg = 180

        if orientation_in_deg < 0:
            orientation_in_deg = 360 + orientation_in_deg

        return orientation_in_deg

    def get_car_orientation(self):
        compass_values = self.compass.getValues()

        return math.atan2(compass_values[0], compass_values[1])  # orientation expressed in rad

    def get_car_current_position(self):
        translation_field = self.car_node.getField("translation")  # the translation field contains the position of the car
        return translation_field.getSFVec3f()

    @staticmethod
    def trunc_to_two_decimal_places(num):
        return trunc(num * 100) / 100

    @staticmethod
    def calculate_distance(start_position, current_position):
        return math.sqrt((start_position[0] - current_position[0]) ** 2 +
                         (start_position[1] - current_position[1]) ** 2 +
                         (start_position[2] - current_position[2]) ** 2)


if __name__ == "__main__":
    driver = CarController("Citroen", 0.631)
    driver.act()













