import time

from vehicle.driver import Driver
from vehicle.car import Car

import cv2

import paho.mqtt.client as mqtt

import queue
import math

import constants as cns


class CarController(Car):
    def __init__(self, car_brand_name, max_steering_angle):
        super().__init__()
        self.car_brand_name = car_brand_name

        self.max_steering_angle = max_steering_angle

        self.camera = self.getDevice("moving_car_camera")
        self.camera.enable(17)  # Enable the camera with a sampling period of 10ms
        self.cameraImgPath = cns.VIRTUAL_CAR_CAMERA_IMG_PATH

        self.message_queue = queue.Queue()

        self.mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqttc.on_message = self.on_message_received
        self.mqttc.connect(cns.MQTT_HOSTNAME, 1883, 60)  # connect to the broker
        self.mqttc.subscribe(cns.MQTT_TOPIC)

        self.mqttc.loop_start()  # start the loop in a separate thread

    def on_message_received(self, client, userdata, msg):
        self.message_queue.put(msg.payload.decode('utf-8'))

    def act(self):
        parking_entrance_crossed = False

        while driver.step() != 1:
            # self.rotate(5, True)

            if self.message_queue.empty():
                if not parking_entrance_crossed:
                    self.go_straight(2)
            else:
                last_received_message = self.message_queue.get()
                print(f"Last received message: {last_received_message}")

                if last_received_message == cns.PARKING_ENTRANCE_CROSSED:
                    parking_entrance_crossed = True
                elif last_received_message == cns.STOP:
                    self.stop()
                elif last_received_message == cns.GO_STRAIGHT:
                    self.go_straight(5)
                elif last_received_message == cns.ROTATE_90_DEGREES_TO_RIGHT:
                    self.rotate(10, True)

                # START camera
            self.camera.getImage()
            self.camera.saveImage(self.cameraImgPath, 100)
            img = cv2.imread(self.cameraImgPath, cv2.IMREAD_UNCHANGED)
            cv2.imshow(self.car_brand_name + " camera", img)
            # END camera

            if cv2.waitKey(1) == 27:
                # the "ESC" has been pressed => stop the execution of the robot_controller
                self.camera.disable()
                cv2.destroyWindow(self.car_brand_name + " camera")

                self.stop()

                break

        self.mqttc.loop_stop()  # stop the loop

    def stop(self):
        # CHECK
        # self.setCruisingSpeed(0)  # Set cruising speed to 0 m/s ==> stop the car

        self.setThrottle(0)  # stop the throttle
        self.setBrakeIntensity(1)  # apply full brakes

    def go_straight(self, speed):
        self.setCruisingSpeed(speed)
        self.setSteeringAngle(0)  # Set steering angle to 0 (straight)

    def rotate(self, speed, turn_to_the_right, steering_angle=None):
        if steering_angle is None:
            steering_angle = self.max_steering_angle if turn_to_the_right is True else -self.max_steering_angle

        self.setRightSteeringAngle(steering_angle)  # direct setting of the steering angle for the right wheel
        self.setLeftSteeringAngle(steering_angle)  # direct setting of the steering angle for the left wheel

        self.setCruisingSpeed(speed)


if __name__ == "__main__":
    driver = CarController("Citroen", 0.631)
    driver.act()













