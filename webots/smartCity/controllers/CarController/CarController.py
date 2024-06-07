from vehicle.car import Car

import cv2

import paho.mqtt.client as mqtt
from mqtt_publisher import MQTTClient

import queue
import math

import constants as cns


class CarController(Car):
    def __init__(self, car_brand_name, max_steering_angle):
        super().__init__()
        self.car_brand_name = car_brand_name

        self.max_steering_angle = max_steering_angle
        self.required_angle_to_brake = 6  # expressed in degrees. Precisely 6,5520015705 degrees

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
            self.go_straight(-4)
            # self.rotate(-4, True)
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
                    elif last_received_message == cns.GO_STRAIGHT:
                        self.go_straight(5)

                        self.message_queue.get()  # pop this message from the queue
                    elif last_received_message == cns.ROTATE_90_DEGREES_TO_RIGHT:
                        self.rotate_90_degrees(10, True)

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

    def go_straight(self, speed):
        self.setCruisingSpeed(speed)
        self.setSteeringAngle(0)  # Set steering angle to 0 (straight)

    def rotate(self, speed, turn_to_the_right, steering_angle=None):
        if steering_angle is None:
            steering_angle = self.max_steering_angle if turn_to_the_right is True else -self.max_steering_angle

        self.setRightSteeringAngle(steering_angle)  # direct setting of the steering angle for the right wheel
        self.setLeftSteeringAngle(steering_angle)  # direct setting of the steering angle for the left wheel

        self.setCruisingSpeed(speed)

    def rotate_90_degrees(self, speed, turn_to_the_right, steering_angle=None):
        if self.get_car_orientation_in_degrees() == 0 + self.required_angle_to_brake:  # CHECK 0 for the moment, later I have to edit it
            # the car has finished the maneuver of rotation
            self.message_queue.get()  # pop the first value of the queue
            self.stop()
            self.mqtt.publish_message(cns.MANEUVER_COMPLETED)  # inform the CCTV camera that the maneuver is completed
            print("MANEUVER COMPLETED")
            self.message_queue.get()  # pop cns.MANEUVER_COMPLETED message from the queue
        else:
            # the car hasn't finished yet the maneuver of rotation
            self.rotate(speed, turn_to_the_right, steering_angle)

    def get_car_orientation_in_degrees(self):
        compass_values = self.compass.getValues()

        orientation = math.atan2(compass_values[0], compass_values[1])
        return int(math.degrees(orientation))


if __name__ == "__main__":
    driver = CarController("Citroen", 0.631)
    driver.act()













