from lib.Motor import Motor

import sys
import os
import time

import paho.mqtt.client as mqttc

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # Add the parent directory to the sys.path
import constants as cns


class CarController:
    def __init__(self):
        # START message broker
        mqtt = mqttc.Client(mqttc.CallbackAPIVersion.VERSION2)
        mqtt.on_message = self.on_message_received
        mqtt.connect("192.168.98.87", 1883, 60)  # connect to the broker
        mqtt.subscribe("remoteControl")

        mqtt.loop_start()  # Start the loop in a separate thread
        # END message broker

        self.received_message = None

        # START freenove library
        self.motor = Motor()
        # END freenove library

        self.time_duration_to_reach_the_target = cns.TIME_DURATION_TO_REACH_THE_TARGET  # expressed in seconds

    def act(self):
        while True:
            # CHECK
            # print(self.received_message)

            if self.received_message == "FORWARD":
                # CHECK
                print("message_received: FORWARD")

                self.go_forward()
                time.sleep(self.time_duration_to_reach_the_target)  # ==> the robot won't move for more than time_duration_to_reach_the_target seconds
                self.stop()

                self.received_message = None
                exit()

                """
                time_duration_to_reach_the_target = 3  # expressed in seconds
                current_time = time.time()
                time_when_it_should_reach_the_target = current_time + time_duration_to_reach_the_target  # expressed in seconds
                while time.time() < time_when_it_should_reach_the_target:
                    self.go_forward()
                """

        mosquitto.loop_stop()  # stop the loop when done

    def go_forward(self):
        self.motor.setMotorModel(-1000, -1000, -1000, -1000)

    def stop(self):
        self.motor.setMotorModel(0, 0, 0, 0)

    def on_message_received(self, client, userdata, msg):
        self.received_message = msg.payload.decode('utf-8')


if __name__ == '__main__':
    carController = CarController()
    carController.act()
