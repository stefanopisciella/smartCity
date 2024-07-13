from physical.motor import Motor

import paho.mqtt.client as mqtt
from mqtt_publisher import MQTTClient

import constants as cns


class CarController:
    def __init__(self):
        # START message broker
        self.mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqttc.on_message = self.on_message_received
        self.mqttc.connect(cns.MQTT_HOSTNAME, 1883, 60)  # connect to the broker
        self.mqttc.subscribe(cns.MQTT_TOPIC)
        self.mqttc.loop_start()  # start the loop in a separate thread
        self.mqtt = MQTTClient(cns.MQTT_HOSTNAME, cns.MQTT_TOPIC)
        # END message broker

        # START freenove library
        self.motor = Motor()
        # END freenove library

        self.received_messages = None

    def act(self):
        while True:
            if self.received_messages == "FORWARD":
                self.go_forward()

    def go_forward(self):
        self.motor.setMotorModel(0, 0, 0, 0)  # CHECK

    def on_message_received(self, client, userdata, msg):
        self.received_messages = msg.payload.decode('utf-8')


if __name__ == '__main__':
    carController = CarController()
    carController.act()
