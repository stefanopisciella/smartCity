import paho.mqtt.client as mqtt

import constants as cns


class MQTTClient:
    def __init__(self, msg_broker_hostname, topic):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.topic = topic

        self.client.connect(msg_broker_hostname, 1883, 60)  # connect to the broker

    def publish_message(self, message):
        self.client.publish(self.topic, message)

    def disconnect(self):
        self.client.disconnect()  # disconnect from the broker


if __name__ == "__main__":
    client = MQTTClient(cns.MQTT_HOSTNAME, cns.MQTT_TOPIC)
    client.publish_message("John")
    client.disconnect()
