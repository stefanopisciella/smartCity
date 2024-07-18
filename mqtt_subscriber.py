import paho.mqtt.client as mqttc
import queue

message_queue = queue.Queue()


def on_message_received(client, userdata, msg):
    message_queue.put(msg.payload.decode('utf-8'))

mqtt = mqttc.Client(mqttc.CallbackAPIVersion.VERSION2)
mqtt.on_message = on_message_received
mqtt.connect("localhost", 1883, 60)  # connect to the broker
mqtt.subscribe("remoteControl")

mqtt.loop_start()  # Start the loop in a separate thread

""" old code
mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqtt.on_message = on_message_received
mqtt.connect("localhost", 1883, 60)  # connect to the broker
mqtt.subscribe("remoteControl")

mqtt.loop_start()  # Start the loop in a separate thread
"""

# Main thread can perform other tasks while MQTT client thread is running
while True:
    if not message_queue.empty():
        last_received_message = message_queue.get()
        print(f"Last received message: {last_received_message}")

mqtt.loop_stop()  # stop the loop when done