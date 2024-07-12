from abc import ABC, abstractmethod

import cv2
import pickle
import os.path as pt
import time
import json
import numpy as np
import queue

from ultralytics import YOLO

import constants as cns
from mqtt_publisher import MQTTClient
import paho.mqtt.client as mqtt


# Define the common interface
class ParkingManager(ABC):
    def __init__(self):
        self.minimum_confidence_threshold = 0.0

        self.cctv_camera_img = cv2.imread(cns.VIRTUAL_CCTV_CAMERA_IMG_PATH) if pt.exists(
            cns.VIRTUAL_CCTV_CAMERA_IMG_PATH) else None

        self.detected_cars = []
        self.detected_objects = None

        self.car_about_to_park_x1_coordinate = None  # needed for marking with the "P" the car that is about to park
        self.car_about_to_park_y1_coordinate = None  # needed for marking with the "P" the car that is about to park
        self.car_about_to_park = None

        self.current_driving_phase = 1

        self.required_distance_to_brake = 22  # expressed in pixels

        self.free_parking_stalls = None
        self.occupied_parking_stalls = None
        self.parking_stall_target = None

        self.mqtt = MQTTClient(cns.MQTT_HOSTNAME, cns.MQTT_TOPIC)

        # START mqtt subscriber
        self.mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqttc.on_message = self.on_message_received
        self.mqttc.connect(cns.MQTT_HOSTNAME, 1883, 60)  # connect to the broker
        self.mqttc.subscribe(cns.MQTT_TOPIC)
        # START mqtt subscriber

        self.message_queue = queue.Queue()

        # parkingStallsPos: ['id', 'x1', 'x2', 'parking_row_id']

    @abstractmethod
    def pick_corners(self, element=None):
        pass

    def on_message_received(self, client, userdata, msg):
        self.message_queue.put(msg.payload.decode('utf-8'))
