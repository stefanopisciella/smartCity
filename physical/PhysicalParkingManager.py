from ParkingM import ParkingManager
import constants as cns

import cv2
import requests
import numpy as np
import imutils
import pickle
import os.path as pt
import json
import time

from ultralytics import YOLO

from collections.abc import Iterable


class PhysicalParkingManager(ParkingManager):
    def __init__(self, cctv_camera_ip_address):
        self.cctv_camera_ip_address = cctv_camera_ip_address

        super().__init__()

        # START variables used only in click_parking_stall_corners()
        self.parking_stall_corner_num = 0
        self.parking_stall_count = 1

        # START importing the positions from ParkingStallsPos
        try:
            # ParkingStallsPos already exists
            with open('ParkingStallsPos', 'rb') as f:
                self.stalls = pickle.load(f)
        except:
            # ParkingStallsPos doesn't exist yet
            self.stalls = []  # creating new empty list
        # END importing the positions from ParkingStallsPos

        """
        self.cctv_camera_img = None

        self.detected_objects = None
        """

    def pick_corners(self, element=None):
        self.read_a_single_frame_captured_by_the_cctv_camera()

        while True:
            # START drawing all parking stalls
            for pos in self.stalls:
                if 'x2' in pos:  # checking if the field "x2" has already been defined
                    cv2.rectangle(self.cctv_camera_img, (pos["x1"], pos["y1"]), (pos["x2"], pos["y2"]), (255, 0, 255),
                                  2)
            # END drawing all parking stalls

            cv2.imshow("Parking row picker", self.cctv_camera_img)
            cv2.setMouseCallback("Parking row picker", self.click_parking_stall_corners)

            if cv2.waitKey(1) == 27:
                # the "ESC" has been pressed => stop the execution of this script
                cv2.destroyWindow("Parking row picker")
                return

    def save_a_single_frame_captured_by_the_cctv_camera(self):
        url = f"http://{self.cctv_camera_ip_address}:8080/shot.jpg"

        img_resp = requests.get(url)
        img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
        img = cv2.imdecode(img_arr, -1)
        img = imutils.resize(img, width=1000, height=1800)

        cv2.imwrite('cctvCamera.jpg', img)  # save the captured img from the cctv_camera

    def read_a_single_frame_captured_by_the_cctv_camera(self):
        self.cctv_camera_img = cv2.imread('cctvCamera.jpg')

    def click_parking_stall_corners(self, events, x, y, flags, params):
        if events == cv2.EVENT_LBUTTONDOWN:
            # user has left-clicked ==> add the clicked position to the list

            self.parking_stall_corner_num += 1

            if self.parking_stall_corner_num == 1:
                # selected the left corner of the current parking stall

                stall = {
                    "id": self.parking_stall_count,
                    "x1": x,
                    "y1": y
                }

                self.stalls.append(stall)
            if self.parking_stall_corner_num == 2:
                # selected the right corner of the current parking stall

                self.stalls[-1]["x2"] = x
                self.stalls[-1]["y2"] = y

                self.parking_stall_count += 1  # so that the parking stall id can be incremented
                self.parking_stall_corner_num = 0  # so that can be added others parking stalls

        # START saving the positions in the file
        with open('ParkingStallsPos', 'wb') as f:
            pickle.dump(self.stalls, f)
        # END saving the positions in the file

    def detect_and_track_objects(self):
        model = YOLO(cns.YOLO_MODEL_PATH)

        self.save_a_single_frame_captured_by_the_cctv_camera()

        frame_path = "cctvCamera.jpg"

        while True:
            if pt.exists(frame_path):
                img = cv2.imread(frame_path)

                if img is not None:
                    results = model(frame_path)

                    if isinstance(results[0],
                                  Iterable):  # it does this check to avoid an exception rise by the tojson() function
                        # YOLO has detected at least one object
                        self.detected_objects = json.loads(results[0].tojson())

                    # DEBUG
                    # print(results[0].tojson())
                    # annotated_frame = results[0].plot() # Visualize the results on the frame

                    self.read_a_single_frame_captured_by_the_cctv_camera()  # read the current frame
                    self.save_a_single_frame_captured_by_the_cctv_camera()  # save the next frame

                    cv2.imshow("CCTV camera", self.cctv_camera_img)

                    if cv2.waitKey(1) == 27:
                        # the "ESC" button has been pressed
                        break

            time.sleep(1)  # !!! SLEEP

    def detect_cars(self):
        self.detected_cars.clear()  # clear detected_cars populated in the previous detection

        for detected_object in self.detected_objects:
            if detected_object["class"] == 67 and detected_object["confidence"] > self.minimum_confidence_threshold:
                # the current detected_object is a car
                self.detected_cars.append(detected_object)

    def draw_car_boxes(self):
        for car in self.detected_cars:
            car_box_top_left_corner = (int(car["box"]["x1"]), int(car["box"]["y1"]))
            cv2.rectangle(self.cctv_camera_img, car_box_top_left_corner,
                          (int(car["box"]["x2"]), int(car["box"]["y2"])), (255, 0, 0), 2)  # blue rectangle

            # START setting the text properties
            confidence = "%.2f" % car["confidence"]  # approximation to two decimal places

            position = (int(car["box"]["x1"]), int(car["box"]["y1"] - 5))
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            color = (255, 0, 0)  # blue color
            thickness = 1  # line thickness
            # END setting the text properties

            cv2.putText(self.cctv_camera_img, "#" + " Car " + confidence, position, font, font_scale,
                        color, thickness,
                        cv2.LINE_AA)  # writing the class of the detected object


if __name__ == "__main__":
    pm = PhysicalParkingManager(cns.SMARTPHONE_IP_ADDRESS)

    # CHECK
    pm.detect_and_track_objects()
