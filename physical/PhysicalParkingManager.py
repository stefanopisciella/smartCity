from abc import ABC

from ParkingM import ParkingManager
import constants as cns

import cv2
import requests
import numpy as np
import imutils
import pickle


class PhysicalParkingManager(ParkingManager):
    def prova(self, element=None):
        pass

    def __init__(self, cctv_camera_ip_address):
        self.cctv_camera_ip_address = cctv_camera_ip_address

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

        self.cctv_camera_img = None

    def pick_corners(self, element=None):
        self.read_a_single_frame_captured_by_the_cctv_camera()

        while True:
            # START drawing all parking stalls
            for pos in self.stalls:
                if 'x2' in pos:  # checking if the field "x2" has already been defined
                    cv2.rectangle(self.cctv_camera_img, (pos["x1"], pos["y1"]), (pos["x2"], pos["y2"]), (255, 0, 255), 2)
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


if __name__ == "__main__":
    pm = PhysicalParkingManager(cns.SMARTPHONE_IP_ADDRESS)

