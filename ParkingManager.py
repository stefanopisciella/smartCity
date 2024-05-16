import cv2
import pickle
import os.path as pt
import time
import json
import numpy as np

from ultralytics import YOLO

import constants as cns
from mqtt_publisher import MQTTClient


class ParkingManager:

    def __init__(self):
        self.width = 55  # in pixels
        self.height = 107  # in pixels

        self.minimum_confidence_threshold = 0.0

        # START variables used only in click_parking_row_corners() and click_centerline_corners()
        self.parking_row_corner_num = 0
        self.centerline_corner_num = 0
        self.parking_entrance_corner_num = 0

        self.x_coordinate_of_current_centerline_left_corner = None

        self.parking_entrance_x1_coordinate = None

        self.parking_row_id = 1
        # END variables used only in click_parking_row_corners() and click_centerline_corners()

        self.cctv_camera_img = cv2.imread(cns.VIRTUAL_CCTV_CAMERA_IMG_PATH)

        self.detected_cars = []
        self.detected_objects = None

        self.car_about_to_park_track_id = None
        self.car_about_to_park_x1_coordinate = None  # needed for marking with the "P" the car that is about to park
        self.car_about_to_park_y1_coordinate = None  # needed for marking with the "P" the car that is about to park
        self.car_about_to_park = None

        self.current_driving_phase = 1

        self.mqtt = MQTTClient(cns.MQTT_HOSTNAME, cns.MQTT_TOPIC)

        # centerlinePos: centerlinePos['X_coordinate_of_the_left_corner', 'X_coordinate_of_the_right_corner', 'Y_coordinate']
        # parkingStallsPos: ['id', 'x1', 'x2', 'parking_row_id']
        # parkingRowsPos: ['id', 'x1', 'y1', 'x2', 'y2']
        # ParkingEntrancePos ['x1', 'x2', 'y']

        # START importing the positions from ParkingStallsPos
        try:
            # ParkingStallsPos already exists
            with open(cns.PARKING_STALLS_POS_PATH, 'rb') as f:
                self.stalls = pickle.load(f)
        except:
            # ParkingStallsPos doesn't exist yet
            self.stalls = []  # creating new empty list
        # END importing the positions from ParkingStallsPos

        # START importing the positions from CenterlinePos
        try:
            # CenterlinePos already exists
            with open(cns.CENTERLINE_POS_PATH, 'rb') as f:
                self.centerlines = pickle.load(f)
        except:
            # CenterlinePos doesn't exist yet
            self.centerlines = []  # creating new empty list
        # END importing the positions from CenterlinePos

        # START importing the position from ParkingEntrancePos
        try:
            # ParkingEntrancePos already exists
            with open(cns.PARKING_ENTRANCE_POS_PATH, 'rb') as f:
                self.parking_entrance = pickle.load(f)
        except:
            # ParkingEntrancePos doesn't exist yet
            self.parking_entrance = None
        # END importing the positions from ParkingEntrancePos

        # START importing the positions from ParkingRowsPos
        try:
            # ParkingRowsPos already exists
            with open('ParkingRowsPos', 'rb') as f:
                self.parkingRows = pickle.load(f)
        except:
            # ParkingRowsPos doesn't exist yet
            self.parkingRows = []  # creating new empty list
        # END importing the positions from ParkingRowsPos

    def click_parking_row_corners(self, events, x, y, flags, params):
        if events == cv2.EVENT_LBUTTONDOWN:
            # user has left-clicked ==> add the clicked position to the list

            self.parking_row_corner_num += 1

            if self.parking_row_corner_num == 1:
                parking_row = {
                    "id": self.parking_row_id,
                    "x1": x,
                    "y1": y
                }
                self.parkingRows.append(parking_row)

            if self.parking_row_corner_num == 2:
                # it has a sufficient number of corners for splitting the parking row in more parking stalls

                self.parkingRows[-1]["x2"] = x
                self.parkingRows[-1]["y2"] = y

                current_parking_row_left_corner = (self.parkingRows[-1]["x1"], self.parkingRows[-1]["y1"])
                current_stall_left_corner = (current_parking_row_left_corner[0], current_parking_row_left_corner[1])
                stall_id = 1
                while current_stall_left_corner[0] <= x:
                    stall = {
                        "id": stall_id,
                        "x1": current_stall_left_corner[0],
                        "y1": current_stall_left_corner[1],
                        "x2": current_stall_left_corner[0] + self.width,
                        "y2": current_stall_left_corner[1] + self.height,
                        "parking_row_id": self.parking_row_id
                    }
                    self.stalls.append(stall)

                    current_stall_left_corner = (
                        current_stall_left_corner[0] + self.width, current_stall_left_corner[1])

                    stall_id += 1

                self.parking_row_id += 1
                self.parking_row_corner_num = 0  # so that can be added others parking rows

        """
        if events == cv2.EVENT_RBUTTONDOWN:
            # user has right-clicked ==> remove the clicked position from the list
    
            for i, pos in enumerate(posList):  # looping the saved parking stalls
                x1, y1 = pos
                if x1 < x < x1 + width and y1 < y < y1 + height:
                    # the clicked position belongs to the current parking stall area
                    posList.pop(i)
        """

        # START saving the positions in the file
        with open('ParkingStallsPos', 'wb') as f:
            pickle.dump(self.stalls, f)
        # END saving the positions in the file

        # START saving the positions in the file
        with open('ParkingRowsPos', 'wb') as f:
            pickle.dump(self.parkingRows, f)
        # END saving the positions in the file

    def click_parking_entrance_corners(self, events, x, y, flags, params):
        if events == cv2.EVENT_LBUTTONDOWN:
            self.parking_entrance_corner_num += 1

            if self.parking_entrance_corner_num == 1:
                self.parking_entrance_x1_coordinate = x

            if self.parking_entrance_corner_num == 2:
                self.parking_entrance = (self.parking_entrance_x1_coordinate, x, y)

                # START saving the positions in the file
                with open('ParkingEntrancePos', 'wb') as f:
                    pickle.dump(self.parking_entrance, f)
                # END saving the positions in the file

    def click_centerline_corners(self, events, x, y, flags, params):
        if events == cv2.EVENT_LBUTTONDOWN:
            # user has left-clicked ==> add the clicked position to the list

            if self.centerline_corner_num == 0:
                self.x_coordinate_of_current_centerline_left_corner = x
                self.centerline_corner_num += 1
                return

            if self.centerline_corner_num == 1:
                self.centerlines.append((self.x_coordinate_of_current_centerline_left_corner, x, y))  # here x is
                # relative to the right corner of the centerline
                self.centerline_corner_num = 0

        """
        if events == cv2.EVENT_RBUTTONDOWN:
            # user has right-clicked ==> remove the clicked position from the list
    
            for i, pos in enumerate(centerlines):  # looping the saved parking stalls
                x1, y1 = pos
                if x1 < x < x1 + width and y1 < y < y1 + height:
                    # the clicked position belongs to the current parking stall area
                    centerlines.pop(i)
        """

        # START saving the positions in the file
        with open('CenterlinePos', 'wb') as f:
            pickle.dump(self.centerlines, f)
        # END saving the positions in the file

    def get_parking_stall_positions(self):
        if self.stalls is not None and len(self.stalls) > 0:
            return self.stalls

    def get_closest_free_parking_stall(self):
        pass

    def pick_corners(self, element):
        while True:
            # START drawing all parking stalls
            for pos in self.stalls:
                # it only considers corners relative to parking stalls and not those relative to parking rows
                cv2.rectangle(self.cctv_camera_img, (pos["x1"], pos["y1"]),
                              (pos["x1"] + self.width, pos["y1"] + self.height), (255, 0, 255), 2)
            # END drawing all parking stalls

            # START drawing all centerlines
            for pos in self.centerlines:
                cv2.line(self.cctv_camera_img, (pos[0], pos[2]), (pos[1], pos[2]), (0, 0, 255), 2)
            # END drawing all centerlines

            # START drawing all entrance lines
            if self.parking_entrance is not None:
                cv2.line(self.cctv_camera_img, (self.parking_entrance[0], self.parking_entrance[2]), (self.parking_entrance[1], self.parking_entrance[2]), (0, 255, 0), 2)
            # END drawing all entrance lines

            if element == "stalls":
                window_name = "Parking row picker"
                callback_function = self.click_parking_row_corners
            elif element == "centerlines":
                window_name = "Centerline picker"
                callback_function = self.click_centerline_corners
            elif element == "entrances":
                window_name = "Entrance line picker"
                callback_function = self.click_parking_entrance_corners

            cv2.imshow(window_name, self.cctv_camera_img)

            cv2.setMouseCallback(window_name, callback_function)

            if cv2.waitKey(1) == 27:
                # the "ESC" has been pressed => stop the execution of this script
                cv2.destroyWindow(window_name)
                return

    def detect_cars(self):
        self.detected_cars.clear()  # clear detected_cars populated in the previous detection

        for detected_object in self.detected_objects:
            if detected_object["class"] == 67 and detected_object["confidence"] > self.minimum_confidence_threshold:
                # the current detected_object is a car
                self.detected_cars.append(detected_object)

                if self.check_if_the_car_about_to_park_has_been_already_detected()\
                        and detected_object["track_id"] == self.car_about_to_park_track_id:

                    self.car_about_to_park_x1_coordinate = detected_object["box"]["x1"]
                    self.car_about_to_park_y1_coordinate = detected_object["box"]["y1"]

    def get_all_parking_stalls(self):
        occupied_stalls = []
        free_stalls = []

        for stall in self.stalls:
            current_stall_is_occupied = False

            for car in self.detected_cars:
                if self.compute_intersection_between_car_box_and_stall_box(car["box"], stall) > 300:  # 300 pixels
                    occupied_stalls.append(stall)
                    current_stall_is_occupied = True

            if current_stall_is_occupied is False:
                free_stalls.append(stall)

        return free_stalls, occupied_stalls

    def draw_all_parking_stalls(self):
        free_stalls, occupied_stalls = self.get_all_parking_stalls()

        for stall in free_stalls:
            cv2.rectangle(self.cctv_camera_img, (stall["x1"], stall["y1"]),
                          (stall["x1"] + self.width, stall["y1"] + self.height), (0, 255, 0), 2)  # green rectangle

        for stall in occupied_stalls:
            cv2.rectangle(self.cctv_camera_img, (stall["x1"], stall["y1"]),
                          (stall["x1"] + self.width, stall["y1"] + self.height), (0, 0, 255), 2)  # red rectangle

    def draw_centerlines(self):
        for centerline in self.centerlines:
            cv2.line(self.cctv_camera_img, (centerline[0], centerline[2]), (centerline[1], centerline[2]), (0, 255, 255), 1)  # yellow line

    def compute_intersection_between_car_box_and_stall_box(self, car_box, stall_box):
        # intersection_x1 and intersection_y1 are the coordinates for the top-left corner of the intersection
        intersection_x1 = max(car_box["x1"], stall_box["x1"])
        intersection_y1 = max(car_box["y1"], stall_box["y1"])

        # intersection_x2 and intersection_y2 are the coordinates for the bottom-right corner of the intersection
        intersection_x2 = min(car_box["x2"], stall_box["x1"] + self.width)
        intersection_y2 = min(car_box["y2"], stall_box["y1"] + self.height)

        intersection_area = max(0, intersection_x2 - intersection_x1 + 1) * max(0,
                                                                                intersection_y2 - intersection_y1 + 1)

        return intersection_area

    def draw_car_boxes(self):
        for car in self.detected_cars:
            car_box_top_left_corner = (int(car["box"]["x1"]), int(car["box"]["y1"]))
            cv2.rectangle(self.cctv_camera_img, car_box_top_left_corner,
                          (int(car["box"]["x2"]), int(car["box"]["y2"])), (255, 0, 0), 2)  # blue rectangle

            # START setting the text properties
            confidence = "%.2f" % car["confidence"]  # approximation to two decimal places
            track_id = car["track_id"]

            position = (int(car["box"]["x1"]), int(car["box"]["y1"] - 5))
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            color = (255, 0, 0)  # blue color
            thickness = 1  # line thickness
            # END setting the text properties

            """ CHECK
            # START creating a rectangle for the background of the text
            (text_width, text_height) = cv2.getTextSize(text, font, font_scale, thickness)[0]  # Get the size of the
            # text box

            rectangle_bgr = (255, 0, 0)  # blue color
            rectangle_top_left = (position[0], position[1] - text_height)
            rectangle_bottom_right = (position[0] + text_width, position[1] + text_height)

            cv2.rectangle(self.cctv_camera_img, rectangle_top_left, rectangle_bottom_right, rectangle_bgr, -1)
            # END creating a rectangle for the background of the text
            """

            cv2.putText(self.cctv_camera_img, "#" + str(track_id) + " Car " + confidence, position, font, font_scale, color, thickness,
                        cv2.LINE_AA)  # writing the
            # class of the detected object

    def get_annotated_cctv_camera_img(self):
        return self.cctv_camera_img

    def read_cctv_camera_img(self):
        if pt.isfile(cns.VIRTUAL_CCTV_CAMERA_IMG_PATH):
            self.cctv_camera_img = cv2.imread(cns.VIRTUAL_CCTV_CAMERA_IMG_PATH, cv2.IMREAD_UNCHANGED)

    def detect_and_track_objects(self, lock):
        model = YOLO(cns.YOLO_MODEL_PATH)

        frame_path = cns.VIRTUAL_CCTV_CAMERA_IMG_PATH

        while True:
            if pt.exists(frame_path):
                lock.acquire()
                img = cv2.imread(frame_path)
                if img is not None:
                    results = model.track(frame_path, persist=True)  # persisting tracks between frames

                    # annotated_frame = results[0].plot() # Visualize the results on the frame

                    # CHECK
                    # print(f"BOOOMER {results[0]}")

                    self.detected_objects = json.loads(results[0].tojson())

                    self.read_frame()  # without this call all the drawing methods couldn't draw on the frame

                    # CHECK
                    lock.release()

                    self.detect_cars()

                    self.detect_the_car_that_is_about_to_park()
                    self.mark_the_car_that_is_about_to_park()
                    self.guide_the_car_that_is_about_to_park()

                    self.draw_car_boxes()
                    self.draw_all_parking_stalls()
                    self.draw_centerlines()

                    cv2.imshow("CCTV camera", self.cctv_camera_img)

                    if cv2.waitKey(1) == 27:
                        # the "ESC" button has been pressed
                        break

                    """
                    # Break the loop if 'q' is pressed
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                    """

                # CHECK
                # lock.release()

            time.sleep(0.5)  # !!! SLEEP

    def guide_the_car_that_is_about_to_park(self):
        if not self.find_the_car_that_is_about_to_park():
            return

        if self.current_driving_phase == 1:
            self.guide_the_car_until_the_centerline()  # 1° phase
        elif self.current_driving_phase == 2:
            self.rotate_the_car_90_degrees()  # 2° phase

    def guide_the_car_until_the_centerline(self):
        # for the moment I will only consider the lowest centerline
        centerline = self.centerlines[0]
        centerline_y_coordinate = centerline[2]

        if self.car_about_to_park["box"]["y1"] <= centerline_y_coordinate:
            # the car that is about to park has crossed the centerline ==> it has to stop, and it has to jump to the next phase
            self.send_stop()
            self.current_driving_phase += 1
            return  # ==> it goes to the next phase
        else:
            # the car that is about to park has not crossed the centerline yet ==> it has to continue to move to reach it
            self.send_go_straight()

    def rotate_the_car_90_degrees(self):
        # for the moment I assume that the car can only rotate to its right
        pass

    def find_the_car_that_is_about_to_park(self):
        if not self.check_if_the_car_about_to_park_has_been_already_detected():
            return False

        for car in self.detected_cars:
            if car["track_id"] == self.car_about_to_park_track_id:
                self.car_about_to_park = car
                return True
        return False

    def read_frame(self):
        self.cctv_camera_img = cv2.imread(cns.VIRTUAL_CCTV_CAMERA_IMG_PATH)

    def read_image(self, delay=0.5):
        while True:
            try:
                img = cv2.imread(cns.VIRTUAL_CCTV_CAMERA_IMG_PATH)
                if img is not None:
                    cv2.imshow("test", img)
            except Exception as e:
                print(f"Attempt failed with error: {e}")
            time.sleep(delay)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    def mark_the_car_that_is_about_to_park(self):
        if self.check_if_the_car_about_to_park_has_been_already_detected()\
                and self.car_about_to_park_x1_coordinate is not None\
                and self.car_about_to_park_y1_coordinate is not None:
            cv2.putText(self.cctv_camera_img,
                        "P",
                        (int(self.car_about_to_park_x1_coordinate) - 20, int(self.car_about_to_park_y1_coordinate)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),  # red color
                        3,
                        cv2.LINE_AA)  # writing the "P" mark

    def detect_the_car_that_is_about_to_park(self):
        if self.check_if_the_car_about_to_park_has_been_already_detected():
            return

        for detected_car in self.detected_cars:
            # CHECK
            # print("if cond" + str(detected_car["box"]["y1"]) + " : " + str(self.get_parking_entrance_y_coordinate()))

            if int(detected_car["box"]["y1"]) >= self.get_parking_entrance_y_coordinate()\
                    and self.parking_entrance[0] <= int(detected_car["box"]["x1"]) <= self.parking_entrance[1]\
                    and self.parking_entrance[0] <= int(detected_car["box"]["x2"]) <= self.parking_entrance[1]:
                # the current detected car is behind the parking entrance line, and it is aligned to the parking entrance line ==> the current
                # detected car is about to park

                print(f'track_id: {detected_car["track_id"]}')
                print(f"parking_entrance: {self.parking_entrance}")

                self.car_about_to_park_track_id = detected_car["track_id"]  # I assume that only one car at a time can
                # be about to park
                self.send_parking_entrance_crossed()

        """
        if self.check_if_the_car_about_to_park_has_been_already_detected():
            return self.car_about_to_park_track_id

        car_about_to_park_track_id = None
        detected_car_in_lowest_position = 0  # detected car in the lowest position means the detected car with the
        # highest value of "y1"

        for detected_car in self.detected_cars:
            if detected_car["y1"] >= detected_car_in_lowest_position:
                detected_car_in_lowest_position = detected_car["y1"]
                car_about_to_park_track_id = detected_car["track_id"]

        return car_about_to_park_track_id
        """

    def get_parking_entrance_y_coordinate(self):
        return self.parking_entrance[2]

    def check_if_the_car_about_to_park_has_been_already_detected(self):
        return self.car_about_to_park_track_id is not None

    @staticmethod
    def check_if_two_lines_are_parallel(line1, line2):
        """
        line1 = np.array([[x1_1, y1_1], [x1_2, y1_2]])
        line2 = np.array([[x2_1, y2_1], [x2_2, y2_2]])
        """

        # Check if the lines are vertical
        if line1["x1"] == line1["x2"] and line2["x1"] == line2["x2"]:
            print("The lines are vertical and parallel.")
        elif line1["x1"] == line1["x2"] or line2["x1"] == line2["x2"]:
            print("One of the lines is vertical.")
        else:
            # Calculate the slopes of the lines
            slope1 = (line1["y1"] - line1["y2"]) / (line1["x2"] - line1["x1"])  # Adjusted for image coordinates
            slope2 = (line2["y1"] - line2["y2"]) / (line2["x2"] - line2["x1"])  # Adjusted for image coordinates

            # Check if the lines are parallel
            if np.isclose(slope1, slope2):
                print("The lines are parallel.")
            else:
                print("The lines are not parallel.")

    def send_stop(self):
        self.mqtt.publish_message(cns.STOP)

    def send_go_straight(self):
        self.mqtt.publish_message(cns.GO_STRAIGHT)

    def send_parking_entrance_crossed(self):
        self.mqtt.publish_message(cns.PARKING_ENTRANCE_CROSSED)

    @staticmethod
    def run(lock):
        pm = ParkingManager()
        pm.detect_and_track_objects(lock)