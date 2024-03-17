import cv2
import pickle
import os.path as pt

from virtual.VirtualObjectDetection import VirtualObjectDetection
import constants as cns


class ParkingManager:

    def __init__(self):
        self.width = 57  # in pixels
        self.height = 113  # in pixels

        # START variables used only in click_parking_row_corners() and click_centerline_corners()
        self.parking_row_corner_num = 0
        self.centerline_corner_num = 0

        self.x_coordinate_of_current_centerline_left_corner = None

        self.parking_row_id = 1
        # END variables used only in click_parking_row_corners() and click_centerline_corners()

        self.cctv_camera_img = None
        # CHECK
        # self.annotated_cctv_camera_img = cv2.imread(cns.VIRTUAL_ANNOTATED_CCTV_CAMERA_IMG_PATH, cv2.IMREAD_UNCHANGED)

        self.vod = VirtualObjectDetection(cns.VIRTUAL_CCTV_CAMERA_IMG_PATH)

        self.detected_cars = []

        # centerlinePos: centerlinePos['X_coordinate_of_the_left_corner', 'X_coordinate_of_the_right_corner', 'Y_coordinate']
        # parkingStallsPos: ['id', 'x1', 'x2', 'parking_row_id']
        # parkingRowsPos: ['id', 'x1', 'y1', 'x2', 'y2']

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
            with open('CenterlinePos', 'rb') as f:
                self.centerlines = pickle.load(f)
        except:
            # CenterlinePos doesn't exist yet
            self.centerlines = []  # creating new empty list
        # END importing the positions from CenterlinePos

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

    def pick_corners(self, picking_parking_row_corners):
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

            if picking_parking_row_corners:
                window_name = "Parking row picker"
                callback_function = self.click_parking_row_corners
            else:
                window_name = "Centerline picker"
                callback_function = self.click_centerline_corners

            cv2.imshow(window_name, self.cctv_camera_img)

            cv2.setMouseCallback(window_name, callback_function)

            if cv2.waitKey(1) == 27:
                # the "ESC" has been pressed => stop the execution of this script
                cv2.destroyWindow(window_name)
                return

    def detect_cars(self):
        self.detected_cars.clear()  # clear detected_cars populated in the previous detection

        detected_objects = self.vod.detect()

        for detected_object in detected_objects:
            if detected_object["class"] == 67:
                # the current detected_object is a car
                self.detected_cars.append(detected_object)

    def get_all_parking_stalls(self):
        occupied_stalls = []
        free_stalls = []

        for stall in self.stalls:
            current_stall_is_occupied = False

            for car in self.detected_cars:
                if self.compute_intersection_between_car_box_and_stall_box(car["box"], stall) > 100:  # 100 pixels
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

        # CHECK
        # cls.annotate_img(self.cctv_camera_img)

    def compute_intersection_between_car_box_and_stall_box(self, car_box, stall_box):
        # intersection_x1 and intersection_y1 are the coordinates for the top-left corner of the intersection
        intersection_x1 = max(car_box["x1"], stall_box["x1"])
        intersection_y1 = max(car_box["y1"], stall_box["y1"])

        # intersection_x2 and intersection_y2 are the coordinates for the bottom-right corner of the intersection
        intersection_x2 = min(car_box["x2"], stall_box["x1"] + self.width)
        intersection_y2 = min(car_box["y2"], stall_box["y1"] + self.height)

        intersection_area = max(0, intersection_x2 - intersection_x1 + 1) * max(0, intersection_y2 - intersection_y1 + 1)

        return intersection_area

    def draw_car_boxes(self):
        for car in self.detected_cars:
            cv2.rectangle(self.cctv_camera_img, (int(car["box"]["x1"]), int(car["box"]["y1"])),
                          (int(car["box"]["x2"]), int(car["box"]["y2"])), (255, 0, 0), 1)  # blue rectangle

    def get_annotated_img(self):
        return self.cctv_camera_img

    def read_cctv_camera_img(self):
        if pt.isfile(cns.VIRTUAL_CCTV_CAMERA_IMG_PATH):
            self.cctv_camera_img = cv2.imread(cns.VIRTUAL_CCTV_CAMERA_IMG_PATH, cv2.IMREAD_UNCHANGED)

    def set_cctv_camera_img(self, img):
        self.cctv_camera_img = img

    @staticmethod
    def annotate_img(cls, img):
        cv2.imwrite(cns.VIRTUAL_ANNOTATED_CCTV_CAMERA_IMG_PATH, img)
