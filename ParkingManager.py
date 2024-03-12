import cv2
import pickle

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

        self.parking_row_id = 0
        # END variables used only in click_parking_row_corners() and click_centerline_corners()

        self.parking_img = cns.VIRTUAL_CCTV_CAMERA_IMG_PATH

        self.vod = VirtualObjectDetection(cns.VIRTUAL_CCTV_CAMERA_IMG_PATH)

        self.detected_cars = []

        # pos STRUCTURE: pos['X_coordinate', 'Y_coordinate', 'is_a_parking_stall_corner']
        # centerlinePos STRUCTURE: centerlinePos['X_coordinate_of_the_left_corner', 'X_coordinate_of_the_right_corner', 'Y_coordinate']

        # START importing the positions from CarParkPos
        try:
            # CarParkPos already exists
            with open('CarParkPos', 'rb') as f:
                self.posList = pickle.load(f)
        except:
            # CarParkPos doesn't exist yet
            self.posList = []  # creating new empty list
        # END importing the positions from CarParkPos

        # START importing the positions from CenterlinePos
        try:
            # CenterlinePos already exists
            with open('CenterlinePos', 'rb') as f:
                self.centerlinePos = pickle.load(f)
        except:
            # CenterlinePos doesn't exist yet
            self.centerlinePos = []  # creating new empty list
        # END importing the positions from CenterlinePos

    def click_parking_row_corners(self, events, x, y, flags, params):
        if events == cv2.EVENT_LBUTTONDOWN:
            # user has left-clicked ==> add the clicked position to the list

            self.posList.append((x, y, False))

            self.parking_row_corner_num += 1

            if self.parking_row_corner_num == 2:
                # it has a sufficient number of corners for splitting the parking row in more parking stalls
                current_parking_row_left_corner = (self.posList[-2][0], self.posList[-2][1])  # posList[-2] for
                # retrieving the second-last inserted position

                current_stall_left_corner = (current_parking_row_left_corner[0], current_parking_row_left_corner[1])

                while current_stall_left_corner[0] < x:
                    self.posList.append((current_stall_left_corner[0], current_stall_left_corner[1], True))
                    current_stall_left_corner = (current_stall_left_corner[0] + self.width, current_stall_left_corner[1])

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
        with open('CarParkPos', 'wb') as f:
            pickle.dump(self.posList, f)
        # END saving the positions in the file

    def click_centerline_corners(self, events, x, y, flags, params):
        if events == cv2.EVENT_LBUTTONDOWN:
            # user has left-clicked ==> add the clicked position to the list

            if self.centerline_corner_num == 0:
                self.x_coordinate_of_current_centerline_left_corner = x
                self.centerline_corner_num += 1
                return

            if self.centerline_corner_num == 1:
                self.centerlinePos.append((self.x_coordinate_of_current_centerline_left_corner, x, y))  # here x is
                # relative to the right corner of the centerline
                self.centerline_corner_num = 0

        """
        if events == cv2.EVENT_RBUTTONDOWN:
            # user has right-clicked ==> remove the clicked position from the list
    
            for i, pos in enumerate(centerlinePos):  # looping the saved parking stalls
                x1, y1 = pos
                if x1 < x < x1 + width and y1 < y < y1 + height:
                    # the clicked position belongs to the current parking stall area
                    centerlinePos.pop(i)
        """

        # START saving the positions in the file
        with open('CenterlinePos', 'wb') as f:
            pickle.dump(self.centerlinePos, f)
        # END saving the positions in the file

    def get_parking_stall_positions(self):
        if self.posList is not None and len(self.posList) > 0:
            return self.posList

    def get_closest_free_parking_stall(self):
        pass

    def pick_corners(self, picking_parking_row_corners):
        while True:
            img = cv2.imread(self.parking_img)

            # START drawing all parking stalls
            for pos in self.posList:
                if pos[2]:
                    # it only considers corners relative to parking stalls and not those relative to parking rows
                    cv2.rectangle(img, (pos[0], pos[1]), (pos[0] + self.width, pos[1] + self.height), (255, 0, 255), 2)
            # END drawing all parking stalls

            # START drawing all centerlines
            for pos in self.centerlinePos:
                cv2.line(img, (pos[0], pos[2]), (pos[1], pos[2]), (0, 0, 255), 2)
            # END drawing all centerlines

            if picking_parking_row_corners:
                window_name = "Parking row picker"
                callback_function = self.click_parking_row_corners
            else:
                window_name = "Centerline picker"
                callback_function = self.click_centerline_corners

            cv2.imshow(window_name, img)

            cv2.setMouseCallback(window_name, callback_function)

            if cv2.waitKey(1) == 27:
                # the "ESC" has been pressed => stop the execution of this script
                cv2.destroyWindow(window_name)
                return

    def detect_cars(self):
        self.detected_cars = self.vod.detect()

    def get_free_parking_stalls(self):
        """
        free_parking_stalls = []

        # print("OHHHHHH", detected_cars[0]["class"])
        for car in self.detected_cars:
            if car["class"] != 67:
                continue

            for stall in self.posList:
                if car["box"]["x1"]
        """

    def get_occupied_parking_stall(self):
        pass


if __name__ == "__main__":
    pm = ParkingManager()
    pm.pick_corners(True)
