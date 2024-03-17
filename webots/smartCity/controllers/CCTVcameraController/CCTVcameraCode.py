import cv2
import os.path as pt


import constants as cns
from controller import Robot
from ParkingManager import ParkingManager


class CCTVcameraController(Robot):
    def __init__(self):
        super().__init__()

        self.timestamp = int(self.getBasicTimeStep())

        self.camera = self.getDevice("cctv_camera")
        self.camera.enable(self.timestamp)  # Enable the camera with a sampling period of the value of self.timestamp

        # CHECK
        # self.virtualObjectDetection = VirtualObjectDetection(cns.VIRTUAL_CCTV_CAMERA_IMG_PATH)
        self.pm = ParkingManager()

    def act(self):
        while self.step(self.timestamp) != -1:
            self.camera.getImage()
            self.camera.saveImage(cns.VIRTUAL_CCTV_CAMERA_IMG_PATH, 100)

            self.pm.read_cctv_camera_img()
            self.pm.detect_cars()
            self.pm.draw_car_boxes()
            self.pm.draw_all_parking_stalls()

            cv2.imshow("CCTV camera", self.pm.get_annotated_img())

            if cv2.waitKey(1) == 27:
                # the "ESC" has been pressed => stop the execution of the robot_controller
                self.camera.disable()
                cv2.destroyWindow("CCTV camera")
                break


if __name__ == "__main__":
    driver = CCTVcameraController()
    driver.act()
