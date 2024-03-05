import cv2
import os
import os.path as pt


os.chdir(pt.dirname(pt.dirname(pt.dirname(pt.dirname(pt.dirname(__file__))))))  # changing the working directory to the
# root path of the project

from controller import Robot
from virtual.VirtualObjectDetection import VirtualObjectDetection


class CCTVcameraController(Robot):
    def __init__(self):
        super().__init__()

        self.timestamp = int(self.getBasicTimeStep())

        self.camera = self.getDevice("cctv_camera")
        self.camera.enable(self.timestamp)  # Enable the camera with a sampling period of the value of self.timestamp
        self.camera_img_basename = "cctvCamera.jpg"
        self.camera_img_path = pt.join(os.getcwd(), "virtual", self.camera_img_basename)

        self.virtualObjectDetection = VirtualObjectDetection(self.camera_img_path)

    def act(self):
        while self.step(self.timestamp) != -1:
            self.camera.getImage()
            self.camera.saveImage(self.camera_img_path, 100)

            """
            img = cv2.imread(self.camera_img_path, cv2.IMREAD_UNCHANGED)
            cv2.imshow("CCTV camera", img)
            """

            if pt.isfile(self.camera_img_path):
                print("ESISTE")
                annotated_img = self.virtualObjectDetection.detect()
                cv2.imshow("CCTV camera", annotated_img)

            if cv2.waitKey(1) == 27:
                # the "ESC" has been pressed => stop the execution of the robot_controller
                self.camera.disable()
                cv2.destroyWindow("CCTV camera")
                break


if __name__ == "__main__":
    driver = CCTVcameraController()
    driver.act()
