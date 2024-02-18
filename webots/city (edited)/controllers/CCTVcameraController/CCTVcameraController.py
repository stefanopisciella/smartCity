from controller import Robot

import cv2


class CCTVcameraController(Robot):
    def __init__(self):
        super().__init__()

        self.timestamp = int(self.getBasicTimeStep())

        self.camera = self.getDevice("cctv_camera")
        self.camera.enable(self.timestamp)  # Enable the camera with a sampling period of the value of self.timestamp
        self.cameraImgPath = "../../../../cctvCamera.jpg"

    def act(self):
        while self.step(self.timestamp) != -1:
            self.camera.getImage()
            self.camera.saveImage(self.cameraImgPath, 100)
            img = cv2.imread(self.cameraImgPath, cv2.IMREAD_UNCHANGED)
            cv2.imshow("CCTV camera", img)

            if cv2.waitKey(1) == 27:
                # the "ESC" has been pressed => stop the execution of the robot_controller
                self.camera.disable()
                cv2.destroyWindow("CCTV camera")
                break


if __name__ == "__main__":
    driver = CCTVcameraController()
    driver.act()
