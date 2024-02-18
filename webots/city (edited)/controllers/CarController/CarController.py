from vehicle.driver import Driver
import cv2


class CarController(Driver):
    def __init__(self, car_brand_name):
        super().__init__()
        self.car_brand_name = car_brand_name

        self.camera = self.getDevice("moving_car_camera")
        self.camera.enable(17)  # Enable the camera with a sampling period of 10ms
        self.cameraImgPath = "../../../../carCamera.jpg"

    def park(self):
        pass

    def act(self):
        while driver.step() != 1:
            # START camera
            self.camera.getImage()
            self.camera.saveImage(self.cameraImgPath, 100)
            img = cv2.imread(self.cameraImgPath, cv2.IMREAD_UNCHANGED)
            cv2.imshow(self.car_brand_name + " camera", img)
            # END camera

            # START move car
            self.setCruisingSpeed(2)  # Set cruising speed to 10 m/s
            self.setSteeringAngle(0)    # Set steering angle to 0 (straight)
            # END move car

            if cv2.waitKey(1) == 27:
                # the "ESC" has been pressed => stop the execution of the robot_controller
                self.camera.disable()
                cv2.destroyWindow(self.car_brand_name + " camera")

                self.setCruisingSpeed(0)  # Set cruising speed to 0 m/s ==> stop the car

                break


if __name__ == "__main__":
    driver = CarController("Tesla")
    driver.act()
















