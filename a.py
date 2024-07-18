from abc import ABC, abstractmethod

# Define the common interface
class Robot(ABC):
    @abstractmethod
    def move(self):
        pass

    @abstractmethod
    def pickUp(self):
        pass

# Implement the interface for the physical robot
class PhysicalRobot(Robot):
    def move(self):
        # Code to physically move the robot
        pass

    def pickUp(self):
        # Code to physically pick up an object
        pass

# Implement the interface for the virtual robot
class VirtualRobot(Robot):
    def move(self):
        # Code to move the robot in the virtual environment
        pass

    def pickUp(self):
        # Code to pick up an object in the virtual environment
        pass

# High-level control logic
class RobotController:
    def __init__(self, robot: Robot):
        self.robot = robot

    def doSomething(self):
        self.robot.move()
        self.robot.pickUp()

# Use dependency injection to control either robot
physical_robot = PhysicalRobot()
virtual_robot = VirtualRobot()

controller = RobotController(physical_robot)  # Or use virtual_robot
controller.doSomething()
