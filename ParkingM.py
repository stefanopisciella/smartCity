from abc import ABC, abstractmethod


# Define the common interface
class ParkingManager(ABC):
    @abstractmethod
    def pick_corners(self, element=None):
        pass

    @abstractmethod
    def prova(self, element=None):
        pass
