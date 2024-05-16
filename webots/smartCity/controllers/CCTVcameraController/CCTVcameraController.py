import os
from multiprocessing import Process, Lock

from webots.smartCity.controllers.CCTVcameraController.CCTVcameraCode import CCTVcameraController
from ParkingManager import ParkingManager


def run_parking_manager_process(lock):
    shell_command = 'cmd /c "conda activate pyTorch"'
    os.system(shell_command)  # we run ParkingManager.py code in another terminal after the "conda activate pyTorch"
    # command so that we can exploit the libraries installed in the pyTorch conda environment

    ParkingManager.run(lock)


def run_cctv_camera_controller(lock):
    CCTVcameraController.run(lock)


if __name__ == "__main__":
    lk = Lock()

    cctv_cam_process = Process(target=run_cctv_camera_controller, args=(lk,))
    parking_manager_process = Process(target=run_parking_manager_process, args=(lk,))

    cctv_cam_process.start()
    parking_manager_process.start()