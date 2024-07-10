import os

from PhysicalParkingManager import PhysicalParkingManager
import constants as cns


stalls_pos_file_path = 'ParkingStallsPos'

if os.path.isfile(stalls_pos_file_path):
    if input("Do you want to pick the parking stalls again (y/n): ") == "y":
        # user has to pick parking stall from scratch
        os.remove(stalls_pos_file_path)

        pm = PhysicalParkingManager(cns.SMARTPHONE_IP_ADDRESS)
        pm.save_a_single_frame_captured_by_the_cctv_camera()  # save another frame captured by the cctv camera
    else:
        pm = PhysicalParkingManager(cns.SMARTPHONE_IP_ADDRESS)
else:
    pm = PhysicalParkingManager(cns.SMARTPHONE_IP_ADDRESS)
    pm.save_a_single_frame_captured_by_the_cctv_camera()  # save another frame captured by the cctv camera

pm.pick_corners()


