import os

from ParkingManager import ParkingManager


stalls_pos_file_path = './ParkingStallsPos'
parking_rows_file_path = './ParkingRowsPos'
centerline_pos_file_path = './CenterlinePos'


if os.path.isfile(stalls_pos_file_path) and os.path.isfile(centerline_pos_file_path) and os.path.isfile(parking_rows_file_path):
    if input("Do you want to pick the parking stalls and centerlines again (y/n): ") == "y":
        # user has to pick parking stall from scratch

        os.remove(stalls_pos_file_path)
        os.remove(centerline_pos_file_path)
        os.remove(parking_rows_file_path)

    # with open("ParkingManager.py") as script:
    #    exec(script.read())
    """
    import ParkingManager as pm
    pm.pick_corners(True)
    pm.pick_corners(False)
    """
    pm = ParkingManager()
    pm.pick_corners(True)
    pm.pick_corners(False)

else:
    # with open("ParkingManager.py") as script:
    #    exec(script.read())
    """
    import ParkingManager as pm
    pm.pick_corners(True)
    pm.pick_corners(False)
    """
    pm = ParkingManager()
    pm.pick_corners(True)
    pm.pick_corners(False)



