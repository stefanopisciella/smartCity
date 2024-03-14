import os
import os.path as pt

PROJECT_ROOT_PATH = os.environ.get('smartCity')


VIRTUAL_CCTV_CAMERA_IMG_PATH = pt.join(PROJECT_ROOT_PATH, "virtual", "cctvCamera.jpg")
YOLO_MODEL_PATH = pt.join(PROJECT_ROOT_PATH, "YOLOv8x.pt")
PARKING_STALLS_POS_PATH = pt.join(PROJECT_ROOT_PATH, "ParkingStallsPos")