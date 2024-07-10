import os
import os.path as pt

PROJECT_ROOT_PATH = os.environ.get('smartCity')

PARKING_MANAGER_SCRIPT_ABSOLUTE_PATH = pt.join(PROJECT_ROOT_PATH, "ParkingManager.py")
VIRTUAL_FOLDER_PATH = pt.join(PROJECT_ROOT_PATH, "virtual")
PHYSICAL_FOLDER_PATH = pt.join(PROJECT_ROOT_PATH, "physical")

VIRTUAL_CCTV_CAMERA_IMG_PATH = pt.join(PROJECT_ROOT_PATH, "virtual", "cctvCamera.jpg")
VIRTUAL_CAR_CAMERA_IMG_PATH = pt.join(PROJECT_ROOT_PATH, "virtual", "carCamera.jpg")
YOLO_MODEL_PATH = pt.join(PROJECT_ROOT_PATH, "yolov8x.pt")

PARKING_STALLS_POS_PATH = pt.join(PROJECT_ROOT_PATH, "ParkingStallsPos")
PARKING_ENTRANCE_POS_PATH = pt.join(PROJECT_ROOT_PATH, "ParkingEntrancePos")
CENTERLINE_POS_PATH = pt.join(PROJECT_ROOT_PATH, "CenterlinePos")

# START msg broker
MQTT_HOSTNAME = "localhost"
MQTT_TOPIC = "remoteControl"

PARKING_ENTRANCE_CROSSED = "PARKING_ENTRANCE_CROSSED"
STOP = "STOP"
GO_FORWARD_STRAIGHT = "GO_FORW_STRAIGHT"
GO_FORWARD_STRAIGHT_LENGTH = len(GO_FORWARD_STRAIGHT)
GO_BACKWARD_STRAIGHT = "GO_BACKW_STRAIGHT"
GO_BACKWARD_STRAIGHT_LENGTH = len(GO_BACKWARD_STRAIGHT)
ROTATE_90_DEGREES_TO_RIGHT = "ROT_R"
ROTATE_90_DEGREES_TO_LEFT = "ROT_L"
MANEUVER_COMPLETED = "DONE"
START_PARKING_PHASE_IN_A_LEFT_SQUARE = "START_PAR_L"
START_PARKING_PHASE_IN_A_RIGHT_SQUARE = "START_PAR_R"
APPROACH_TO_THE_PARKING_STALL_COMPLETED = "APP_PAR_COMPL"
# END msg broker

SMARTPHONE_IP_ADDRESS = "192.168.1.9"


