"""
import torch

import  ultralytics.utils.plotting
# print(torch.cuda.get_device_name(0))

print(torch.cuda.current_device())

# print(torch.cuda.device_count())
"""
"""
import ParkingManager as pm

pm.detect_cars()
pm.get_free_parking_stalls()
"""
"""
import cv2


def click_parking_row_corners(events, x, y, flags, params):
    if events == cv2.EVENT_LBUTTONDOWN:
        print("OK")


while True:
    img = cv2.imread("camera.jpg")

    cv2.imshow("window_name", img)
    cv2.setMouseCallback("window_name", click_parking_row_corners)

    if cv2.waitKey(1) == 27:
        # the "ESC" has been pressed => stop the execution of this script
        cv2.destroyWindow("window_name")
        exit()
"""
"""
import cv2
from ultralytics import YOLO

import constants as cns

# Load the YOLOv8 model
model = YOLO('yolov8x-seg.pt')

while True:
    # Run YOLOv8 tracking on the frame, persisting tracks between frames
    results = model.track(cns.VIRTUAL_CCTV_CAMERA_IMG_PATH, persist=True)

    # Visualize the results on the frame
    annotated_frame = results[0].plot()

    # Display the annotated frame
    cv2.imshow("YOLOv8 Tracking", annotated_frame)

    # CHECK
    # print(results[0])
    for r in results:
        print(r.obb)  # print the OBB object containing the oriented detection bounding boxes

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        cv2.destroyAllWindows()
        break
"""
"""
import pickle

# START importing the positions from ParkingStallsPos
try:
    # ParkingStallsPos already exists
    with open("prova", 'rb') as f:
        prova = pickle.load(f)
except:
    # ParkingStallsPos doesn't exist yet
    prova = None  # creating new empty list
# END importing the positions from ParkingStallsPos

print(prova)

t = ("ciao", "miao")

# START saving the positions in the file
with open('prova', 'wb') as f:
    pickle.dump(t, f)
# END saving the positions in the file
"""
"""
import cv2

import constants as cns

while True:
    img = cv2.imread(cns.VIRTUAL_CCTV_CAMERA_IMG_PATH)

    resized_img = cv2.resize(img, (800, 720))

    cv2.imshow("buh", resized_img)

    if cv2.waitKey(1) == 27:
        # the "ESC" button has been pressed
        break
"""
"""
import cv2
import numpy as np
from deep_sort import DeepSort
from ultralytics import YOLO

# Load YOLOv8 and DeepSORT
model = YOLO('yolov8n.pt')
deepsort = DeepSort("mars-small128.pb")

# Loading video
cap = cv2.VideoCapture("cars.mp4")

while(cap.isOpened()):
    ret, img = cap.read()
    if not ret:
        break

    # Detecting objects
    results = model.predict(img)

    # Prepare data for DeepSORT
    bbox_xywh = []
    confidences = []
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            obj_conf = box.conf
            bbox_xywh.append([(x1+x2)/2, (y1+y2)/2, x2-x1, y2-y1])
            confidences.append(obj_conf)

    # Pass detections to DeepSORT
    tracks = deepsort.update(bbox_xywh, confidences, img)

    # Draw bounding boxes for object tracking
    for track in tracks:
        bbox = track.to_tlbr()  # Get the corrected/predicted bounding box
        cv2.rectangle(img, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (255,0,0), 2)
        cv2.putText(img, "Car ID: " + str(track.track_id),(int(bbox[0]), int(bbox[1])),0, 5e-3 * 200, (0,255,0),2)

    cv2.imshow('YOLO V8 Detection', img)
    if cv2.waitKey(1) & 0xFF == ord(' '):
        break

cap.release()
cv2.destroyAllWindows()
"""
"""
from ultralytics import YOLO

# Load a model
model = YOLO('YOLOv8x-obb.pt')  # load an official model

# Predict with the model
results = model('https://ultralytics.com/images/bus.jpg') # predict on an image
"""
"""
# I assume that the user picks the parking rows starting from the lowest and rightmost parking square to the highest and leftmost parking square
parking_row_count = 8

in_the_highest_parking_rectangle = 0 if parking_row_count <= 4 else 1

rows_in_leftmost_parking_squares = [3, 4, 7, 8]
rows_in_rightmost_parking_squares = [1, 2, 5, 6]

in_the_leftmost_parking_square = 0 if parking_row_count in rows_in_rightmost_parking_squares else 1

in_the_lower_parking_row = 0 if parking_row_count % 2 != 0 else 1
prefix_of_parking_stall_id = str(in_the_highest_parking_rectangle) + str(in_the_leftmost_parking_square) + str(
    in_the_lower_parking_row)

print(prefix_of_parking_stall_id)
"""
"""
valori = [-8.72246, -8.79357, -8.72247, -8.82912, -8.79357, -8.79357, -8.79357, -8.79357, -8.93577]
media = sum(valori) / len(valori)

varianza = sum((xi - media) ** 2 for xi in valori) / len(valori)

import math
deviazione_standard = math.sqrt(varianza)

print(deviazione_standard)
"""
target_coordinate_phase2_arr = {"lower_right_parking_square": 560,
                                "lower_left_parking_square": 540,
                                "higher_right_parking_square": 200,
                                "higher_left_parking_square": 200}
