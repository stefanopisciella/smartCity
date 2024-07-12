from ultralytics import YOLO
import constants as cns
import cv2
import requests
import numpy as np
import imutils
import os.path as pt
import time


def save_a_single_frame_captured_by_the_cctv_camera():
    url = f"http://{cns.SMARTPHONE_IP_ADDRESS}:8080/shot.jpg"

    img_resp = requests.get(url)
    img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
    img = cv2.imdecode(img_arr, -1)
    img = imutils.resize(img, width=1000, height=1800)

    cv2.imwrite('cctvCamera.jpg', img)  # save the captured img from the cctv_camera


def detect():
    model = YOLO(cns.YOLO_MODEL_PATH)

    frame_path = "cctvCamera.jpg"

    while True:
        save_a_single_frame_captured_by_the_cctv_camera()

        if pt.exists(frame_path):
            img = cv2.imread(frame_path)

            if img is not None:
                results = model(frame_path)

                # print(results[0].tojson()) # DEBUG
                annotated_frame = results[0].plot()  # Visualize the results on the frame

                cv2.imshow("smartphone camera", annotated_frame)

                if cv2.waitKey(1) == 27:
                    # the "ESC" button has been pressed
                    break

        time.sleep(1)  # !!! SLEEP


if __name__ == '__main__':
    detect()
