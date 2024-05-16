from ultralytics import YOLO

import json

import constants as cns


class VirtualObjectDetection:
    def __init__(self, frame_path):
        self.model = YOLO(cns.YOLO_MODEL_PATH)  # Load the YOLOv8 model
        self.frame_path = frame_path

    def detect(self):
        results = self.model(self.frame_path)  # Run YOLOv8 inference on the frame
        detected_objects = json.loads(results[0].tojson())

        # print(results[0].tojson())  # for debugging
        return detected_objects

        # annotated_frame = results[0].plot()
        # return annotated_frame

        # CHECK
        # self.model.cuda()

        # CHECK
        # results = self.model.predict(source=self.frame_path, device='cuda')

        # Visualize the results on the frame
        # annotated_frame = results[0].plot()

        """
        # CHECK
        # cv2.rectangle(annotated_frame, results[0][0].boxes[0])
        print("len ", len(results[0]))
        print("OHHHHHHHHHHH", results[0].boxes.xyxy[0][0])
        print("len ", len(results))
        print(results[0].boxes)
        """

        # print(detected_objects[0]["name"], detected_objects[0]["class"])
        # print(results[0].boxes.cls)
        # print("confidence", results[0].boxes.conf)

        # print(detected_objects[0]["name"], detected_objects[0]["class"])
        # print(results[0].boxes.cls)
        # print("confidence", results[0].boxes.conf)

        
