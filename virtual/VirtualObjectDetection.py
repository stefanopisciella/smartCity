from ultralytics import YOLO


class VirtualObjectDetection:
    def __init__(self, frame_path):
        self.model = YOLO('yolov8x.pt')  # Load the YOLOv8 model
        self.frame_path = frame_path

    def detect(self):
        # Run YOLOv8 inference on the frame
        results = self.model(self.frame_path)  # results = model.predict(source=frame, device='cpu')

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # CHECK
        # cv2.rectangle(annotated_frame, results[0][0].boxes[0])
        print("len ", len(results[0]))
        print("OHHHHHHHHHHH", results[0].boxes.xyxy[0][0])

        """
        print("len ", len(results))
        print(results[0].boxes.cls)
        print("confidence" , results[0].boxes.conf)
        """

        return annotated_frame
