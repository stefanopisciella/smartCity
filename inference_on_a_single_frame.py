import cv2
from ultralytics import YOLO

import constants as cns

model = YOLO('best.pt')

frame = cv2.imread(cns.VIRTUAL_CCTV_CAMERA_IMG_PATH)

while True:
    # Run YOLOv8 inference on the frame
    results = model(frame)

    # Visualize the results on the frame
    annotated_frame = results[0].plot()

    # Display the annotated frame
    cv2.imshow("YOLOv8 Inference", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        cv2.destroyAllWindows()
        break
