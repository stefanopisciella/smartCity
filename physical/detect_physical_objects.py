from ultralytics import YOLO

import os
import os.path as pt


os.chdir(pt.dirname(pt.dirname(__file__)))  # changing the working directory to the
# root path of the project

model = YOLO('yolov8x.pt')

results = model(source=0, show=True, conf=0.4, save=True)