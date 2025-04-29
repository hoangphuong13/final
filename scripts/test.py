#!/usr/bin/env python3
from ultralytics import YOLO
import cv2

# Load model
model = YOLO("runs/detect/custom_yolov8/weights/best.pt")

# Load ảnh test
img_path = "/home/ph/catkin_ws/src/slam/dataset/human.v1i.yolov8/train/images/img0000_jpg.rf.502342f611274eccc53bffe9e8b7177b.jpg"
results = model(img_path, conf=0.01)  # hạ conf nếu cần

# Hiển thị kết quả
annotated_frame = results[0].plot()
cv2.imshow("Prediction", annotated_frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
