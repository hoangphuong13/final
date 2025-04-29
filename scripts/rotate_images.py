#!/usr/bin/env python3
import cv2
import os

folder = "/home/ph/catkin_ws/src/slam/dataset/valid_id2"

for file in os.listdir(folder):
    if file.endswith(".jpg"):
        path = os.path.join(folder, file)
        img = cv2.imread(path)
        img_rotated = cv2.rotate(img, cv2.ROTATE_180)
        cv2.imwrite(path, img_rotated)
        print(f"✔ Đã xoay: {file}")
