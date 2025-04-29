#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YoloGazeboCamera:
    def __init__(self):
        rospy.init_node("yolo_gazebo_cam")
        self.bridge = CvBridge()
        self.model = YOLO("/home/lehoangthanhphuong/runs/detect/train7/weights/best.pt")

        self.last_depth = None  # Lưu frame depth mới nhất

        # Subscribe camera color + depth
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        rospy.loginfo("🚀 YOLOv8 Gazebo camera detector with depth started...")
        rospy.spin()

    def depth_callback(self, msg):
        try:
            # Lưu ảnh depth mới nhất
            self.last_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(f"[Lỗi đọc depth]: {e}")

    def image_callback(self, msg):
        try:
            if self.last_depth is None:
                return  # Chờ có dữ liệu depth

            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            depth = cv2.rotate(self.last_depth.copy(), cv2.ROTATE_180)

            results = self.model(frame, conf=0.5)
            annotated = frame.copy()

            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.model.names[cls_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                # Đọc khoảng cách tại (cx, cy) từ ảnh depth
                distance = depth[cy, cx]  # nếu đơn vị là mm → chuyển sang mét

                # Vẽ bbox
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Viết label + khoảng cách
                text = f"{label} {conf*100:.1f}% - {distance:.2f}m"
                cv2.putText(annotated, text, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("YOLOv8 Gazebo Detection with Depth", annotated)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"[Lỗi xử lý ảnh]: {e}")

if __name__ == "__main__":
    YoloGazeboCamera()
