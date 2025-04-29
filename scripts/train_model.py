#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YoloGazeboCamera:
    def __init__(self):
        rospy.init_node("yolo_gazebo_cam")

        self.bridge = CvBridge()
        self.model = YOLO("/home/ph/catkin_ws/src/slam/scripts/runs/detect/custom_yolov85/weights/best.pt")

        # Giao tiếp ROS
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        rospy.Subscriber("/follow_person/enable", Bool, self.follow_callback)
        rospy.Subscriber("/target_label", String, self.target_label_callback)  # 👉 Thêm lắng nghe target
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.enable_follow = False
        self.last_depth = None
        self.target_label = None  

        rospy.loginfo("🚀 YOLOv8 + Depth + Chờ chọn người để theo...")
        rospy.spin()

    def follow_callback(self, msg):
        self.enable_follow = msg.data
        state = "BẬT" if msg.data else "TẮT"
        rospy.loginfo(f"🟢 Chế độ theo người: {state}")

    def target_label_callback(self, msg):
        self.target_label = msg.data
        rospy.loginfo(f"🎯 Đã cập nhật người cần theo: {self.target_label}")

    def depth_callback(self, msg):
        try:
            self.last_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(f"[Lỗi đọc depth]: {e}")

    def image_callback(self, msg):
        try:
            if self.last_depth is None or self.target_label is None:
                return  # Chưa có depth hoặc target thì bỏ qua

            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            depth = cv2.rotate(self.last_depth.copy(), cv2.ROTATE_180)

            results = self.model(frame, conf=0.5)
            annotated = frame.copy()

            target_found = False

            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.model.names[cls_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                # Lấy khoảng cách tại tâm bbox
                if cy >= depth.shape[0] or cx >= depth.shape[1]:
                    continue  # Né lỗi nếu out of image size

                distance = depth[cy, cx]

                # Nếu distance lỗi (NaN, inf) thì bỏ
                if np.isnan(distance) or np.isinf(distance) or distance == 0:
                    continue

                # Vẽ bbox
                color = (0, 255, 0) if label == self.target_label else (0, 0, 255)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                text = f"{label} {conf*100:.1f}% {distance:.2f}m"
                cv2.putText(annotated, text, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                if self.enable_follow and label == self.target_label:
                    target_found = True
                    self.move_towards_target(cx, frame.shape[1], distance)

            if self.enable_follow and not target_found:
                self.cmd_pub.publish(Twist())

            cv2.imshow("YOLOv8 Gazebo Detection", annotated)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"[Lỗi xử lý ảnh]: {e}")

    def move_towards_target(self, cx, frame_width, distance):
        twist = Twist()     

        center_x = frame_width // 2
        error_x = cx - center_x

        rotate_k = 0.006
        error_threshold = 15

        if abs(error_x) > error_threshold:
            twist.angular.z = -rotate_k * error_x
            twist.angular.z = np.clip(twist.angular.z, -2.0, 2.0)
        else:
            twist.angular.z = 0.0

        safe_min = 1.8
        safe_max = 2.5

        if distance > safe_max:
            twist.linear.x = 0.6
        elif distance < safe_min:
            twist.linear.x = -0.6
        else:
            twist.linear.x = 0.2

        self.cmd_pub.publish(twist)

if __name__ == "__main__":
    try:
        YoloGazeboCamera()
    except rospy.ROSInterruptException:
        pass
