#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from ultralytics import YOLO

class FollowPersonNavigation:
    def __init__(self):
        rospy.init_node("follow_person_navigation")
        self.bridge = CvBridge()

        # Load model YOLOv8
        self.model = YOLO("/home/ph/catkin_ws/src/slam/scripts/runs/detect/custom_yolov8/weights/best.pt")

        # Subscribers
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback, queue_size=1)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback, queue_size=1)
        rospy.Subscriber("/camera/camera_info", CameraInfo, self.camera_info_callback, queue_size=1)

        # Publisher
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        # Data holders
        self.last_rgb = None
        self.last_depth = None
        self.fx = self.fy = self.cx = self.cy = None

        # Update goal mỗi 1s
        self.update_rate = rospy.Rate(1)  # Hz

        rospy.loginfo("🚀 Node Follow Person Navigation đã khởi động.")
        self.run()

    def camera_info_callback(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def depth_callback(self, msg):
        try:
            self.last_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(f"Lỗi đọc depth: {e}")

    def image_callback(self, msg):
        try:
            self.last_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Lỗi đọc ảnh RGB: {e}")

    def run(self):
        while not rospy.is_shutdown():
            if self.last_rgb is None or self.last_depth is None:
                self.update_rate.sleep()
                continue
            if self.fx is None:
                rospy.logwarn_throttle(5, "⚠️ Chưa nhận được camera_info...")
                self.update_rate.sleep()
                continue

            try:
                frame = self.last_rgb.copy()
                frame = cv2.rotate(frame, cv2.ROTATE_180)
                depth = cv2.rotate(self.last_depth.copy(), cv2.ROTATE_180)

                results = self.model(frame, conf=0.5)
                person_target = None

                for box in results[0].boxes:
                    cls_id = int(box.cls[0])
                    label = self.model.names[cls_id]

                    if label.lower() != "person":
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                    if 0 <= cx < depth.shape[1] and 0 <= cy < depth.shape[0]:
                        distance = depth[cy, cx]
                        if 0.2 < distance < 10.0:  # khoảng cách hợp lý
                            person_target = (cx, cy, distance)
                            break  # Ưu tiên người đầu tiên tìm thấy

                if person_target:
                    self.send_goal(person_target)

            except Exception as e:
                rospy.logerr(f"Lỗi xử lý frame: {e}")

            self.update_rate.sleep()

    def send_goal(self, target):
        cx, cy, depth = target

        # Chuyển pixel (u,v,depth) sang tọa độ (X, Y) trong robot frame
        X = (cx - self.cx) * depth / self.fx
        Y = (cy - self.cy) * depth / self.fy
        Z = depth

        # Trong robot frame (base_link), ta dùng X trước mặt, Z là chiều cao nên không cần
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "base_link"  # goal tương đối so với robot

        goal.pose.position.x = Z  # Z là trục trước của camera (robot)
        goal.pose.position.y = -X  # X lệch trái/phải
        goal.pose.position.z = 0
        goal.pose.orientation.w = 1.0  # Quay mặt đúng hướng

        self.goal_pub.publish(goal)
        rospy.loginfo_throttle(2, f"🎯 Đang theo người tại (X={Z:.2f}, Y={-X:.2f})")

if __name__ == "__main__":
    FollowPersonNavigation()
