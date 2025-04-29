#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class DelayedImageSaver:
    def __init__(self):
        rospy.init_node("delayed_image_saver", anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.last_saved_time = 0
        self.save_interval = 2  # giây
        self.image_count = 0
        rospy.spin()

    def image_callback(self, msg):
        now = time.time()
        if now - self.last_saved_time >= self.save_interval:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                filename = f"/home/ph/catkin_ws/src/slam/dataset/tester_idw2/img{self.image_count:04d}.jpg"
                cv2.imwrite(filename, cv_image)
                rospy.loginfo(f"📸 Saved: {filename}")
                self.last_saved_time = now
                self.image_count += 1
            except Exception as e:
                rospy.logerr(f"Lỗi khi lưu ảnh: {e}")

if __name__ == "__main__":
    DelayedImageSaver()
