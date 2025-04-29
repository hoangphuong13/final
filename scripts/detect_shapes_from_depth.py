#!/usr/bin/env python3
import rospy, cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

def detect_shape_from_depth(depth_msg):
    try:
        # Đọc ảnh depth đúng kiểu
        depth_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")

        # Chuẩn hóa về uint8 để xử lý
        depth_scaled = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
        depth_uint8 = depth_scaled.astype("uint8")

        # Làm mịn ảnh và tìm biên cạnh
        blurred = cv2.medianBlur(depth_uint8, 5)
        edges = cv2.Canny(blurred, 50, 150)

        # Tìm contour
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
            shape = "unknown"
            if len(approx) == 3:
                shape = "tam_giac"
            elif len(approx) == 4:
                shape = "hinh_vuong/hinh_chu_nhat"
            elif len(approx) > 10:
                shape = "hinh_tron"
            print(f"[+] Phát hiện hình dạng: {shape}")

        # Hiển thị để debug
        cv2.imshow("Depth Edges", edges)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr(f"Lỗi xử lý ảnh depth: {e}")

rospy.init_node("shape_detector")
rospy.Subscriber("/camera/depth/image_raw", Image, detect_shape_from_depth)
rospy.spin()
