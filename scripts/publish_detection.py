#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO  # Giả sử bạn dùng thư viện YOLOv8 Ultralytics

class YoloPublisherNode:
    def __init__(self):
        rospy.init_node('yolo_publisher_node')
        self.pub = rospy.Publisher('/detections', Detection2DArray, queue_size=1)
        self.bridge = CvBridge()

        self.model = YOLO('/home/ph/catkin_ws/src/slam/scripts/runs/detect/custom_yolov8/weights/best.pt')  # hoặc đường dẫn model của bạn
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        rospy.loginfo("🚀 YOLO Publisher Node Started.")
        rospy.spin()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return

        results = self.model.predict(cv_image, conf=0.5)

        if not results:
            return

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        # Xử lý từng bounding box
        for result in results[0].boxes.data:  # Chỉ lấy kết quả đầu tiên
            x1, y1, x2, y2, score, class_id = result.tolist()
            class_id = int(class_id)

            if class_id != 0:  # Chỉ lấy human (class_id = 0 trong COCO)
                continue

            detection = Detection2D()
            detection.header = msg.header

            # Bounding Box center x, y và size
            detection.bbox.center.x = (x1 + x2) / 2
            detection.bbox.center.y = (y1 + y2) / 2
            detection.bbox.size_x = x2 - x1
            detection.bbox.size_y = y2 - y1

            # Hypothesis: độ tin cậy + id
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = 0  # class human = 0
            hypothesis.score = score
            detection.results.append(hypothesis)

            detection_array.detections.append(detection)

        # Publish lên topic
        self.pub.publish(detection_array)

if __name__ == '__main__':
    try:
        YoloPublisherNode()
    except rospy.ROSInterruptException:
        pass
