#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class BoxVsRoundDetector:
    def __init__(self):
        rospy.init_node('box_vs_round_detector', anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        rospy.loginfo("🚀 Box vs Round Detector (depth ≤ 3m) is running...")
        rospy.spin()

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            depth_array = np.array(depth_image, dtype=np.float32)

            # Xoay ảnh depth 180 độ nếu camera ngược
            depth_array = cv2.rotate(depth_array, cv2.ROTATE_180)

            # Hiển thị dạng 8-bit
            depth_display = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX)
            depth_display = np.uint8(depth_display)
            blurred = cv2.GaussianBlur(depth_display, (5, 5), 0)
            _, thresh = cv2.threshold(blurred, 10, 255, cv2.THRESH_BINARY)

            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            output = cv2.cvtColor(depth_display, cv2.COLOR_GRAY2BGR)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 500:
                    continue

                mask = np.zeros(depth_array.shape, dtype=np.uint8)
                cv2.drawContours(mask, [cnt], -1, 255, -1)
                mean_depth = cv2.mean(depth_array, mask=mask)[0]

                if mean_depth > 3.0 or mean_depth <= 0.0:
                    continue

                # --- Circularity ---
                (_, _), radius = cv2.minEnclosingCircle(cnt)
                circle_area = np.pi * (radius ** 2)
                circularity = area / circle_area if circle_area != 0 else 0

                # --- Độ cong theo chiều sâu ---
                depth_vals = depth_array[mask == 255]
                stddev_depth = np.std(depth_vals)

                # --- Phân loại ---
                if circularity > 0.8 and stddev_depth > 0.05:
                    shape = "Round"
                else:
                    shape = "Box"

                # --- Hiển thị ---
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cX, cY = x + w // 2, y + h // 2

                label = f"{shape} ({mean_depth:.2f}m)"
                cv2.drawContours(output, [cnt], -1, (0, 255, 0), 2)
                cv2.putText(output, label, (cX - 60, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                rospy.loginfo(f"✅ {label} | Area={area:.1f} | Circ={circularity:.2f} | StdZ={stddev_depth:.3f}")

            cv2.imshow("Box vs Round Detection", output)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"❌ Error: {e}")

if __name__ == '__main__':
    try:
        BoxVsRoundDetector()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
