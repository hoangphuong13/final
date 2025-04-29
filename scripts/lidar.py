#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import pygame
import numpy as np

class LidarDisplay:
    def __init__(self):
        rospy.init_node("lidar_display_node")

        # Publisher để gửi lệnh điều khiển
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Tần suất gửi lệnh
        self.rate = rospy.Rate(10)

        # Khởi tạo dữ liệu LIDAR
        self.lidar_data = []

        # Đăng ký subscriber LIDAR
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        # Khởi tạo Pygame và font
        pygame.init()
        self.win = pygame.display.set_mode((500, 400))
        pygame.display.set_caption("LIDAR Distance Display")
        
        # Kiểm tra nếu Pygame font không được khởi tạo đúng cách
        try:
            self.font = pygame.font.SysFont("Arial", 22)
        except pygame.error as e:
            print(f"Error initializing font: {e}")
            self.font = pygame.font.SysFont("Helvetica", 22)  # Sử dụng font khác nếu Arial không khả dụng

    def lidar_callback(self, msg):
        """Xử lý dữ liệu LIDAR nhận được"""
        self.lidar_data = msg.ranges

    def display_lidar_info(self):
        """Hiển thị thông tin khoảng cách LIDAR trên Pygame"""
        self.win.fill((255, 255, 255))  # Màu nền trắng

        # Lọc các giá trị không hợp lệ (NaN, Inf)
        valid_lidar_data = [distance for distance in self.lidar_data if not np.isnan(distance) and not np.isinf(distance)]

        if not valid_lidar_data:  # Nếu không có dữ liệu hợp lệ
            lidar_text = self.font.render("Không có dữ liệu LIDAR!", True, (255, 0, 0))
            self.win.blit(lidar_text, (50, 150))
        else:
            # Lấy các góc quan trọng: 0°, 90°, 180°, 270° tương ứng với trước, phải, sau, trái
            angles = [0, 90, 180, 270]
            distances = []
            for angle in angles:
                index = int(angle * len(self.lidar_data) / 360)  # Chuyển góc thành chỉ số trong mảng LIDAR
                distance = self.lidar_data[index] if self.lidar_data[index] != float('Inf') else 10.0
                distances.append(distance)

            # Hiển thị thông tin về các vật cản ở 4 phía
            directions = ["Sau", "Phải", "Trước", "Trái"]
            for i, (direction, distance) in enumerate(zip(directions, distances)):
                lidar_text = self.font.render(f"{direction}: {distance:.2f}m", True, (0, 0, 0))
                self.win.blit(lidar_text, (50, 100 + i * 30))  # Vẽ các thông tin vào cửa sổ

        pygame.display.update()

    def run(self):
        """Chạy chương trình hiển thị dữ liệu LIDAR"""
        running = True
        while running and not rospy.is_shutdown():
            pygame.event.pump()  # Đảm bảo pygame hoạt động chính xác

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Hiển thị thông tin LIDAR
            self.display_lidar_info()

            # Gửi lệnh điều khiển nếu cần (giả sử nếu cần điều khiển robot tránh vật cản, bạn sẽ gửi lệnh ở đây)
            twist = Twist()  # Ví dụ gửi lệnh tiến
            self.cmd_pub.publish(twist)

            self.rate.sleep()

        pygame.quit()


if __name__ == "__main__":
    try:
        lidar_display = LidarDisplay()
        lidar_display.run()
    except rospy.ROSInterruptException:
        pass
