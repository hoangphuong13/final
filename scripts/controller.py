#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
import pygame

def init_pygame():
    pygame.init()
    win = pygame.display.set_mode((500, 400))
    pygame.display.set_caption("Human Tracking Control Panel")
    return win

def init_ros():
    rospy.init_node("follow_keyboard_control_node")
    follow_enable_pub = rospy.Publisher("/follow_person/enable", Bool, queue_size=10)
    target_label_pub = rospy.Publisher("/target_label", String, queue_size=10)
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    return follow_enable_pub, target_label_pub, cmd_vel_pub

class ControlPanel:
    def __init__(self, win, follow_enable_pub, target_label_pub, cmd_vel_pub):
        self.win = win
        self.follow_enable_pub = follow_enable_pub
        self.target_label_pub = target_label_pub
        self.cmd_vel_pub = cmd_vel_pub

        self.font = pygame.font.SysFont("Arial", 22)
        self.move_font = pygame.font.SysFont("Arial", 48, bold=True)  # Font to cho TIẾN/LÙI/TRÁI/PHẢI
        self.follow_enabled = False
        self.current_target_label = "none"
        self.last_move = "none"  # Giá trị mặc định

        self.linear_speed = 0.5
        self.angular_speed = 1.0

    def handle_key_input(self):
        keys = pygame.key.get_pressed()
        twist = Twist()

        self.last_move = "none"  # Reset mỗi vòng

        if keys[pygame.K_w]:
            twist.linear.x = self.linear_speed
            self.last_move = "TIẾN"
        elif keys[pygame.K_s]:
            twist.linear.x = -self.linear_speed
            self.last_move = "LÙI"

        if keys[pygame.K_a]:
            twist.angular.z = self.angular_speed
            self.last_move = "TRÁI"
        elif keys[pygame.K_d]:
            twist.angular.z = -self.angular_speed
            self.last_move = "PHẢI"

        self.cmd_vel_pub.publish(twist)

        if keys[pygame.K_e]:
            self.follow_enabled = True
            self.follow_enable_pub.publish(Bool(True))
            rospy.loginfo("🟢 Human tracking: ENABLED")
        if keys[pygame.K_q]:
            self.follow_enabled = False
            self.follow_enable_pub.publish(Bool(False))
            rospy.loginfo("🔴 Human tracking: DISABLED")

        if keys[pygame.K_1]:
            self.current_target_label = "ID-1 human"
            self.target_label_pub.publish(String("ID-1 human"))
            rospy.loginfo("🎯 Selected: ID-1 human")
        if keys[pygame.K_2]:
            self.current_target_label = "ID-2 human"
            self.target_label_pub.publish(String("ID-2 human"))
            rospy.loginfo("🎯 Selected: ID-2 human")

    def display_info(self):
        self.win.fill((255, 255, 255))  # Nền trắng

        # Status
        status_text = self.font.render(f"Human Tracking: {'ON' if self.follow_enabled else 'OFF'}", True, (0, 128, 0) if self.follow_enabled else (255, 0, 0))
        target_text = self.font.render(f"Target: {self.current_target_label}", True, (0, 0, 255))

        instruction1 = self.font.render("W/S: Tiến/Lùi | A/D: Quay Trái/Phải", True, (0, 0, 0))
        instruction2 = self.font.render("E: Bật theo dõi | Q: Tắt theo dõi", True, (0, 0, 0))
        instruction3 = self.font.render("1: Theo ID-1 | 2: Theo ID-2", True, (0, 0, 0))
        bottom_text = self.font.render("Press 1 or 2 to open camera!", True, (128, 0, 128))

        # Hiển thị các dòng thông tin
        self.win.blit(status_text, (10, 20))
        self.win.blit(target_text, (10, 60))
        self.win.blit(instruction1, (10, 120))
        self.win.blit(instruction2, (10, 160))
        self.win.blit(instruction3, (10, 200))
        self.win.blit(bottom_text, (10, 350))

        # 🚀 Vẽ khung hiển thị hướng di chuyển
        pygame.draw.rect(self.win, (200, 200, 200), (150, 230, 200, 100))  # Vẽ khung xám
        pygame.draw.rect(self.win, (0, 0, 0), (150, 230, 200, 100), 2)  # Viền đen

        move_text = self.move_font.render(self.last_move, True, (0, 0, 0))
        text_rect = move_text.get_rect(center=(250, 280))  # Giữa khung
        self.win.blit(move_text, text_rect)

        pygame.display.update()

def main():
    win = init_pygame()
    follow_enable_pub, target_label_pub, cmd_vel_pub = init_ros()
    panel = ControlPanel(win, follow_enable_pub, target_label_pub, cmd_vel_pub)

    running = True
    rate = rospy.Rate(10)

    while running and not rospy.is_shutdown():
        pygame.event.pump()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        panel.handle_key_input()
        panel.display_info()

        rate.sleep()

    pygame.quit()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
