#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

if __name__ == "__main__":
    rospy.init_node("toggle_follow_node")
    pub = rospy.Publisher("/follow_person/enable", Bool, queue_size=1)

    rate = rospy.Rate(1)
    state = False

    print("Nhấn Enter để bật/tắt chế độ theo dõi người (giữ giữa & cách 2m)...")

    while not rospy.is_shutdown():
        input()
        state = not state
        pub.publish(Bool(data=state))
        print(f"🟢 Chế độ theo dõi: {'BẬT' if state else 'TẮT'}")
        rate.sleep()
