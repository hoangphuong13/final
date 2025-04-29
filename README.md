## ROS CUỐI KỲ
### KARTO SLAM + NAVIGATION + HUMAN TRACKING
#GIỚi THIỆU

Dự án này tập trung vào việc nghiên cứu và mô phỏng thuật toán KartoSLAM trong môi trường robot mô phỏng sử dụng Gazebo và ROS. Đồng thời, dự án mở rộng thêm bằng cách phát triển bài toán định vị (Navigation) và theo dõi con người (Human Tracking), tạo tiền đề cho các ứng dụng trong robot di động thông minh, chẳng hạn như robot phục vụ, robot hộ tống hoặc robot giám sát.

#MỤC TIÊU

- Thiết lập mô hình mô phỏng robot di động trong Gazebo
- Triển khai thuật toán KartoSLAM để xây dựng bản đồ môi trường.
- Tích hợp hệ thống định vị sử dụng ROS Navigation Stack.
- Phát triển khả năng theo dõi con người dựa trên các thuật toán nhận dạng và bám đuổi mục tiêu.
- Kiểm thử và đánh giá hiệu năng toàn hệ thống trong môi trường ảo.

#SƯ DỤNG PACKAGE

Trước tiên update ubuntu
```
sudo apt update
```
Sau đó cài đặt các package sau:
Cài đặt thư viện kartoslam và navigation
```
sudo apt install ros-noetic-slam-karto ros-noetic-navigation
```
Cài đặt pip và python
```
sudo apt install python3 python3-pip -y
```
Cài đặt pygame
```
pip3 install pygame
```
Cài ultralytics
```
pip3 install ultralytics

 Thư viện ultralytics sẽ được cài và dùng để gọi YOLO(...) trong file Python.

#ĐƯA MAP VA HUMAN VÀO TRONG GAZEBO (lưu ý đúng đường dẫn nếu không sẽ không khởi động được gazebo```)
Có tổng cộng 3 models cần đưa vào: map6; testdae; VisitorKidWalk
```
cp -r map6 ~/.gazebo/models
```
```
cp -r testdae ~/.gazebo/models
```
```
cp -r VisitorKidWalk ~/.gazebo/models
```

#CHẠY MÔ PHỎNG
