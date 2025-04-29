## ROS CUỐI KỲ
### KARTO SLAM + NAVIGATION + HUMAN TRACKING
#GIỚi THIỆU

video demo:
```
https://drive.google.com/file/d/14pbjE2V6E2G2q7Pn4TOR8o9Q99FabIsQ/view
```


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
```

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


Hoặc thực hiện thủ công bằng cách: 

copy 3 file testdae; VisitorKidWalk; map6  
vào mục Home ấn contrl+H để hiện các thư mục ẩn vào .gazebo tạo thư mục models và dán 3 file trên vào đây. (Home/.gazebo/models/ 3 file trên)

- Để chay được camera AI cần đảm bảo file train_model.py dòng 16 thay bằng đường dẫn tới file của bạn 
```
        self.model = YOLO("/home/ph/catkin_ws/src/slam/custom_yolov85/weights/best.pt")
```

  - file controller.py để mở bảng điều khiển
  - file data.py dùng để chụp ảnh tự động sử dụng để lấy dữ liệu train model (lưu ý đường dẫn)
  - filde detect_cicre.py và detect_depth.py dùng để nhận diện các khối ( không sử dụng trong prj này)
  - lidar.py dùng để theo dõi khoảng cách tới các vật cản trong quá trình human tracking.
  - navigation_fl_human.py dùng để humantrack và né vật cản trong quá trình theo (đang phát triển)
  - rotate_images.py để xoay ảnh giúp train ảnh phù hợp
  - train_model.py sử dụng để human tracking và thay đổi đối tượng track
  - train_yolov8.py sử dụng để train lại dữ liệu khi muốn nâng cấp model (để ý đường dẫn đúng )
