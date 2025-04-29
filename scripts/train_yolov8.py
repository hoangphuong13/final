from ultralytics import YOLO

# 1. Chọn model (có thể là yolov8n, yolov8s, yolov8m, yolov8l, yolov8x)
model = YOLO('yolov8n.pt')  # n = nano (nhẹ), bạn có thể đổi sang yolov8s.pt nếu cần mạnh hơn

# 2. Bắt đầu train
model.train(
    data='/home/ph/catkin_ws/src/slam/dataset/Person Recognition.v1i.yolov8/data.yaml',  # đường dẫn tới file yaml
    epochs=100,              # số epochs train
    imgsz=640,               # kích thước ảnh resize về (default 640)
    batch=16,                # batch size
    name='custom_yolov8',    # tên fo):lder output trong runs/detect/
    workers=4                # số luồng load data
)
    