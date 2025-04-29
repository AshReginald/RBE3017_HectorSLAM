# RBE3017_HectorSLAM
Bài tập cuối kỳ môn Lập trình Robot với ROS - Hector SLAM

CÁC THAO TÁC QUÉT VÀ ĐIỀU HƯỚNG ROBOT VỚI GÓI "tk8":
1. Biên dịch và cấu hình môi trường
Trước tiên, người dùng cần biên dịch workspace và cấu hình môi trường ROS như sau:

```bash
catkin_make
source devel/setup.bash
```

3. Khởi động môi trường mô phỏng
Chọn một trong các file .launch dưới đây để khởi động môi trường mong muốn:
```bash
roslaunch tk8 launch.launch
```

```bash
roslaunch tk8 launch1.launch
```

```bash
roslaunch tk8 launch3.launch
```

```bash
roslaunch tk8 launch4.launch
```

```bash
roslaunch tk8 launch5.launch
```

4. Điều khiển robot để quét bản đồ
Mở một terminal mới và chạy lệnh điều khiển bằng bàn phím:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Sử dụng các phím điều hướng để di chuyển robot và thực hiện quét bản đồ.

4. Thực hiện điều hướng (Navigation)
Khởi động môi trường điều hướng bằng lệnh sau:
```bash
roslaunch tk8 launch.launch
```

Mở một terminal khác và chạy:
```bash
roslaunch tk8 navigation.launch
```

Trong giao diện RViz, sử dụng công cụ 2D Nav Goal để chọn điểm đích di chuyển cho robot.


5. Quét bản đồ tự động
a. Cài đặt các thư viện cần thiết
Đảm bảo rằng các thư viện scipy và numpy đã được cài đặt. Nếu chưa, thực hiện:
```bash
pip3 install spicy numpy
```

b. Cấp quyền thực thi cho tập tin điều khiển quét tự động
Cấp quyền chạy cho file explore_robot.py (nếu chưa cấp):
```bash
chmod +x explore_robot.py
```
c. Xóa các bản đồ cũ
Truy cập vào thư mục:
catkin_ws/src/tk8/maps

Và xóa hai tệp bản đồ cũ (nếu có):

turtle1.pgm

turtle1.yaml

d. Khởi động chế độ quét tự động
Mở terminal và chạy:
```bash
roslaunch tk8 explore.launch
```
