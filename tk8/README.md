**Mid_term_ros**

Thiết kế một robot gồm có xe và tay máy. Xe được điều khiển bằng cơ chế điều hướng Ackermann Steering (về sau đổi thành Differential Drive). Tay máy hai khớp gồm 2 khớp quay. Trên robot có gắn ba loại cảm biến: Lidar, Camera và Encoder.

Yêu cầu các gói

ROS Noetic

Gazebo

RViz

Các gói điều khiển bổ sung trong gazebo-ros và rviz

robot_state_publisher

joint_state_publisher_gui

gazebo_ros

rviz

controller_manager

joint_state_controller

position_controllers


**Cách cài đặt**

**1. Tạo package và source về nguồn**

cd catkin_ws

catkin_make

source devel/setup.bash

Đảm bảo tạo quyền truy cập cho các file tk8/scripts

chmod +x servo_teleop_keyboard.py

Cài đặt bổ sung các package nếu chưa có

**2. Khởi chạy mô phỏng và điều khiển**

roslaunch tk8 launch.launch

mở terminal mới và chạy rosrun teleop_twist_keyboard teleop_twist_keyboard.py để điều khiển di chuyển

tiếp tục mở 1 terminal mới khác và chạy rosrun tk8 servo_teleop_keyboard.py để điều khiển tay máy

**3. Đọc riêng các giá trị cảm biến**

rostopic echo /scan

rostopic echo /rrbot/camera1/image_raw

rostopic echo /joint_states

rostopic echo /odom

rostopic echo /cmd_vel

