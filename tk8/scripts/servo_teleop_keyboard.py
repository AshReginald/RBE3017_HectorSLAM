#!/usr/bin/env python3

import rospy
import sys
import tty
import termios
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

# Tên các khớp 
JOINT_NAMES = ["base_servo_joint", "end_servo_joint"]

# Topic command của controller (bao gồm cả namespace)
CONTROLLER_TOPIC = "/servo_controller/command"

# --- Cài đặt điều khiển ---
BASE_SERVO_STEP = 0.1  # Radian thay đổi mỗi lần nhấn phím cho khớp base
END_SERVO_STEP = 0.1   # Radian thay đổi mỗi lần nhấn phím cho khớp end

# Giới hạn khớp
BASE_SERVO_MIN = 0.0
BASE_SERVO_MAX = 3.14159
END_SERVO_MIN = 0.0
END_SERVO_MAX = 3.14159

# Thời gian để đạt được điểm quỹ đạo tiếp theo (giây)
TIME_TO_REACH_POINT = 0.2

# --- Hàm đọc phím ---
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno(), termios.TCSANOW) # Sửa lại hàm gọi cho setraw
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

# --- Hàm chính ---
def servo_teleop():
    rospy.init_node('servo_teleop_keyboard', anonymous=True)

    # Publisher gửi lệnh quỹ đạo
    pub = rospy.Publisher(CONTROLLER_TOPIC, JointTrajectory, queue_size=10)

    # Vị trí hiện tại mong muốn (khởi tạo ở giữa tầm)
    current_base_pos = 0.0
    current_end_pos = 0.0

    # Tạo message mẫu để tái sử dụng
    traj = JointTrajectory()
    traj.header = Header()
    traj.joint_names = JOINT_NAMES

    # Gửi vị trí ban đầu khi khởi động
    point = JointTrajectoryPoint()
    point.positions = [current_base_pos, current_end_pos]
    point.time_from_start = rospy.Duration(1.0) # Cho 1 giây để đến vị trí ban đầu
    traj.points = [point]
    traj.header.stamp = rospy.Time.now()
    pub.publish(traj)
    rospy.sleep(1.0) # Chờ robot đến vị trí ban đầu

    print("Điều khiển Servo:")
    print("  a: left (base)")
    print("  d: right (base)")
    print("  w: up (end)")
    print("  s: down (end)")
    print("  1: quit")
    print("  2: reset")

    rate = rospy.Rate(10) # Tần suất gửi lệnh 

    while not rospy.is_shutdown():
        key = get_key()

        moved = False # Cờ để kiểm tra xem có cần gửi lệnh không

        if key == 'a':
            current_base_pos -= BASE_SERVO_STEP
            moved = True
        elif key == 'd':
            current_base_pos += BASE_SERVO_STEP
            moved = True
        elif key == 'w':
            current_end_pos -= END_SERVO_STEP
            moved = True
        elif key == 's':
            current_end_pos += END_SERVO_STEP
            moved = True
        elif key == '1':
            print("quit")
            break
        elif key == '2':
            print("reset")
            current_base_pos = 0.0
            current_end_pos = 0.0
            moved = True
        else:
            # Không làm gì nếu nhấn phím khác
            continue

        # --- Áp dụng giới hạn khớp ---
        if current_base_pos > BASE_SERVO_MAX:
            current_base_pos = BASE_SERVO_MAX
        elif current_base_pos < BASE_SERVO_MIN:
            current_base_pos = BASE_SERVO_MIN

        if current_end_pos > END_SERVO_MAX:
            current_end_pos = END_SERVO_MAX
        elif current_end_pos < END_SERVO_MIN:
            current_end_pos = END_SERVO_MIN

        # --- Chỉ gửi lệnh nếu có sự thay đổi ---
        if moved:
            # Tạo điểm quỹ đạo mới
            point = JointTrajectoryPoint()
            point.positions = [current_base_pos, current_end_pos]
            point.time_from_start = rospy.Duration(TIME_TO_REACH_POINT)

            # Cập nhật và gửi message
            traj.points = [point] # Quỹ đạo chỉ có 1 điểm đích
            traj.header.stamp = rospy.Time.now() # Cập nhật thời gian
            pub.publish(traj)

            rospy.loginfo("Đang gửi: Base=%.2f rad, End=%.2f rad", current_base_pos, current_end_pos)


if __name__ == '__main__':
    try:
        original_settings = termios.tcgetattr(sys.stdin)
        servo_teleop()
    except rospy.ROSInterruptException:
        print("ROS bị ngắt.")
    except Exception as e:
        print(f"Lỗi xảy ra: {e}")
    finally:
        # Khôi phục cài đặt terminal khi kết thúc (quan trọng!)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        print("Đã khôi phục cài đặt terminal.")
