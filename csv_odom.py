#!/usr/bin/env python3
import rospy
import math
import csv
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion

class RobotLogger:
    def __init__(self):
        rospy.init_node('robot_move_logger_node')
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.is_navigating = True
        self.data_log = [] 

        # Topic มาตรฐาน TurtleBot2/Kobuki
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.reset_odom_pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        rospy.loginfo("ระบบ: กำลังเชื่อมต่อ Odometry...")
        rospy.wait_for_message('/odom', Odometry)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(quaternion)

    def move_forward(self, distance, bias=0.0):
        # 1. พยายามรีเซ็ต Odom (ส่งคำสั่งเผื่อไว้)
        rospy.loginfo("ระบบ: ส่งคำสั่งรีเซ็ต Odometry...")
        for _ in range(5):
            self.reset_odom_pub.publish(Empty())
            rospy.sleep(0.1)
        
        # รอให้ระบบนิ่ง 1 วินาที
        rospy.sleep(1.0)
        
        # 2. ตั้งจุดเริ่มต้น ณ ตำแหน่งปัจจุบัน (Relative Origin) 
        start_x, start_y = self.x, self.y
        target_yaw = self.yaw 
        
        rate = rospy.Rate(20)
        LINEAR_SPEED = 0.50
        current_linear_speed = 0.05
        accel = 0.01 
        min_speed = 0.08
        decel_dist = 0.4
        
        start_time = rospy.get_time()
        rospy.loginfo(f"ระบบ: เริ่มเดินหน้า {distance} ม. จากจุด ({start_x:.2f}, {start_y:.2f}) [ไม่ใช้ PID]")

        while not rospy.is_shutdown() and self.is_navigating:
            # คำนวณระยะที่ "ขยับไปแล้ว" เทียบกับจุดที่เริ่มเดินจริงๆ
            traveled = math.sqrt((self.x - start_x)**2 + (self.y - start_y)**2)
            remaining_dist = distance - traveled
            
            if traveled >= distance: 
                break
            
            # ระบบควบคุมความเร็ว (Speed Ramping)
            if remaining_dist > decel_dist:
                if current_linear_speed < LINEAR_SPEED:
                    current_linear_speed += accel
            else:
                # ผ่อนความเร็วเมื่อใกล้ถึงเป้าหมาย
                current_linear_speed = max(min_speed, (remaining_dist / decel_dist) * LINEAR_SPEED)
            
            # คำนวณ Yaw Error (ยังคงคำนวณไว้เพื่อใช้บันทึก Log การเบี่ยงเบนเท่านั้น)
            error_yaw = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))
            
            twist = Twist()
            twist.linear.x = current_linear_speed
            # ปิดการใช้ PID ควบคุมทิศทาง หมุนตามค่า bias เท่านั้น (ปกติคือ 0)
            twist.angular.z = bias 
            
            self.data_log.append([
                round(rospy.get_time() - start_time, 3), # เวลา
                round(traveled, 4),                      # ระยะทาง
                round(current_linear_speed, 3),          # ความเร็ว
                round(error_yaw, 4)                      # Error มุม
            ])

            self.pub.publish(twist)
            print(f"กำลังเดิน: {traveled:.3f} / {distance} ม. (V: {current_linear_speed:.2f})", end='\r')
            rate.sleep()
            
        self.pub.publish(Twist()) # หยุดหุ่น
        rospy.loginfo("\nระบบ: ถึงจุดหมายแล้ว! กำลังสรุปข้อมูล...")
        self.save_results(distance)

    def save_results(self, target_dist):
        # บันทึก CSV
        csv_file = "ผลการทดสอบ_เดินตรง_ไม่ใช้PID.csv"
        with open(csv_file, mode='w', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f)
            writer.writerow(['เวลา (วินาที)', 'ระยะที่วัดได้ (เมตร)', 'ความเร็ว (m/s)', 'Error มุม (rad)'])
            writer.writerows(self.data_log)
        
        # คำนวณความคลาดเคลื่อน
        final_dist = self.data_log[-1][1]
        error_cm = (target_dist - final_dist) * 100
        
        print(f"\n--- [ สรุปผลการทดลอง (ไม่ใช้ PID) ] ---")
        print(f"เป้าหมาย: {target_dist} เมตร")
        print(f"ระยะจาก Odom: {final_dist:.4f} เมตร")
        print(f"ความคลาดเคลื่อน: {error_cm:.2f} เซนติเมตร")

        # สร้างกราฟสรุปผล
        t_vals = [d[0] for d in self.data_log]
        s_vals = [d[2] for d in self.data_log]
        d_vals = [d[1] for d in self.data_log]

        plt.figure(figsize=(10, 8))
        plt.subplot(2, 1, 1)
        plt.plot(t_vals, s_vals, 'r', label='Speed (m/s)') # เปลี่ยนสีเพื่อแยกความต่าง
        plt.title('Robot Movement Profile (No PID)')
        plt.grid(True); plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(t_vals, d_vals, 'orange', label='Distance (m)')
        plt.xlabel('Time (s)'); plt.grid(True); plt.legend()

        plt.savefig('graph_output_no_pid.png')
        rospy.loginfo(f"บันทึกไฟล์ '{csv_file}' และ 'graph_output_no_pid.png' สำเร็จ")

if __name__ == '__main__':
    try:
        logger = RobotLogger()
        logger.move_forward(3.0) # ทดสอบที่ 3 เมตร
    except rospy.ROSInterruptException:
        pass