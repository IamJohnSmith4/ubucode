#!/usr/bin/env python3
import rospy
import math
import csv
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion

# --- คลาส PID ตามค่าพารามิเตอร์ที่กำหนด ---
class PID:
    def __init__(self, kp, ki, kd, min_val, max_val):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.min_val, self.max_val = min_val, max_val
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.last_error = error
        return max(self.min_val, min(self.max_val, output))

class RobotLogger:
    def __init__(self):
        rospy.init_node('robot_move_logger_node')
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.is_navigating = True
        self.data_log = [] 

        # Topic สำหรับ TurtleBot2/Kobuki
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.reset_odom_pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # ตั้งค่า PID ตามผลการทดสอบ
        self.pid_straight = PID(kp=1.8, ki=0.005, kd=0.1, min_val=-0.4, max_val=0.4)
        
        rospy.loginfo("กำลังรอดึงข้อมูลจาก Odometry...")
        rospy.wait_for_message('/odom', Odometry)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(quaternion)

    def move_forward(self, distance, bias=0.0):
        # 1. การรีเซ็ต Odom ให้เป็น 0 อย่างสมบูรณ์ (แก้ปัญหาหุ่นไม่วิ่งรอบสอง)
        rospy.loginfo("กำลังรีเซ็ตค่าตำแหน่ง (Odometry)...")
        for _ in range(10):
            self.reset_odom_pub.publish(Empty())
            rospy.sleep(0.1)
            
        reset_timeout = rospy.get_time() + 3.0 
        rospy.loginfo("รอการยืนยันค่าศูนย์จากระบบ...")
        
        # รอจนกว่าค่า x และ y จะใกล้ศูนย์จริงๆ
        while abs(self.x) > 0.05 or abs(self.y) > 0.05:
            if rospy.get_time() > reset_timeout:
                rospy.logwarn("Timeout: ระบบ Odom ไม่เป็นศูนย์ในเวลาที่กำหนด จะเริ่มเคลื่อนที่ต่อโดยใช้ค่าปัจจุบัน")
                break
            rospy.sleep(0.1)
        
        start_x, start_y = self.x, self.y
        target_yaw = self.yaw 
        rate = rospy.Rate(20)
        
        LINEAR_SPEED = 0.50
        current_linear_speed = 0.05
        accel = 0.008
        min_speed = 0.07
        decel_dist = 0.4
        
        self.pid_straight.integral = 0.0
        self.pid_straight.last_error = 0.0
        
        start_time = rospy.get_time()
        rospy.loginfo(f"เริ่มเคลื่อนที่: เป้าหมาย {distance} เมตร (Bias: {bias})")

        while not rospy.is_shutdown() and self.is_navigating:
            # คำนวณระยะทางที่เคลื่อนที่ได้จริง
            traveled = math.sqrt((self.x - start_x)**2 + (self.y - start_y)**2)
            remaining_dist = distance - traveled
            
            if traveled >= distance: 
                break
            
            # ระบบ Speed Ramping
            if remaining_dist > decel_dist:
                if current_linear_speed < LINEAR_SPEED:
                    current_linear_speed += accel
                else:
                    current_linear_speed = LINEAR_SPEED
            else:
                current_linear_speed = max(min_speed, (remaining_dist / decel_dist) * LINEAR_SPEED)
            
            # คำนวณมุมที่เบี่ยงเบน (Yaw Error)
            error_yaw = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))
            
            twist = Twist()
            twist.linear.x = current_linear_speed
            twist.angular.z = self.pid_straight.compute(error_yaw, 0.05) + bias
            
            # บันทึกข้อมูล (ภาษาไทยสำหรับ CSV)
            self.data_log.append([
                round(rospy.get_time() - start_time, 3), # เวลา
                round(traveled, 4),                      # ระยะทาง
                round(current_linear_speed, 3),          # ความเร็ว
                round(error_yaw, 4)                       # ค่า Error มุม
            ])

            self.pub.publish(twist)
            print(f"ระยะที่ได้: {traveled:.3f} ม. | ความเร็ว: {current_linear_speed:.3f} ม./วิ", end='\r')
            rate.sleep()
            
        self.pub.publish(Twist())
        rospy.loginfo("\nถึงจุดหมายแล้ว! กำลังบันทึกข้อมูลและสร้างกราฟ...")
        self.save_data(distance)

    def save_data(self, target_dist):
        # บันทึกเป็น CSV (หัวข้อภาษาไทย)
        filename = "ผลการทดสอบ_เดินตรง.csv"
        with open(filename, mode='w', newline='', encoding='utf-8-sig') as file:
            writer = csv.writer(file)
            writer.writerow(['เวลา (วินาที)', 'ระยะทางที่ได้ (เมตร)', 'ความเร็ว (ม/วิ)', 'ความคลาดเคลื่อนมุม (เรเดียน)'])
            writer.writerows(self.data_log)
        
        # แสดงสรุปผลในสไตล์ตารางการทดลอง
        final_odom = self.data_log[-1][1]
        error_cm = (target_dist - final_odom) * 100
        print(f"\n--- สรุปผลการทดลอง ---")
        print(f"ระยะทางที่สั่ง: {target_dist} ม.")
        print(f"ค่าจาก Odom: {final_odom:.4f} ม.")
        print(f"ความคลาดเคลื่อน: {error_cm:.2f} ซม.")

        # สร้างกราฟ (ใช้ภาษาอังกฤษใน Label เพื่อป้องกันตัวอักษรไม่แสดงในบางระบบ)
        times = [d[0] for d in self.data_log]
        speeds = [d[2] for d in self.data_log]
        distances = [d[1] for d in self.data_log]

        plt.figure(figsize=(10, 8))
        plt.subplot(2, 1, 1)
        plt.plot(times, speeds, 'b-', linewidth=2, label='Linear Speed (m/s)')
        plt.title('Robot Movement Analysis')
        plt.ylabel('Speed (m/s)')
        plt.grid(True)
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(times, distances, 'g-', linewidth=2, label='Traveled Distance (m)')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (m)')
        plt.grid(True)
        plt.legend()

        plt.savefig('graph_output.png')
        rospy.loginfo("บันทึกไฟล์ 'ผลการทดสอบ_เดินตรง.csv' และ 'graph_output.png' เรียบร้อย")

if __name__ == '__main__':
    try:
        logger = RobotLogger()
        # ทดสอบเดินหน้า 3 เมตรตามตารางบันทึกผล
        logger.move_forward(3.0)
    except rospy.ROSInterruptException:
        pass
