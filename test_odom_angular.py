#!/usr/bin/env python3
import rospy
import os  # สำหรับเรียกใช้คำสั่งเล่นเสียง
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class OdomRotateTest:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Subscriber รับค่าจาก /odom เพื่ออัปเดตมุม Yaw ปัจจุบัน
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        
        rospy.loginfo("Waiting for odom data...")
        rospy.wait_for_message('/odom', Odometry)
        
        # หยุดหุ่นยนต์ให้สนิทก่อนเริ่ม
        self.velocity_publisher.publish(Twist())
        rospy.sleep(1)

    def odom_callback(self, msg):
        """รับค่าจาก Odom และแปลง Quaternion เป็นมุม Euler (Yaw)"""
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(quaternion)

    def rotate(self, angle_rad):
        # 1. คำนวณหาเป้าหมาย (ใช้ atan2 เพื่อจัดการช่วงมุม -pi ถึง pi ให้ถูกต้อง)
        target_yaw = math.atan2(math.sin(self.yaw + angle_rad), math.cos(self.yaw + angle_rad))
        rate = rospy.Rate(30)
        
        # ตั้งค่าพารามิเตอร์สำหรับการควบคุมความเร็ว
        max_ang_vel = 0.5    # ความเร็วสูงสุดที่ต้องการ (rad/s)
        min_ang_vel = 0.1    # ความเร็วขั้นต่ำเพื่อให้หุ่นไม่หยุดนิ่งก่อนถึงเป้าหมาย
        ramp_threshold = 0.2  # ระยะห่างจากเป้าหมาย (เรเดียน) ที่จะเริ่มลดความเร็ว
        
        # สมมติว่า is_navigating คือตัวแปรสถานะในคลาสของคุณ
        while not rospy.is_shutdown() and self.is_navigating:
            # 2. คำนวณส่วนต่าง (Error) โดยใช้ atan2 เพื่อหาทางที่สั้นที่สุดเสมอ
            error = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))
            
            # 3. จุดหยุด (Tolerance) ประมาณ 0.3 องศา
            if abs(error) < 0.005: 
                break
            
            # 4. คำนวณความเร็วแบบ Mini-Ramping (แทนการใช้ PID)
            direction = 1 if error > 0 else -1
            
            if abs(error) < ramp_threshold:
                # ช่วงผ่อนความเร็ว: ลดความเร็วลงตามสัดส่วนของระยะที่เหลือ
                speed = max_ang_vel * (abs(error) / ramp_threshold)
                # คุมไม่ให้ความเร็วต่ำเกินไปจนหุ่นนิ่ง (Dead zone)
                speed = max(speed, min_ang_vel)
            else:
                # ช่วงความเร็วคงที่
                speed = max_ang_vel

            twist = Twist()
            twist.angular.z = speed * direction
            self.pub.publish(twist)
            
            print(f"Current Yaw: {self.yaw:.3f} | Error: {error:.3f}", end='\r')
            rate.sleep()
            
        # 5. สั่งหยุดหุ่นยนต์ให้สนิท
        self.pub.publish(Twist())
        rospy.sleep(0.3)

if __name__ == "__main__":
    try:
        rospy.init_node('odom_rotate_test_node')
        tester = OdomRotateTest()
        
        # ทดสอบหมุน 90 องศา (ใส่ค่าลบถ้าต้องการหมุนตามเข็ม)
        tester.rotate_test(90.0) 
        
    except rospy.ROSInterruptException:
        pass
