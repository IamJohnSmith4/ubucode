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

    def play_system_sound(self):
        """ฟังก์ชันสำหรับเล่นเสียงระบบ Ubuntu (เสียง Login)"""
        # Path มาตรฐานของเสียงระบบใน Ubuntu
        sound_file = "/usr/share/sounds/ubuntu/stereo/service-login.oga"
        
        if os.path.exists(sound_file):
            rospy.loginfo("--- Target Reached: Playing System Sound ---")
            # ใช้ paplay ในการเล่นไฟล์เสียง .oga
            os.system(f"paplay {sound_file}")
        else:
            rospy.logwarn("System sound file not found! Please check the path.")

    def rotate_test(self, target_deg):
        # แปลงองศาเป็นเรเดียน
        target_rad = math.radians(target_deg)
        start_yaw = self.yaw
        rotated_rad = 0.0
        
        rate = rospy.Rate(20)
        print(f"--- Starting Angular Test: Target {target_deg} Degrees ---")
        
        while not rospy.is_shutdown():
            # คำนวณมุมที่หมุนไปแล้ว (จัดการเรื่องมุม -pi ถึง pi)
            diff = self.yaw - start_yaw
            
            # Normalize angle เพื่อให้ได้ค่าความต่างที่ถูกต้อง
            if diff > math.pi: diff -= 2*math.pi
            if diff < -math.pi: diff += 2*math.pi
            
            rotated_rad = abs(diff)
            remaining_rad = abs(target_rad) - rotated_rad
            
            # จุดหยุด (Tolerance ประมาณ 0.5 องศา)
            if remaining_rad <= 0.01: 
                break
            
            twist = Twist()
            # กำหนดทิศทาง: บวก=หมุนซ้าย(ทวนเข็ม), ลบ=หมุนขวา(ตามเข็ม)
            speed = 0.3 if target_deg > 0 else -0.3
            
            # ระบบลดความเร็วเมื่อใกล้ถึงเป้าหมาย (Mini-Ramping)
            if remaining_rad < 0.2:
                twist.angular.z = speed * (remaining_rad / 0.2)
                # กำหนดความเร็วขั้นต่ำไม่ให้หุ่นหยุดนิ่งก่อนถึงเป้าหมาย
                if abs(twist.angular.z) < 0.1: 
                    twist.angular.z = 0.1 if speed > 0 else -0.1
            else:
                twist.angular.z = speed

            self.velocity_publisher.publish(twist)
            
            print(f"Rotated: {math.degrees(rotated_rad):.2f} / {target_deg} deg")
            rate.sleep()

        # --- ส่วนที่ทำงานหลังจากหมุนครบองศา ---
        # 1. สั่งหยุดหุ่นยนต์ทันที
        self.velocity_publisher.publish(Twist())
        
        # 2. เล่นเสียงระบบแจ้งเตือน
        self.play_system_sound()
        
        print(f"--- Finished! Target: {target_deg} | Measured: {math.degrees(rotated_rad):.2f} deg ---")

if __name__ == "__main__":
    try:
        rospy.init_node('odom_rotate_test_node')
        tester = OdomRotateTest()
        
        # ทดสอบหมุน 90 องศา (ใส่ค่าลบถ้าต้องการหมุนตามเข็ม)
        tester.rotate_test(90.0) 
        
    except rospy.ROSInterruptException:
        pass