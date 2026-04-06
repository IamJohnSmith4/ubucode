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
        # 1. เตรียมตัวแปร
        target_rad = math.radians(target_deg)
        accumulated_yaw = 0.0        # มุมสะสมรวม (เรเดียน)
        last_yaw = self.yaw          # บันทึกค่ามุมครั้งล่าสุด
        
        rate = rospy.Rate(20)
        rospy.loginfo(f"--- Starting Pure Odom Rotation: {target_deg} deg ---")
        
        while not rospy.is_shutdown():
            # 2. คำนวณส่วนต่างของมุม (Delta Yaw)
            current_yaw = self.yaw
            delta_yaw = current_yaw - last_yaw
            
            # 3. จัดการเรื่องมุมพลิก (Angle Wrap-around) 
            # ถ้า delta กระโดดเกิน pi แสดงว่ามุมพลิกจาก 180 ไป -180 หรือในทางกลับกัน
            if delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            elif delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi
            
            # 4. สะสมมุมที่หมุนไป
            accumulated_yaw += delta_yaw
            last_yaw = current_yaw
            
            # 5. คำนวณระยะที่เหลือ (ใช้ค่า Absolute เพื่อให้หมุนได้ทั้งซ้าย/ขวา)
            remaining_rad = abs(target_rad) - abs(accumulated_yaw)
            
            # จุดหยุด (Tolerance 0.01 rad ประมาณ 0.5 องศา)
            if remaining_rad <= 0.01:
                break
                
            # 6. ควบคุมความเร็ว (Ramping)
            twist = Twist()
            speed_dir = 0.3 if target_deg > 0 else -0.3
            
            if remaining_rad < 0.2: # ช่วงผ่อนความเร็ว
                twist.angular.z = speed_dir * (remaining_rad / 0.2)
                if abs(twist.angular.z) < 0.1: # ความเร็วขั้นต่ำ
                    twist.angular.z = 0.1 if speed_dir > 0 else -0.1
            else:
                twist.angular.z = speed_dir
                
            self.velocity_publisher.publish(twist)
            
            print(f"Accumulated: {math.degrees(accumulated_yaw):.2f} / {target_deg} deg", end='\r')
            rate.sleep()

        # หยุดหุ่นและเล่นเสียง
        self.velocity_publisher.publish(Twist())
        self.play_system_sound()
        print(f"\n Finished! Measured: {math.degrees(accumulated_yaw):.2f} deg")

if __name__ == "__main__":
    try:
        rospy.init_node('odom_rotate_test_node')
        tester = OdomRotateTest()
        
        # ทดสอบหมุน 90 องศา (ใส่ค่าลบถ้าต้องการหมุนตามเข็ม)
        tester.rotate_test(90.0) 
        
    except rospy.ROSInterruptException:
        pass
