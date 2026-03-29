#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class OdomRotateTest:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Subscriber รับค่าจาก /odom เพื่ออัปเดตค่า self.yaw
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        
        rospy.loginfo("Waiting for odom data...")
        rospy.wait_for_message('/odom', Odometry)
        rospy.sleep(1)

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(quaternion)

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
            # Normalize angle
            if diff > math.pi: diff -= 2*math.pi
            if diff < -math.pi: diff += 2*math.pi
            
            rotated_rad = abs(diff)
            remaining_rad = abs(target_rad) - rotated_rad
            
            if remaining_rad <= 0.01: # จุดหยุด (Tolerance ประมาณ 0.5 องศา)
                break
            
            # กำหนดความเร็วในการหมุน (ใช้ความเร็วต่ำเพื่อให้วัดค่าได้แม่นยำ)
            twist = Twist()
            # ถ้า target เป็นบวกหมุนซ้าย (ทวนเข็ม) ถ้าลบหมุนขวา (ตามเข็ม)
            speed = 0.3 if target_deg > 0 else -0.3
            
            # ลดความเร็วลงเมื่อใกล้ถึงเป้าหมาย (Mini-Ramping)
            if remaining_rad < 0.2:
                twist.angular.z = speed * (remaining_rad / 0.2)
                if abs(twist.angular.z) < 0.1: twist.angular.z = 0.1 if speed > 0 else -0.1
            else:
                twist.angular.z = speed

            self.velocity_publisher.publish(twist)
            
            print(f"Rotated: {math.degrees(rotated_rad):.2f} / {target_deg} deg | Odom Yaw: {math.degrees(self.yaw):.2f}")
            rate.sleep()

        # หยุดหุ่นยนต์
        self.velocity_publisher.publish(Twist())
        print(f"--- Finished! Target: {target_deg} | Odom Measured: {math.degrees(rotated_rad):.2f} deg ---")
        print("Please use a protractor or floor markings to measure the actual angle.")

if __name__ == "__main__":
    try:
        rospy.init_node('odom_rotate_test_node')
        tester = OdomRotateTest()
        
        # ทดสอบหมุน 90 องศา (ใส่ค่าลบถ้าต้องการหมุนตามเข็ม)
        tester.rotate_test(90.0) 
        
    except rospy.ROSInterruptException:
        pass