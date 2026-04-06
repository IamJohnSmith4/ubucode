#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class SimpleRotate:
    def __init__(self):
        self.yaw = 0.0
        # รับค่าจาก /odom
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # ส่งความเร็วไปที่หุ่น (เปลี่ยน topic ตามรุ่นหุ่นของคุณ เช่น /cmd_vel)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        
        rospy.loginfo("Waiting for odom...")
        rospy.wait_for_message('/odom', Odometry)

    def odom_callback(self, msg):
        # แปลง Quaternion เป็น Yaw
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(quaternion)

    def rotate_90_test(self):
        rate = rospy.Rate(20)
        start_yaw = self.yaw
        # 90 องศา = 1.5708 เรเดียน
        target_rad = 1.5708 
        
        rospy.loginfo("Start Rotating 90 degrees...")

        while not rospy.is_shutdown():
            # คำนวณระยะที่หมุนไปแล้ว (ใช้ abs เพื่อให้รองรับทั้งซ้ายและขวา)
            rotated_rad = abs(self.yaw - start_yaw)
            
            # จัดการกรณีมุมพลิกข้ามช่วง -pi ไป pi
            if rotated_rad > math.pi:
                rotated_rad = abs(rotated_rad - 2 * math.pi)

            # เงื่อนไขหยุด: ถ้าหมุนไปจนถึงหรือเกิน 90 องศาตามที่ Odom บอก
            if rotated_rad >= target_rad:
                break

            # สั่งหมุนด้วยความเร็วคงที่ (เช่น 0.2 rad/s)
            twist = Twist()
            twist.angular.z = 0.2 
            self.pub.publish(twist)
            
            print(f"Current Rotated: {math.degrees(rotated_rad):.2f} / 90.00 deg", end='\r')
            rate.sleep()

        # หยุดหุ่นทันทีเพื่อวัดผลจริง
        self.pub.publish(Twist())
        rospy.loginfo(f"Finished! Odom reported: {math.degrees(rotated_rad):.2f} deg")

if __name__ == "__main__":
    try:
        rospy.init_node('simple_rotate_test')
        tester = SimpleRotate()
        tester.rotate_90_test()
    except rospy.ROSInterruptException:
        pass
