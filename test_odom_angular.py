#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class SimpleRotate90:
    def __init__(self):
        self.current_yaw = 0.0
        rospy.init_node('rotate_90_test_node')
        
        # Publisher สำหรับสั่งความเร็ว (ใช้ Topic มาตรฐานของ TurtleBot2/Kobuki)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        
        # Subscriber รับค่า Odometry
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        rospy.loginfo("รอดึงค่าจาก /odom...")
        rospy.wait_for_message('/odom', Odometry)

    def odom_callback(self, msg):
        # แปลง Quaternion เป็น Yaw (หน่วยเรเดียน)
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw_rad) = euler_from_quaternion(quaternion)
        
        # แปลงเป็นองศาเพื่อให้ดูง่ายตอนบันทึกผล
        self.current_yaw = math.degrees(yaw_rad)

    def execute(self):
        rate = rospy.Rate(20)
        
        # 1. บันทึกมุมเริ่มต้น และตั้งเป้าหมาย (+ 90 องศา)
        start_yaw = self.current_yaw
        target_yaw = start_yaw + 90.0
        
        rospy.loginfo(f"เริ่มหมุนจาก: {start_yaw:.2f} -> เป้าหมาย: {target_yaw:.2f}")

        while not rospy.is_shutdown():
            # 2. เช็คเงื่อนไขหยุด: ถ้า Yaw ปัจจุบัน >= เป้าหมาย ให้หยุดทันที
            # (ตามที่คุณต้องการเพื่อวัด Error จริงอีกรอบ)
            if self.current_yaw >= target_yaw:
                break
            
            # 3. สั่งหมุนด้วยความเร็วคงที่ (ใช้ความเร็วต่ำ 0.2 เพื่อให้หยุดได้แม่นยำ)
            twist = Twist()
            twist.angular.z = 0.2 
            self.pub.publish(twist)
            
            print(f"Yaw ปัจจุบัน: {self.current_yaw:.2f} / เป้าหมาย: {target_yaw:.2f}", end='\r')
            rate.sleep()

        # 4. สั่งหยุดหุ่นยนต์
        self.pub.publish(Twist())
        rospy.loginfo(f"หยุดแล้ว! ค่าที่ Odom รายงาน: {self.current_yaw:.2f} deg")
        print("\n--- บันทึกค่า Odom นี้ลงตาราง และวัดมุมจริงที่ตัวหุ่นได้เลยครับ ---")

if __name__ == '__main__':
    try:
        tester = SimpleRotate90()
        tester.execute()
    except rospy.ROSInterruptException:
        pass
