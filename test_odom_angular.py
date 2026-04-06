#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion

class SimpleRobotRotate:
    def __init__(self):
        self.yaw = 0.0
        rospy.init_node('simple_rotate_node')

        # การเชื่อมต่อ Topic มาตรฐาน
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.reset_pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.loginfo("รอดึงข้อมูลจาก Odometry...")
        rospy.wait_for_message('/odom', Odometry)

    def odom_callback(self, msg):
        # แปลงค่าจาก Odom เป็นมุม Yaw (หน่วยเรเดียน)
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(quaternion)

    def rotate(self, angle_deg):
        # 1. รีเซ็ตค่าทุกอย่างให้เริ่มจาก 0
        rospy.loginfo("กำลังรีเซ็ตค่า Odometry...")
        for _ in range(5):
            self.reset_pub.publish(Empty())
            rospy.sleep(0.1)
        rospy.sleep(1.0) # รอให้ระบบนิ่ง

        # 2. ตั้งเป้าหมาย
        target_rad = math.radians(angle_deg)
        rate = rospy.Rate(20)
        
        rospy.loginfo(f"เริ่มหมุนไปที่ {angle_deg} องศา (วัดจาก Odom ตรงๆ)")

        while not rospy.is_shutdown():
            # ดึงค่ามุมปัจจุบัน (ใช้ abs เพื่อให้เช็คเงื่อนไขง่ายๆ)
            current_rotated = abs(self.yaw)

            # แสดงค่าที่หุ่นยนต์อ่านได้ทุกรอบ
            print(f"Odom อ่านค่าได้: {math.degrees(current_rotated):.2f}° / เป้าหมาย: {angle_deg}°", end='\r')

            # 3. เงื่อนไขการหยุด: ถ้าค่า Yaw ถึงเป้าหมายที่สั่งให้หยุดทันที
            if current_rotated >= abs(target_rad):
                break

            # 4. สั่งหมุนด้วยความเร็วคงที่
            t = Twist()
            t.angular.z = 0.3 if angle_deg > 0 else -0.3
            self.pub.publish(t)
            
            rate.sleep()

        # 5. หยุดหุ่นยนต์
        self.pub.publish(Twist())
        print(f"\n✅ หยุดแล้ว! ค่าสุดท้ายใน Odom: {math.degrees(self.yaw):.2f}°")

if __name__ == '__main__':
    try:
        robot = SimpleRobotRotate()
        # ตัวอย่าง: สั่งหมุน 90 องศา
        robot.rotate(90.0)
    except rospy.ROSInterruptException:
        pass
