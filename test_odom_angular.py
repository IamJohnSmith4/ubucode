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

        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.reset_pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.loginfo("รอดึงข้อมูลจาก Odometry...")
        rospy.wait_for_message('/odom', Odometry)

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(quaternion)

    def rotate(self, angle_deg):
        # 1. รีเซ็ตค่า Odometry
        rospy.loginfo("กำลังรีเซ็ตค่า Odometry...")
        for _ in range(5):
            self.reset_pub.publish(Empty())
            rospy.sleep(0.1)
        rospy.sleep(1.0) 

        # 2. เตรียมตัวแปรสำหรับ "มุมสะสม"
        target_rad = math.radians(angle_deg)
        accumulated_yaw = 0.0
        last_yaw = self.yaw  # บันทึกค่ามุมครั้งล่าสุด
        
        rate = rospy.Rate(20)
        rospy.loginfo(f"เริ่มหมุนไปที่ {angle_deg} องศา (ใช้ระบบมุมสะสม)")

        while not rospy.is_shutdown():
            # 3. คำนวณส่วนต่างของมุม (Delta)
            current_yaw = self.yaw
            delta_yaw = current_yaw - last_yaw
            
            # จัดการเรื่องมุมพลิก (Wrap-around) จาก 180 ไป -180 หรือกลับกัน
            if delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            elif delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi
            
            # สะสมค่ามุมที่หมุนไปจริง
            accumulated_yaw += delta_yaw
            last_yaw = current_yaw

            # แสดงค่าสะสมให้เห็น (จะเห็นว่ามันเกิน 180 หรือ 360 ได้แล้ว)
            print(f"หมุนไปแล้ว: {math.degrees(accumulated_yaw):.2f}° / เป้าหมาย: {angle_deg}°", end='\r')

            # 4. เงื่อนไขการหยุด: เช็คค่า Absolute ของมุมสะสมเทียบกับเป้าหมาย
            if abs(accumulated_yaw) >= abs(target_rad):
                break

            # 5. สั่งหมุน
            t = Twist()
            # หมุนทวนเข็มถ้าองศาเป็นบวก, ตามเข็มถ้าองศาเป็นลบ
            t.angular.z = 0.4 if angle_deg > 0 else -0.4
            self.pub.publish(t)
            
            rate.sleep()

        # 6. หยุดหุ่นยนต์
        self.pub.publish(Twist())
        print(f"\n✅ หยุดแล้ว! มุมสะสมรวมทั้งหมด: {math.degrees(accumulated_yaw):.2f}°")

if __name__ == '__main__':
    try:
        robot = SimpleRobotRotate()
        # ทดสอบหมุน 360 องศาได้เลยครับ
        robot.rotate(360.0)
    except rospy.ROSInterruptException:
        pass
