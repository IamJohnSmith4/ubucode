#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty  # สำหรับ reset odom
from tf.transformations import euler_from_quaternion

class OdomTest:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Publisher สำหรับ reset odom (ของ Kobuki/TurtleBot2)
        self.reset_odom_pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
        
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        
        rospy.loginfo("Resetting Odometry...")
        self.reset_odometry() # เรียกใช้ฟังก์ชันรีเซ็ต
        
        rospy.loginfo("Waiting for odom data...")
        rospy.wait_for_message('/odom', Odometry)

    def reset_odometry(self):
        # วนลูปส่งคำสั่ง reset เล็กน้อยเพื่อให้แน่ใจว่าหุ่นได้รับคำสั่ง
        for _ in range(10):
            self.reset_odom_pub.publish(Empty())
            rospy.sleep(0.1)
        rospy.loginfo("Odometry Reset Complete.")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(quaternion)

    def move_to_odom_x(self, stop_at_x):
        """ สั่งให้หุ่นวิ่งแบบ Smooth: เริ่มช้า -> เร็ว -> ช้าก่อนหยุด """
        rate = rospy.Rate(20)
        rospy.loginfo(f"Moving Smoothly to Odom X: {stop_at_x} m")

        # --- ตั้งค่าความเร็ว ---
        max_speed = 0.4      # ความเร็วสูงสุดกลางทาง
        min_speed = 0.05     # ความเร็วต่ำสุด (ป้องกันหุ่นอืดจนไม่เดิน)
        ramp_dist = 0.5      # ระยะทางที่ใช้ในการเร่ง/ผ่อน (เมตร)

        while not rospy.is_shutdown():
            current_x = self.x
            dist_to_go = stop_at_x - current_x

            # 1. เงื่อนไขหยุดเมื่อถึงระยะ
            if dist_to_go <= 0:
                break

            # 2. คำนวณความเร็ว (Velocity Scaling)
            # ช่วงเร่งออกตัว (Acceleration)
            accel_speed = (current_x / ramp_dist) * max_speed
            
            # ช่วงผ่อนความเร็วเมื่อใกล้ถึง (Deceleration)
            decel_speed = (dist_to_go / ramp_dist) * max_speed

            # เลือกความเร็วที่เหมาะสม (ใช้ค่าที่น้อยที่สุดระหว่างช่วงเร่ง/คงที่/ผ่อน)
            speed = min(max_speed, accel_speed, decel_speed)
            
            # คุมไม่ให้ต่ำกว่าความเร็วขั้นต่ำที่ตั้งไว้
            speed = max(speed, min_speed)

            # 3. ส่งคำสั่งเคลื่อนที่
            twist = Twist()
            twist.linear.x = speed
            self.velocity_publisher.publish(twist)

            print(f"Dist to go: {dist_to_go:.3f} | Current Speed: {speed:.3f}", end='\r')
            rate.sleep()

        # หยุดหุ่นเมื่อถึงจุดหมาย
        self.velocity_publisher.publish(Twist())
        print(f"\n Arrived at Odom X: {self.x:.3f}")

if __name__ == "__main__":
    try:
        rospy.init_node('odom_direct_test_node')
        tester = OdomTest()
        
        # สั่งให้วิ่งไปจนกว่าค่า x ใน odom จะเท่ากับ 5.0
        tester.move_to_odom_x(5.0) 
        
    except rospy.ROSInterruptException:
        pass
