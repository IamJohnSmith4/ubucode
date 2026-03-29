#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry # ต้องเพิ่มส่วนนี้
from tf.transformations import euler_from_quaternion # สำหรับแปลงมุม
import math

class OdomTest:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # --- จุดสำคัญ: ต้องมี Subscriber เพื่อดึงค่าตำแหน่งปัจจุบัน ---
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        
        # รอให้ระบบเชื่อมต่อกับ Odom ได้ก่อนเริ่ม
        rospy.loginfo("Waiting for odom data...")
        rospy.wait_for_message('/odom', Odometry)
        
        self.velocity_publisher.publish(Twist())
        rospy.sleep(1)

    # ฟังก์ชันรับค่าจาก Robot
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # แปลง Quaternion เป็น Euler (มุม Yaw)
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(quaternion)

    def move_forward_test(self, target_distance):
        # บันทึกจุดเริ่มต้น
        start_x = self.x
        start_y = self.y
        rate = rospy.Rate(20)
        
        current_speed = 0.0
        max_speed = 0.25
        accel = 0.008
        
        rospy.loginfo(f"Start Position: x={start_x:.3f}, y={start_y:.3f}")
        print(f"--- Starting Linear Test: Target {target_distance} m ---")
        
        while not rospy.is_shutdown():
            # คำนวณระยะที่เดินมาแล้วจากจุดเริ่ม
            distance_moved = math.sqrt((self.x - start_x)**2 + (self.y - start_y)**2)
            remaining_dist = target_distance - distance_moved
            
            # เงื่อนไขการหยุด
            if remaining_dist <= 0.01: 
                break
            
            # Speed Ramping
            if remaining_dist > 0.4:
                if current_speed < max_speed:
                    current_speed += accel
            else:
                current_speed = max_speed * (remaining_dist / 0.4)
                if current_speed < 0.05: current_speed = 0.05
            
            twist = Twist()
            twist.linear.x = current_speed
            twist.angular.z = 0.0 
            self.velocity_publisher.publish(twist)
            
            # ถ้าค่า self.x ขยับ ตัวเลขนี้จะเปลี่ยน
            print(f"Dist: {distance_moved:.3f} m | Current Odom X: {self.x:.3f}")
            
            rate.sleep()

        # หยุดหุ่นยนต์
        self.velocity_publisher.publish(Twist())
        print(f"--- Finished! Final Odom X: {self.x:.3f} ---")
        print(f"Total Distance by Odom: {distance_moved:.3f} m")

if __name__ == "__main__":
    try:
        rospy.init_node('odom_linear_test_node')
        tester = OdomTest()
        tester.move_forward_test(5.0) # ทดสอบ 5 เมตร
    except rospy.ROSInterruptException:
        pass