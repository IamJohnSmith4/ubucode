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
        """ สั่งให้หุ่นวิ่งไปจนกว่าค่า odom.x จะถึงค่าที่กำหนด """
        rate = rospy.Rate(20)
        rospy.loginfo(f"Moving until Odom X reaches: {stop_at_x} m")
        
        while not rospy.is_shutdown():
            # เช็คค่า x จาก odom โดยตรง
            if self.x >= stop_at_x:
                break
            
            twist = Twist()
            # ใส่ความเร็วคงที่ หรือจะใส่ PID ตรงนี้ก็ได้
            twist.linear.x = 0.2 
            self.velocity_publisher.publish(twist)
            
            # พิมพ์ค่าปัจจุบันให้ดู
            print(f"Current Odom X: {self.x:.3f}", end='\r')
            rate.sleep()

        # หยุดหุ่น
        self.velocity_publisher.publish(Twist())
        print(f"\n--- Stopped at Odom X: {self.x:.3f} ---")

if __name__ == "__main__":
    try:
        rospy.init_node('odom_direct_test_node')
        tester = OdomTest()
        
        # สั่งให้วิ่งไปจนกว่าค่า x ใน odom จะเท่ากับ 5.0
        tester.move_to_odom_x(5.0) 
        
    except rospy.ROSInterruptException:
        pass