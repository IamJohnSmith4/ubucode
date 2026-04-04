#!/usr/bin/env python3
import rospy
import os
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class OdomTest:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # เชื่อมต่อเพื่อดึงค่าตำแหน่งปัจจุบัน
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        
        rospy.loginfo("Waiting for odom data...")
        rospy.wait_for_message('/odom', Odometry)
        
        self.velocity_publisher.publish(Twist())
        rospy.sleep(1)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(quaternion)


    def move_forward_test(self, target_distance):
        start_x = self.x
        start_y = self.y
        rate = rospy.Rate(20)
        
        current_speed = 0.0
        max_speed = 0.25
        accel = 0.008
        
        rospy.loginfo(f"Target: {target_distance} m")
        
        while not rospy.is_shutdown():
            # คำนวณระยะทางที่เคลื่อนที่ได้ด้วยสูตร Euclidean Distance
            distance_moved = math.sqrt((self.x - start_x)**2 + (self.y - start_y)**2)
            remaining_dist = target_distance - distance_moved
            
            # ถ้าถึงระยะ 5 เมตร ให้หยุดลูป
            if remaining_dist <= 0.01: 
                break
            
            # การคำนวณความเร็วแบบค่อยๆ เร่งและค่อยๆ เบรก (Speed Ramping)
            if remaining_dist > 0.4:
                if current_speed < max_speed:
                    current_speed += accel
            else:
                current_speed = max_speed * (remaining_dist / 0.4)
                if current_speed < 0.05: current_speed = 0.05
            
            twist = Twist()
            twist.linear.x = current_speed
            self.velocity_publisher.publish(twist)
            
            rate.sleep()

        # 1. สั่งหุ่นหยุดนิ่ง
        self.velocity_publisher.publish(Twist())
        
        
        print(f"--- Finished at {self.x:.3f} ---")

if __name__ == "__main__":
    try:
        rospy.init_node('odom_linear_test_node')
        tester = OdomTest()
        tester.move_forward_test(5.0) 
    except rospy.ROSInterruptException:
        pass