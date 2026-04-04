#!/usr/bin/env python3
import rospy
import math
import csv
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class OdomCSVTest:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # ตั้งชื่อไฟล์ CSV ตามเวลาที่รัน
        self.filename = f"robot_test_{int(time.time())}.csv"
        
        # Subscriber รับค่าจาก /odom
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
        (_, _, self.yaw) = euler_from_quaternion(quaternion)

    def move_forward_test(self, target_distance):
        start_x = self.x
        start_y = self.y
        rate = rospy.Rate(20)
        
        current_speed = 0.0
        max_speed = 0.25
        accel = 0.008
        
        # เตรียมไฟล์ CSV
        with open(self.filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            # เขียน Header ของไฟล์
            writer.writerow(['Time_Sec', 'Odom_X', 'Odom_Y', 'Distance_Moved', 'Error'])
            
            start_time = rospy.get_time()
            rospy.loginfo(f"Saving data to {self.filename}")
            print(f"--- Starting Linear Test: Target {target_distance} m ---")
            
            while not rospy.is_shutdown():
                # คำนวณระยะทางและ Error
                distance_moved = math.sqrt((self.x - start_x)**2 + (self.y - start_y)**2)
                error = target_distance - distance_moved # e(t)
                
                # บันทึกข้อมูลลง CSV
                current_time = rospy.get_time() - start_time
                writer.writerow([round(current_time, 2), round(self.x, 4), round(self.y, 4), round(distance_moved, 4), round(error, 4)])
                
                # เงื่อนไขการหยุด
                if error <= 0.01: 
                    break
                
                # Speed Ramping (เร่ง-เบรก)
                if error > 0.4:
                    if current_speed < max_speed:
                        current_speed += accel
                else:
                    current_speed = max_speed * (error / 0.4)
                    if current_speed < 0.05: current_speed = 0.05
                
                twist = Twist()
                twist.linear.x = current_speed
                self.velocity_publisher.publish(twist)
                
                print(f"Dist: {distance_moved:.3f} m | Error: {error:.3f} m")
                rate.sleep()

        # หยุดหุ่นยนต์
        self.velocity_publisher.publish(Twist())
        final_error = target_distance - math.sqrt((self.x - start_x)**2 + (self.y - start_y)**2)
        
        print(f"--- Test Finished ---")
        print(f"Final Odom Distance: {target_distance - final_error:.4f} m")
        print(f"Final Error: {final_error:.4f} m")
        rospy.loginfo(f"Log saved to {self.filename}")

if __name__ == "__main__":
    try:
        rospy.init_node('odom_csv_test_node')
        tester = OdomCSVTest()
        tester.move_forward_test(5.0) 
    except rospy.ROSInterruptException:
        pass