#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math

# --- ส่วนนี้คือ Class OdomRobot ของคุณ (ย่อมาเฉพาะฟังก์ชันที่ใช้ทดสอบ) ---
class OdomTest:
    def __init__(self):
        # สมมติว่าคุณมีส่วนรับค่า x, y, yaw จาก /odom อยู่แล้วใน Class เดิม
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        # สั่งหยุดหุ่นยนต์ก่อนเริ่ม
        self.velocity_publisher.publish(Twist())
        rospy.sleep(1)

    def move_forward_test(self, target_distance):
        start_x = self.x
        start_y = self.y
        rate = rospy.Rate(20)
        
        # ตั้งค่า Speed Ramping ตามโค้ดที่คุณใช้จริง
        current_speed = 0.0
        max_speed = 0.25      # ความเร็วสูงสุด
        accel = 0.008         # อัตราการเร่ง
        
        print(f"--- Starting Linear Test: Target {target_distance} m ---")
        
        while not rospy.is_shutdown():
            # คำนวณระยะทางที่เดินมาแล้ว (Euclidean Distance)
            distance_moved = math.sqrt((self.x - start_x)**2 + (self.y - start_y)**2)
            remaining_dist = target_distance - distance_moved
            
            if remaining_dist <= 0.01: # จุดหยุด (Tolerance)
                break
            
            # ระบบ Speed Ramping (เร่ง และ เบรก)
            if remaining_dist > 0.4: # ช่วงเร่งและคงที่
                if current_speed < max_speed:
                    current_speed += accel
            else: # ช่วงเบรก (0.4 เมตรสุดท้าย)
                current_speed = max_speed * (remaining_dist / 0.4)
                if current_speed < 0.05: current_speed = 0.05 # ความเร็วขั้นต่ำ
            
            # ส่งคำสั่งเดินตรง
            twist = Twist()
            twist.linear.x = current_speed
            twist.angular.z = 0.0 # ในการทดสอบ Odom เพียวๆ เราจะล็อคค่านี้เป็น 0
            self.velocity_publisher.publish(twist)
            
            # Log ค่าเพื่อดูความแม่นยำ (เอาไปใส่ในบทที่ 4)
            print(f"Dist: {distance_moved:.3f} m | Speed: {current_speed:.3f} m/s | Odom X: {self.x:.3f}")
            
            rate.sleep()

        # หยุดหุ่นยนต์เมื่อถึงระยะ
        self.velocity_publisher.publish(Twist())
        print(f"--- Test Finished! Final Odom Distance: {distance_moved:.3f} m ---")
        print("Please measure the actual distance on the floor with a tape measure.")

if __name__ == "__main__":
    try:
        rospy.init_node('odom_linear_test_node')
        tester = OdomTest()
        
        # สั่งทดสอบเดินตรง 5 เมตร (ปรับเปลี่ยนตัวเลขได้ตามต้องการ)
        target = 5.0 
        tester.move_forward_test(target)
        
    except rospy.ROSInterruptException:
        pass