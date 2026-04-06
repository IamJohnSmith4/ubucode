#!/usr/bin/env python3
import rospy
import math
import csv
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion

# --- คลาส PID อย่างง่าย ---
class PID:
    def __init__(self, kp, ki, kd, min_val, max_val):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.min_val, self.max_val = min_val, max_val
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.last_error = error
        return max(self.min_val, min(self.max_val, output))

class RobotLogger:
    def __init__(self):
        rospy.init_node('robot_move_logger_node')
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.is_navigating = True
        
        # ข้อมูลสำหรับบันทึกกราฟ
        self.data_log = [] 

        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.reset_odom_pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.pid_straight = PID(kp=1.8, ki=0.005, kd=0.1, min_val=-0.4, max_val=0.4)
        
        rospy.loginfo("Waiting for odom...")
        rospy.wait_for_message('/odom', Odometry)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(quaternion)

    def move_forward(self, distance, bias=0.0):
        # 1. Reset Odom ก่อนเริ่ม
        self.reset_odom_pub.publish(Empty())
        rospy.sleep(1.0)
        
        start_x, start_y = self.x, self.y
        target_yaw = self.yaw 
        rate = rospy.Rate(20)
        LINEAR_SPEED = 0.50
        
        current_linear_speed = 0.05
        accel = 0.008
        min_speed = 0.07
        decel_dist = 0.4
        
        self.pid_straight.integral = 0.0
        self.pid_straight.last_error = 0.0
        
        start_time = rospy.get_time()
        rospy.loginfo(f"Starting movement: Target {distance}m with bias {bias}")

        while not rospy.is_shutdown() and self.is_navigating:
            traveled = math.sqrt((self.x - start_x)**2 + (self.y - start_y)**2)
            remaining_dist = distance - traveled
            
            if traveled >= distance: break
            
            # Logic การเร่งและผ่อนความเร็วตามที่คุณกำหนด
            if remaining_dist > decel_dist:
                if current_linear_speed < LINEAR_SPEED:
                    current_linear_speed += accel
                else:
                    current_linear_speed = LINEAR_SPEED
            else:
                current_linear_speed = max(min_speed, (remaining_dist / decel_dist) * LINEAR_SPEED)
            
            error_yaw = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))
            
            twist = Twist()
            twist.linear.x = current_linear_speed
            # ใช้ PID คุมทิศทางให้ตรง
            twist.angular.z = self.pid_straight.compute(error_yaw, 0.05) + bias
            
            # บันทึกข้อมูลลงใน List
            self.data_log.append([
                rospy.get_time() - start_time, # เวลา
                traveled,                      # ระยะที่เดินได้
                current_linear_speed,          # ความเร็วเส้นตรง
                error_yaw                      # ค่าความผิดพลาดของมุม
            ])

            self.pub.publish(twist)
            rate.sleep()
            
        self.pub.publish(Twist())
        rospy.loginfo("Finished! Saving data...")
        self.save_data()

    def save_data(self):
        # บันทึกเป็น CSV
        filename = "robot_navigation_data.csv"
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time', 'Distance', 'LinearSpeed', 'YawError'])
            writer.writerows(self.data_log)
        rospy.loginfo(f"Data saved to {filename}")

        # สร้างกราฟสรุปผล
        times = [d[0] for d in self.data_log]
        speeds = [d[2] for d in self.data_log]
        distances = [d[1] for d in self.data_log]

        plt.figure(figsize=(10, 6))
        plt.subplot(2, 1, 1)
        plt.plot(times, speeds, 'b-', label='Linear Speed (m/s)')
        plt.ylabel('Speed')
        plt.legend()
        plt.grid(True)

        plt.subplot(2, 1, 2)
        plt.plot(times, distances, 'g-', label='Traveled Distance (m)')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance')
        plt.legend()
        plt.grid(True)

        plt.savefig('navigation_plot.png')
        rospy.loginfo("Graph saved as navigation_plot.png")

if __name__ == '__main__':
    try:
        logger = RobotLogger()
        # ทดสอบเดินหน้า 3 เมตร
        logger.move_forward(3.0)
    except rospy.ROSInterruptException:
        pass