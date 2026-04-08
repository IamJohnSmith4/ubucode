#!/usr/bin/env python3
import rospy
import math
import csv
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion

# --- คลาส PID ---
class PID:
    def __init__(self, kp, ki, kd, min_val, max_val):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.min_val, self.max_val = min_val, max_val
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, error, dt):
        if dt <= 0: return 0.0
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.last_error = error
        return max(self.min_val, min(self.max_val, output))

class RobotLogger:
    def __init__(self):
        rospy.init_node('robot_rotate_logger_node')
        self.yaw = 0.0
        self.is_navigating = True
        self.data_log = [] 

        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.reset_odom_pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # ตั้งค่า PID สำหรับการหมุน (ปรับจูนค่า Kp, Ki, Kd ตามจริงได้เลย)
        self.pid_rotate = PID(kp=1.5, ki=0.01, kd=0.1, min_val=-0.6, max_val=0.6)
        
        rospy.loginfo("ระบบ: กำลังเชื่อมต่อ Odometry...")
        rospy.wait_for_message('/odom', Odometry)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        (_, _, self.yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def reset_odom(self):
        rospy.loginfo("ระบบ: ส่งคำสั่งรีเซ็ต Odometry...")
        for _ in range(3):
            self.reset_odom_pub.publish(Empty())
            rospy.sleep(0.1)
        rospy.sleep(1.0) # รอให้ Odom นิ่ง

    # =========================================================
    # ฟังก์ชัน 1: หมุนตัวแบบใช้ PID (สมูท, ลดความเร็วเมื่อใกล้ถึง)
    # =========================================================
    def rotate_pid(self, target_angle_deg):
        self.reset_odom()
        self.data_log = []
        
        target_angle_rad = math.radians(target_angle_deg)
        rate = rospy.Rate(20)
        
        self.pid_rotate.integral = 0.0
        self.pid_rotate.last_error = 0.0
        
        start_time = rospy.get_time()
        last_time = start_time
        last_yaw = self.yaw
        traveled_angle_rad = 0.0 # ค่ามุมสะสม
        
        rospy.loginfo(f"ระบบ: เริ่มหมุนตัว [ด้วย PID] เป้าหมาย {target_angle_deg} องศา")

        while not rospy.is_shutdown() and self.is_navigating:
            current_time = rospy.get_time()
            dt = current_time - last_time
            if dt == 0: dt = 0.05
            
            # คำนวณมุมที่ขยับไปทีละนิด แล้วบวกสะสม (กันปัญหามุมข้ามจาก 180 เป็น -180)
            delta_yaw = self.yaw - last_yaw
            delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))
            traveled_angle_rad += delta_yaw
            
            last_yaw = self.yaw
            last_time = current_time

            # หา Error 
            error_rad = target_angle_rad - traveled_angle_rad
            
            # ถ้า Error เหลือน้อยกว่า 0.5 องศา ถือว่าเข้าเป้า ให้เบรก
            if abs(math.degrees(error_rad)) < 0.5:
                break

            # คำนวณความเร็วโดยใช้ PID
            twist = Twist()
            angular_speed = self.pid_rotate.compute(error_rad, dt)
            twist.angular.z = angular_speed
            self.pub.publish(twist)
            
            self.data_log.append([
                round(current_time - start_time, 3), 
                round(math.degrees(traveled_angle_rad), 2), 
                round(angular_speed, 3), 
                round(math.degrees(error_rad), 2)
            ])

            print(f"[PID] หมุนไป: {math.degrees(traveled_angle_rad):.1f} / {target_angle_deg} องศา (Vz: {angular_speed:.2f})", end='\r')
            rate.sleep()
            
        self.pub.publish(Twist()) # สั่งหยุด
        self.save_results(target_angle_deg, "PID")

    # =========================================================
    # ฟังก์ชัน 2: หมุนตัวแบบธรรมดา ไม่มี PID (วิ่งด้วยความเร็วคงที่แล้วตัดจบ)
    # =========================================================
    def rotate_simple(self, target_angle_deg):
        self.reset_odom()
        self.data_log = []
        
        target_angle_rad = math.radians(target_angle_deg)
        direction = 1 if target_angle_deg > 0 else -1
        
        CONSTANT_SPEED = 0.4 * direction # กำหนดความเร็วคงที่ (rad/s)
        rate = rospy.Rate(20)
        
        start_time = rospy.get_time()
        last_yaw = self.yaw
        traveled_angle_rad = 0.0
        
        rospy.loginfo(f"ระบบ: เริ่มหมุนตัว [ไม่ใช้ PID] เป้าหมาย {target_angle_deg} องศา")

        while not rospy.is_shutdown() and self.is_navigating:
            delta_yaw = self.yaw - last_yaw
            delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))
            traveled_angle_rad += delta_yaw
            last_yaw = self.yaw
            
            error_rad = target_angle_rad - traveled_angle_rad

            # ถ้าหมุนถึงมุมเป้าหมายแล้ว ให้เด้งออกจากลูปเลยทันที
            if abs(traveled_angle_rad) >= abs(target_angle_rad):
                break

            twist = Twist()
            twist.angular.z = CONSTANT_SPEED
            self.pub.publish(twist)
            
            self.data_log.append([
                round(rospy.get_time() - start_time, 3), 
                round(math.degrees(traveled_angle_rad), 2), 
                round(CONSTANT_SPEED, 3), 
                round(math.degrees(error_rad), 2)
            ])

            print(f"[Simple] หมุนไป: {math.degrees(traveled_angle_rad):.1f} / {target_angle_deg} องศา (Vz: {CONSTANT_SPEED:.2f})", end='\r')
            rate.sleep()
            
        self.pub.publish(Twist()) # สั่งหยุดแบบกะทันหัน (จะเห็นกราฟไหล Overshoot แน่นอน)
        rospy.sleep(0.5) # รอรถเบรกให้สนิท
        
        # เก็บค่าตอนรถจอดสนิทแล้ว เพื่อดูกรณีรถไหล (Inertia)
        delta_yaw = self.yaw - last_yaw
        delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))
        traveled_angle_rad += delta_yaw
        error_rad = target_angle_rad - traveled_angle_rad
        
        self.data_log.append([
            round(rospy.get_time() - start_time, 3), 
            round(math.degrees(traveled_angle_rad), 2), 
            0.0, 
            round(math.degrees(error_rad), 2)
        ])
        
        self.save_results(target_angle_deg, "NoPID")

    # =========================================================
    # ฟังก์ชันเก็บบันทึกผลและสร้างกราฟ (ใช้ร่วมกัน)
    # =========================================================
    def save_results(self, target_deg, method_name):
        rospy.loginfo(f"\nระบบ: ถึงเป้าหมายแล้ว! กำลังสรุปข้อมูล ({method_name})...")
        csv_file = f"ผลการหมุนตัว_{method_name}.csv"
        
        with open(csv_file, mode='w', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f)
            writer.writerow(['เวลา (วินาที)', 'มุมสะสม (องศา)', 'ความเร็วเชิงมุม (rad/s)', 'Error (องศา)'])
            writer.writerows(self.data_log)
        
        final_deg = self.data_log[-1][1]
        error_deg = self.data_log[-1][3]
        
        print(f"\n--- [ สรุปผลการทดลอง: โหมด {method_name} ] ---")
        print(f"เป้าหมาย: {target_deg} องศา")
        print(f"หมุนได้จริง: {final_deg:.2f} องศา")
        print(f"ความคลาดเคลื่อน (Error): {error_deg:.2f} องศา")

        # พลอตกราฟ
        t_vals = [d[0] for d in self.data_log]
        s_vals = [d[2] for d in self.data_log]
        d_vals = [d[1] for d in self.data_log]

        plt.figure(figsize=(10, 8))
        plt.subplot(2, 1, 1)
        plt.plot(t_vals, s_vals, 'b', label='Angular Velocity (rad/s)')
        plt.title(f'Robot Rotation Profile [{method_name}]')
        plt.ylabel('Velocity')
        plt.grid(True); plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(t_vals, d_vals, 'g', label='Rotated Angle (deg)')
        plt.axhline(y=target_deg, color='r', linestyle='--', label='Target Angle') # เส้นเป้าหมายสีแดง
        plt.xlabel('Time (s)'); plt.ylabel('Degrees')
        plt.grid(True); plt.legend()

        img_name = f'graph_rotation_{method_name}.png'
        plt.savefig(img_name)
        rospy.loginfo(f"บันทึกไฟล์กราฟ '{img_name}' และข้อมูลลง CSV สำเร็จ\n")

if __name__ == '__main__':
    try:
        logger = RobotLogger()
        
        # -----------------------------------------------------------------
        # [ วิธีการใช้งาน ]
        # เลือกว่าจะทดสอบฟังก์ชันไหน ให้เอาเครื่องหมาย # ออกจากบรรทัดนั้น
        # -----------------------------------------------------------------
        
        # ทดสอบที่ 1: หมุน 90 องศา แบบไม่ใช้ PID (จะเห็นรถไหล Overshoot ชัดเจน)
        logger.rotate_simple(90.0) 
        
        # ทดสอบที่ 2: หมุน 90 องศา แบบใช้ PID (จะเห็นกราฟความเร็วค่อยๆ โค้งลงนุ่มนวล)
        # logger.rotate_pid(90.0)
        
    except rospy.ROSInterruptException:
        pass
