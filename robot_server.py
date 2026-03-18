#!/usr/bin/env python3
import rospy
import math
import signal
import sys
import threading
import json, os
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from flask import Flask, request, jsonify

# ==================================================
# GLOBAL SETTINGS & STATE
# ==================================================
is_navigating = False
current_location = 1

# ประกาศ Publisher ไว้ด้านนอกเพื่อให้ทุกส่วนเรียกใช้ได้
velocity_publisher = None

def signal_handler(sig, frame):
    """จัดการเมื่อกด Ctrl+C ให้หยุดหุ่นยนต์ทันที"""
    print("\n[INFO] Detecting Ctrl+C... Stopping Robot and Exiting.")
    if velocity_publisher:
        stop_cmd = Twist()
        velocity_publisher.publish(stop_cmd)
    rospy.signal_shutdown("User Interrupted")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# ==================================================
# PID CONTROLLER CLASS
# ==================================================
class PID:
    def __init__(self, kp, ki, kd, min_val, max_val):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.min_val, self.max_val = min_val, max_val
        self.integral, self.last_error = 0.0, 0.0   
        
    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.last_error = error
        return max(min(output, self.max_val), self.min_val)

# ==================================================
# ROBOT CONTROL CLASS
# ==================================================
class OdomRobot:
    def __init__(self):
        global is_navigating
        rospy.init_node("odom_robot")
        self.pub = velocity_publisher 
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        self.pid_straight = PID(kp=2.5, ki=0.0, kd=0.1, min_val=-0.4, max_val=0.4)
        self.pid_rotate = PID(kp=1.2, ki=0.01, kd=0.05, min_val=-0.6, max_val=0.6)
        
        self.raw_x, self.raw_y, self.raw_yaw = 0.0, 0.0, 0.0
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.offset_x, self.offset_y, self.offset_yaw = 0.0, 0.0, 0.0

        # Wait for the first odom message to ensure we have valid raw data
        rospy.loginfo("Waiting for odom data...")
        rospy.wait_for_message("/odom", Odometry)
        rospy.sleep(1)

        # 1. Capture current position as (0,0) immediately
        self.reset_home()
        
        # 2. Start Home Sequence
        rospy.loginfo("=== START HOME SEQUENCE ===")
        is_navigating = True  # Must be True for loops to run
        self.move_forward(2.5)
        self.rotate(math.radians(-90))
        self.move_forward(0.5)
        self.rotate(math.radians(180))
        
        # 3. Final Reset after movement to ensure we start at exact 0,0
        self.reset_home()
        is_navigating = False 
        rospy.loginfo("=== ROBOT READY (HOME=0,0,0) ===")

    def odom_callback(self, msg):
        self.raw_x = msg.pose.pose.position.x
        self.raw_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.raw_yaw = yaw
        
        # Corrected: Subtract initial raw offset
        self.x = self.raw_x - self.offset_x
        self.y = self.raw_y - self.offset_y
        
        # Corrected: Use atan2 to normalize the angle difference
        diff_yaw = self.raw_yaw - self.offset_yaw
        self.yaw = math.atan2(math.sin(diff_yaw), math.cos(diff_yaw))

    def reset_home(self):
        # Corrected: Set the offset to the current RAW values
        self.offset_x = self.raw_x
        self.offset_y = self.raw_y
        self.offset_yaw = self.raw_yaw
        # Explicitly zero out the relative coordinates
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        rospy.sleep(0.5)

    def move_forward(self, distance,bias=0.0):
        start_x, start_y = self.x, self.y
        target_yaw = self.yaw 
        rate = rospy.Rate(20)
    
        LINEAR_SPEED = 0.30 
        
        while not rospy.is_shutdown() and is_navigating:
            traveled = math.sqrt((self.x-start_x)**2 + (self.y-start_y)**2)
            if traveled >= distance: break
            
            error_yaw = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))
            
            twist = Twist()
            twist.linear.x = LINEAR_SPEED # ใช้ความเร็วใหม่ที่ตั้งไว้
            
            # เมื่อวิ่งเร็วขึ้น PID ต้องทำงานหนักขึ้น
            twist.angular.z = self.pid_straight.compute(error_yaw, 0.05) + bias
            
            self.pub.publish(twist)
            rate.sleep()
            
        self.pub.publish(Twist())
        rospy.sleep(0.3)

    def rotate(self, angle_rad):
        target_yaw = math.atan2(math.sin(self.yaw + angle_rad), math.cos(self.yaw + angle_rad))
        rate = rospy.Rate(30)
        self.pid_rotate.integral = 0 
        
        while not rospy.is_shutdown() and is_navigating:
            error = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))
            if abs(error) < 0.01: break
            
            twist = Twist()
            twist.angular.z = self.pid_rotate.compute(error, 1.0/30.0)
            
            self.pub.publish(twist)
            rate.sleep()
        self.pub.publish(Twist())
        rospy.sleep(0.3)
        
    def reset_home(self):
        self.offset_x += self.x
        self.offset_y += self.y
        self.offset_yaw += self.yaw
        rospy.sleep(0.2)
        
    def execute_path(self, start, target):
        # ... (ใส่ Dictionary paths ทั้งหมดที่คุณเขียนไว้ตรงนี้) ...
        paths = {
    (1, 2): [("rotate", -90), ("move", 6.5), ("rotate", 90), ("move", 5.0),("move", 5.0),("move", 5.2),("rotate", 90),("move", 1.0)],
    (1, 3): [("rotate", -90), ("move", 6.5), ("rotate", 90), ("move", 8.0),("move", 8.0),("move",8.0),("rotate", 90),("move", 2.0)],
    (1, 4): [("rotate", -90), ("move", 6.5),("rotate", 90),("move", )],
    (1, 5): [("rotate", -90), ("move", 6.5)],
    (1, 6): [("rotate", -90), ("move", 6.5)],
    (1, 7): [("rotate", -90), ("move", 6.5)],
    (1, 8): [("rotate", -90), ("move", 5.0),("move", 5.0),("move", 5.0),("rotate", 90),("move", 5.0),("move", 5.0),("move", 5.0),("move", 5.0),("move", 5.0),("move", 4.0),("rotate", -90),("move", 1.0)],
    (1, 9): [("rotate", -90), ("move", 5.0),("move", 5.0),("move", 5.0),("rotate", 90),("move", 5.0),("move", 5.0),("move", 5.0),("rotate", -90),("move", 1.0)],
    (1, 10): [("rotate", -90), ("move", 5.0),("move", 5.0),("move", 5.0),("rotate", 90),("move", 5.0),("move", 5.0),("rotate", -90),("move", 1.0)],
    (1, 11): [("rotate", -90), ("move", 5.0),("move", 5.0),("move", 5.0),("rotate", 90),("move", 5.0),("rotate", -90),("move", 1.0)],
    (2, 1): [("move", 2.0), ("rotate", -90), ("move", 3.0)],
    (2, 3): [("rotate", 180), ("move", 1.0), ("rotate", 90), ("move", 8.0),],
    (2, 4): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (2, 5): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (2, 6): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (2, 7): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (2, 8): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (2, 9): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (2, 10): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (2, 11): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (3, 1): [("move", 2.0), ("rotate", -90), ("move", 3.0)],
    (3, 2): [("rotate", 180), ("move", 3.0), ("rotate", 90), ("move", 2.0)],
    (3, 4): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (3, 5): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (3, 6): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (3, 7): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (3, 8): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (3, 9): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (3, 10): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (3, 11): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (4, 1): [("move", 2.0), ("rotate", -90), ("move", 3.0)],
    (4, 2): [("rotate", 180), ("move", 3.0), ("rotate", 90), ("move", 2.0)],
    (4, 3): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (4, 5): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (4, 6): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (4, 7): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (4, 8): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (4, 9): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (4, 10): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (4, 11): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (5, 1): [("move", 2.0), ("rotate", -90), ("move", 3.0)],
    (5, 2): [("rotate", 180), ("move", 3.0), ("rotate", 90), ("move", 2.0)],
    (5, 3): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (5, 4): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (5, 6): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (5, 7): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (5, 8): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (5, 9): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (5, 10): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (5, 11): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (6, 1): [("move", 2.0), ("rotate", -90), ("move", 3.0)],
    (6, 2): [("rotate", 180), ("move", 3.0), ("rotate", 90), ("move", 2.0)],
    (6, 3): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (6, 4): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (6, 5): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (6, 7): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (6, 8): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (6, 9): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (6, 10): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (6, 11): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (7, 1): [("move", 2.0), ("rotate", -90), ("move", 3.0)],
    (7, 2): [("rotate", 180), ("move", 3.0), ("rotate", 90), ("move", 2.0)],
    (7, 3): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (7, 4): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (7, 5): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (7, 6): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (7, 8): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (7, 9): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (7, 10): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (7, 11): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (8, 1): [("move", 2.0), ("rotate", -90), ("move", 3.0)],
    (8, 2): [("rotate", 180), ("move", 3.0), ("rotate", 90), ("move", 2.0)],
    (8, 3): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (8, 4): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (8, 5): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (8, 6): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (8, 7): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (8, 9): [("rotate", 180),("move", 0.8,0.0),("rotate", 90),("move", 8.0),("rotate", 90),("move", 1.0)],
    (8, 10): [("rotate", 180),("move", 0.8,0.0),("rotate", 90),("move", 8.0),("move", 8.0),("rotate", 90),("move", 1.0)],
    (8, 11): [("rotate", 180),("move", 0.8,0.0),("rotate", 90),("move", 8.0),("move", 8.0),("move", 8.0),("rotate", 90),("move", 1.0)],
    (9, 1): [("move", 2.0), ("rotate", -90), ("move", 3.0)],
    (9, 2): [("rotate", 180), ("move", 3.0), ("rotate", 90), ("move", 2.0)],
    (9, 3): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (9, 4): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (9, 5): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (9, 6): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (9, 7): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (9, 8): [("rotate", 180),("move", 1.0),("rotate", -90),("move", 8.0),("rotate", -90),("move", 1.0)],
    (9, 10): [("rotate", 180),("move", 1.0),("rotate", 90),("move", 8.0),("rotate", 90),("move", 1.0)],
    (9, 11): [("rotate", 180),("move", 1.0),("rotate", 90),("move", 8.0),("move", 8.0),("rotate", 90),("move", 1.0)],
    (10, 1): [("move", 2.0), ("rotate", -90), ("move", 3.0)],
    (10, 2): [("rotate", 180), ("move", 3.0), ("rotate", 90), ("move", 2.0)],
    (10, 3): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (10, 4): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (10, 5): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (10, 6): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (10, 7): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (10, 8): [("rotate", 180),("move", 1.0),("rotate", -90),("move", 8.0),("move", 8.0),("rotate", -90),("move", 1.0)],
    (10, 9): [("rotate", 180),("move", 1.0),("rotate", -90),("move", 8.0),("rotate", -90),("move", 1.0)],
    (10, 11): [("rotate", 180),("move", 1.5),("rotate", 90),("move", 8.0),("rotate", 90),("move", 1.0)],
    (11, 1): [("rotate", 180), ("move", 1.0), ("rotate", 90),("move", 8.0),("rotate", -90),("move", 15)],
    (11, 2): [("rotate", 180), ("move", 3.0), ("rotate", 90), ("move", 2.0)],
    (11, 3): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (11, 4): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (11, 5): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (11, 6): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (11, 7): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (11, 8): [("rotate", 180),("move", 1.0),("rotate", -90),("move", 5.0),("move", 5.0),("move", 5.0),("move", 5.0),("move", 4.0),("rotate", -90),("move", 1.0)],
    (11, 9): [("rotate", 180),("move", 1.0),("rotate", -90),("move", 5.0),("move", 5.0),("move", 6.0),("rotate", -90),("move", 1.0)],
    (11, 10): [("rotate", 180),("move", 1.0),("rotate", -90),("move", 8.0),("rotate", -90),("move", 1.0)],} # ตัวอย่าง
        
        special_bias_pairs = [(1,2),(1,3),(1,8),(1,9),(1,10),(1,11),(8,9),(8,10),(8,11),(9,10),(9,11)]
        
        key = (start, target)
        if key in paths:
            
            bias_to_use = 0.02 if key in special_bias_pairs else 0.0
            
            rospy.loginfo(f"Starting path from {start} to {target}")
            
            for action, value in paths[key]:
                if not is_navigating: break # หยุดถ้ามีการสั่ง Stop ผ่าน API
                
                if action == "move": 
                    rospy.loginfo(f"Moving {value} meters...")
                    self.move_forward(value, bias=bias_to_use)
                    
                elif action == "rotate": 
                    self.rotate(math.radians(value))
                    rospy.sleep(0.5)
            return True
        rospy.logwarn(f"Path not found for {key}")
        return False

# ==================================================
# FLASK API SERVER
# ==================================================
app = Flask(__name__)
my_robot = None

@app.route('/command', methods=['POST'])
def handle_command():
    global is_navigating
    data = request.json
    start, target = data.get('start'), data.get('target')
    
    if is_navigating:
        return jsonify({"status": "error", "message": "Robot is busy"}), 400

    def run_and_finish(s, t):
        global is_navigating, current_location
        is_navigating = True
        success = my_robot.execute_path(s, t)
        is_navigating = False
        if success:
            current_location = t

    threading.Thread(target=run_and_finish, args=(start, target)).start()
    return jsonify({"status": "starting"}), 200

@app.route('/status', methods=['GET'])
def get_status():
    return jsonify({
        "is_navigating": is_navigating,
        "current_location": current_location,
        "position": {"x": round(my_robot.x, 2), "y": round(my_robot.y, 2)},
        "yaw_deg": round(math.degrees(my_robot.yaw), 2)
    })

@app.route('/stop', methods=['POST', 'GET'])
def stop_robot():
    global is_navigating
    is_navigating = False
    velocity_publisher.publish(Twist())
    return jsonify({"status": "success", "message": "Stopped"}), 200

if __name__ == "__main__":
    # 1. สร้าง Publisher ก่อน
    velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    
    # 2. สร้าง Robot Object
    my_robot = OdomRobot()
    
    current_location = 1 
    is_navigating = False
	
    # 3. เริ่ม Flask Server (ปิด Debug เพื่อไม่ให้รันซ้ำ)
    print("--- Robot Server Ready on Port 5000 ---")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
