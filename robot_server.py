#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from flask import Flask, request, jsonify

velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
# ==================================================
# ROBOT CLASS (ยกมาจากโค้ดของคุณทั้งหมด)
# ==================================================
class OdomRobot:
    def __init__(self):
        rospy.init_node("odom_robot")
        self.pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.raw_x, self.raw_y, self.raw_yaw = 0.0, 0.0, 0.0
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.offset_x, self.offset_y, self.offset_yaw = 0.0, 0.0, 0.0

        rospy.sleep(1)
        rospy.loginfo("=== START HOME SEQUENCE ===")
        self.move_forward(2.5)
        self.rotate(-math.pi / 2)
        self.move_forward(0.5)
        self.rotate(math.pi)
        self.reset_home()
        rospy.loginfo("=== ROBOT READY (HOME=0,0,0) ===")

    def odom_callback(self, msg):
        self.raw_x = msg.pose.pose.position.x
        self.raw_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.raw_yaw = yaw
        self.x = self.raw_x - self.offset_x
        self.y = self.raw_y - self.offset_y
        self.yaw = math.atan2(math.sin(self.raw_yaw - self.offset_yaw), math.cos(self.raw_yaw - self.offset_yaw))

    def move_forward(self, distance):
        start_x, start_y = self.x, self.y
        rate, twist = rospy.Rate(20), Twist()
        twist.linear.x = 0.2
        while not rospy.is_shutdown():
            traveled = math.sqrt((self.x-start_x)**2 + (self.y-start_y)**2)
            if traveled >= distance: break
            self.pub.publish(twist)
            rate.sleep()
        self.pub.publish(Twist())
        rospy.sleep(0.3)

    def rotate(self, angle):
        target_yaw = math.atan2(math.sin(self.yaw + angle), math.cos(self.yaw + angle))
        rate, twist = rospy.Rate(30), Twist()
        while not rospy.is_shutdown():
            error = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))
            if abs(error) < 0.02: break
            twist.angular.z = 0.5 if error > 0 else -0.5
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
        paths = {(1, 2): [("rotate", -90), ("move", 2.0), ("move", 2.0), ("move", 2.0), ("move", 0.5), ("rotate", 90), ("move", 2.0), ("move", 2.0), ("move", 2.0), ("move", 2.0), ("move", 2.0), ("rotate", 90), ("move", 1.5)],
    (1, 3): [("rotate", 180), ("move", 3.0), ("rotate", 90), ("move", 2.0)],
    (1, 4): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (1, 5): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (1, 6): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (1, 7): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (1, 8): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (1, 9): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (1, 10): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (1, 11): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (2, 1): [("move", 2.0), ("rotate", -90), ("move", 3.0)],
    (2, 3): [("rotate", 180), ("move", 3.0), ("rotate", 90), ("move", 2.0)],
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
    (8, 9): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (8, 10): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (8, 11): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (9, 1): [("move", 2.0), ("rotate", -90), ("move", 3.0)],
    (9, 2): [("rotate", 180), ("move", 3.0), ("rotate", 90), ("move", 2.0)],
    (9, 3): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (9, 4): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (9, 5): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (9, 6): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (9, 7): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (9, 8): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (9, 10): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (9, 11): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (10, 1): [("move", 2.0), ("rotate", -90), ("move", 3.0)],
    (10, 2): [("rotate", 180), ("move", 3.0), ("rotate", 90), ("move", 2.0)],
    (10, 3): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (10, 4): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (10, 5): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (10, 6): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (10, 7): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (10, 8): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (10, 9): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (10, 11): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (11, 1): [("move", 2.0), ("rotate", -90), ("move", 3.0)],
    (11, 2): [("rotate", 180), ("move", 3.0), ("rotate", 90), ("move", 2.0)],
    (11, 3): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (11, 4): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (11, 5): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (11, 6): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (11, 7): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (11, 8): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (11, 9): [("move", 1.5), ("rotate", 90), ("move", 2.0)],
    (11, 10): [("move", 1.5), ("rotate", 90), ("move", 2.0)],} # ตัวอย่าง
        key = (start, target)
        if key in paths:
            for action, value in paths[key]:
                if action == "move": self.move_forward(value)
                elif action == "rotate": self.rotate(math.radians(value))
            return True
        return False

# ==================================================
# API SERVER (ส่วนที่เพิ่มมาเพื่อคุยกับ Windows)
# ==================================================
app = Flask(__name__)
my_robot = None

@app.route('/command', methods=['POST'])
def handle_command():
    data = request.json
    start, target = data.get('start'), data.get('target')
    rospy.loginfo(f"Received command: Station {start} -> {target}")
    success = my_robot.execute_path(start, target)
    return jsonify({"status": "success" if success else "error"})


@app.route('/stop', methods=['GET', 'POST'])
def stop_robot():
    try:
        # 1. สร้างข้อความความเร็วเป็น 0 ทั้งหมด
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        
        # 2. Publish ไปที่ ROS ทันที
        velocity_publisher.publish(stop_cmd)
        
        # 3. (สำคัญมาก) ถ้าคุณใช้ Loop ในการเดิน ต้องมีตัวแปรขัดจังหวะ
        # เช่นกำหนด global variable ให้หลุดจาก loop เดินหน้า
        global is_navigating
        is_navigating = False 
        
        print("!!! EMERGENCY STOP: Received stop command from Web UI")
        return jsonify({"status": "success", "message": "Robot stopped"}), 200
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500
        
if __name__ == "__main__":
    my_robot = OdomRobot()
	    app.run(host='0.0.0.0', port=5000)
