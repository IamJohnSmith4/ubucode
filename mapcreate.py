#!/usr/bin/env python3
import rospy
import math
import signal
import sys
import threading
import json, os
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from flask import Flask, request, jsonify

# ==================================================
# GLOBAL SETTINGS & STATE
# ==================================================
is_navigating = False
current_location = 1
current_progress = 0
POSITION_FILE = "/tmp/robot_last_position.json"
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

        # --- Odometry State ---
        self.raw_x, self.raw_y, self.raw_yaw = 0.0, 0.0, 0.0
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.offset_x, self.offset_y, self.offset_yaw = 0.0, 0.0, 0.0

        # --- AMCL State ---
        # AMCL ให้ตำแหน่งใน frame "map" ซึ่งแม่นกว่า odom เพราะใช้ LiDAR แก้ drift
        self.amcl_x = 0.0
        self.amcl_y = 0.0
        self.amcl_yaw = 0.0
        self.amcl_covariance = 1.0      # ค่าความไม่แน่นอน (ยิ่งสูง = ยิ่งไม่แน่ใจ)
        self.amcl_ready = False          # AMCL localize ตัวเองสำเร็จหรือยัง
        self.amcl_lock = threading.Lock()

        # --- Subscribers ---
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)

        self.pid_straight = PID(kp=1.8, ki=0.005, kd=0.1,  min_val=-0.4, max_val=0.4)
        self.pid_rotate   = PID(kp=1.0, ki=0.01,  kd=0.1,  min_val=-0.3, max_val=0.3)

        # Wait for odom
        rospy.loginfo("Waiting for odom data...")
        rospy.wait_for_message("/odom", Odometry)
        rospy.sleep(1)

        # --- Home Sequence ---
        self.reset_home()
        rospy.loginfo("=== START HOME SEQUENCE ===")
        is_navigating = True
        self.move_forward(1)
        self.reset_home()
        is_navigating = False
        rospy.loginfo("=== ROBOT READY (HOME=0,0,0) ===")

        # --- Wait for AMCL (non-blocking, warn if unavailable) ---
        self._wait_for_amcl(timeout=10.0)

    # --------------------------------------------------
    # CALLBACKS
    # --------------------------------------------------
    def odom_callback(self, msg):
        self.raw_x = msg.pose.pose.position.x
        self.raw_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.raw_yaw = yaw

        diff_yaw = self.raw_yaw - self.offset_yaw
        self.x   = self.raw_x - self.offset_x
        self.y   = self.raw_y - self.offset_y
        self.yaw = math.atan2(math.sin(diff_yaw), math.cos(diff_yaw))

    def amcl_callback(self, msg):
        """
        รับ pose จาก AMCL (frame: map)
        covariance[0] = xx, covariance[7] = yy  (ยิ่งต่ำยิ่งมั่นใจ)
        """
        with self.amcl_lock:
            self.amcl_x = msg.pose.pose.position.x
            self.amcl_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            (_, _, self.amcl_yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

            # ใช้ค่าเฉลี่ย covariance แกน x, y เป็นตัวบอกความมั่นใจ
            cov = msg.pose.covariance
            self.amcl_covariance = (cov[0] + cov[7]) / 2.0

            # ถือว่า AMCL localize ได้แล้วเมื่อ covariance ต่ำกว่า threshold
            if self.amcl_covariance < 0.05:
                self.amcl_ready = True

    # --------------------------------------------------
    # AMCL HELPERS
    # --------------------------------------------------
    def _wait_for_amcl(self, timeout=10.0):
        """รอ AMCL topic สักพัก ถ้าไม่มีก็เดินหน้าต่อได้"""
        rospy.loginfo("Waiting for /amcl_pose (%.0fs timeout)..." % timeout)
        try:
            rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=timeout)
            rospy.loginfo("[AMCL] Topic found. AMCL is running.")
        except rospy.ROSException:
            rospy.logwarn("[AMCL] /amcl_pose not received within %.0fs. "
                          "Running with odometry only." % timeout)

    def set_initial_pose(self, x, y, yaw_deg):
        """
        ส่ง initial pose ให้ AMCL เพื่อให้ localize เร็วขึ้น
        เรียกผ่าน API: POST /amcl/set-pose  {"x":0,"y":0,"yaw":0}
        """
        pub = rospy.Publisher("/initialpose",
                              PoseWithCovarianceStamped,
                              queue_size=1, latch=True)
        rospy.sleep(0.5)   # ให้ publisher register ก่อน

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.Time.now()

        yaw_rad = math.radians(yaw_deg)
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        # covariance เริ่มต้น (ความไม่แน่นอนพอสมควร)
        cov = [0.0] * 36
        cov[0]  = 0.25    # xx
        cov[7]  = 0.25    # yy
        cov[35] = 0.068   # yaw-yaw  (~15 deg)
        pose_msg.pose.covariance = cov

        pub.publish(pose_msg)
        rospy.loginfo(f"[AMCL] Initial pose set → x={x:.2f}, y={y:.2f}, yaw={yaw_deg}°")

    def get_best_pose(self):
        """
        คืนค่า pose ที่ดีที่สุดในขณะนั้น:
        - ถ้า AMCL ready และ covariance ต่ำ  → ใช้ AMCL (แม่นกว่า)
        - ถ้า AMCL ยังไม่ stable              → ใช้ Odom (เดิม)
        คืนค่า (x, y, yaw, source)
        """
        with self.amcl_lock:
            if self.amcl_ready and self.amcl_covariance < 0.05:
                return self.amcl_x, self.amcl_y, self.amcl_yaw, "amcl"
        return self.x, self.y, self.yaw, "odom"

    # --------------------------------------------------
    # RESET / HOME
    # --------------------------------------------------
    def reset_home(self):
        self.offset_x   = self.raw_x
        self.offset_y   = self.raw_y
        self.offset_yaw = self.raw_yaw
        self.x = self.y = self.yaw = 0.0
        rospy.sleep(0.5)

    def execute_home_sequence(self):
        global is_navigating, current_location
        rospy.loginfo("--- Starting Home Sequence ---")
        is_navigating = True
        self.reset_home()
        self.move_forward(1)
        self.reset_home()
        current_location = 1
        is_navigating = False
        rospy.loginfo("--- Home Sequence Completed: Current Node is 1 ---")

    # --------------------------------------------------
    # MOTION PRIMITIVES
    # --------------------------------------------------
    def move_forward(self, distance, bias=0.0):
        start_x, start_y = self.x, self.y
        target_yaw = self.yaw
        rate = rospy.Rate(20)
        LINEAR_SPEED = 0.10
        rospy.loginfo(f"bias = {bias}")

        current_linear_speed = 0.05
        accel    = 0.008
        min_speed  = 0.07
        decel_dist = 0.4

        self.pid_straight.integral   = 0.0
        self.pid_straight.last_error = 0.0

        while not rospy.is_shutdown() and is_navigating:
            traveled      = math.sqrt((self.x - start_x)**2 + (self.y - start_y)**2)
            remaining_dist = distance - traveled

            if traveled >= distance:
                break

            if remaining_dist > decel_dist:
                current_linear_speed = min(current_linear_speed + accel, LINEAR_SPEED)
            else:
                current_linear_speed = max(min_speed,
                                           (remaining_dist / decel_dist) * LINEAR_SPEED)

            error_yaw = math.atan2(math.sin(target_yaw - self.yaw),
                                   math.cos(target_yaw - self.yaw))

            twist = Twist()
            twist.linear.x  = current_linear_speed
            twist.angular.z = self.pid_straight.compute(error_yaw, 0.05) + bias
            self.pub.publish(twist)
            rate.sleep()

        self.pub.publish(Twist())
        rospy.sleep(0.3)

    def rotate(self, angle_rad):
        target_yaw = math.atan2(math.sin(self.yaw + angle_rad),
                                math.cos(self.yaw + angle_rad))
        rate = rospy.Rate(30)
        self.pid_rotate.integral = 0

        while not rospy.is_shutdown() and is_navigating:
            error = math.atan2(math.sin(target_yaw - self.yaw),
                               math.cos(target_yaw - self.yaw))
            if abs(error) < 0.005:
                break
            twist = Twist()
            twist.angular.z = self.pid_rotate.compute(error, 1.0 / 30.0)
            self.pub.publish(twist)
            rate.sleep()

        self.pub.publish(Twist())
        rospy.sleep(0.3)

    # --------------------------------------------------
    # PATH EXECUTION
    # --------------------------------------------------
    def execute_path(self, start, target):
        global current_progress
        l, h, r, z = 0.04, 0.02, 0.01, 0.00
        paths = {

            (1, 2):  [("move", 10.0, r), ("move", 5.2, z), ("rotate", 90), ("move", 1.0)],
            (1, 3):  [("move", 10.0, r), ("move", 10.0, z), ("move", 7.2, r), ("rotate", 90), ("move", 1.0)],
           
            (2, 1):  [("rotate", 180), ("move", 1.0), ("rotate", -90), ("move", 10.0, l), ("move", 5.2, z), ("rotate", -90), ("move", 6.5), ("rotate", -90)],
            (2, 3):  [("rotate", 180), ("move", 1.0), ("rotate", 90), ("move", 10.0, l), ("move", 2.5, r), ("rotate", 90), ("move", 1.0)],
           
            (3, 1):  [("rotate", 180), ("move", 1.0), ("rotate", -90), ("move", 10.0, r), ("move", 10.0, r), ("move", 7.2, r), ("rotate", -90), ("move", 6.5), ("rotate", -90)],
            (3, 2):  [("rotate", 180), ("move", 1.0), ("rotate", -90), ("move", 10.0, r), ("move", 2.5, r), ("rotate", -90), ("move", 1.0)],

        }

        key = (start, target)
        current_progress = 0
        if key not in paths:
            rospy.logwarn(f"No path defined for ({start} → {target})")
            return False

        rospy.loginfo(f"Starting path from {start} to {target}")
        total_steps   = len(paths[key])
        current_steps = 0

        for cmd in paths[key]:
            if not is_navigating:
                break

            current_steps += 1
            action = cmd[0]

            if action == "move":
                raw_dist = cmd[1]
                dist = float(raw_dist[0]) if isinstance(raw_dist, (list, tuple)) else float(raw_dist)
                bias = 0.0
                if len(cmd) > 2:
                    raw_bias = cmd[2]
                    bias = float(raw_bias[0]) if isinstance(raw_bias, (list, tuple)) else float(raw_bias)
                self.move_forward(dist, bias)

            elif action == "rotate":
                angle = cmd[1]
                self.rotate(math.radians(angle))
                rospy.sleep(0.5)

            current_progress = (current_steps / total_steps) * 100
            rospy.loginfo(f"progress = {current_progress:.2f}%")

        self.pub.publish(Twist())
        return True


# ==================================================
# FLASK API SERVER
# ==================================================
app = Flask(__name__)
my_robot = None


@app.route('/command', methods=['POST'])
def handle_command():
    global is_navigating, current_progress
    data = request.json
    start, target = data.get('start'), data.get('target')

    if is_navigating:
        return jsonify({"status": "error", "message": "Robot is busy"}), 400

    def run_and_finish(s, t):
        global is_navigating, current_location, current_progress
        current_progress = 0
        is_navigating = True
        success = my_robot.execute_path(s, t)
        is_navigating = False
        if success:
            current_location = t

    threading.Thread(target=run_and_finish, args=(start, target)).start()
    return jsonify({"status": "starting"}), 200


@app.route('/status', methods=['GET'])
def get_status():
    # ดึง pose ที่ดีที่สุด ณ ขณะนั้น (AMCL หรือ Odom)
    best_x, best_y, best_yaw, pose_source = my_robot.get_best_pose()

    return jsonify({
        "is_navigating":    is_navigating,
        "current_location": current_location,
        "current_progress": current_progress,

        # Odom (relative to home)
        "odom_position": {
            "x": round(my_robot.x, 3),
            "y": round(my_robot.y, 3),
        },
        "odom_yaw_deg": round(math.degrees(my_robot.yaw), 2),

        # AMCL (absolute, frame: map)
        "amcl_position": {
            "x": round(my_robot.amcl_x, 3),
            "y": round(my_robot.amcl_y, 3),
        },
        "amcl_yaw_deg":    round(math.degrees(my_robot.amcl_yaw), 2),
        "amcl_covariance": round(my_robot.amcl_covariance, 4),
        "amcl_ready":      my_robot.amcl_ready,

        # Best estimate
        "best_position": {
            "x": round(best_x, 3),
            "y": round(best_y, 3),
        },
        "best_yaw_deg":  round(math.degrees(best_yaw), 2),
        "pose_source":   pose_source,   # "amcl" หรือ "odom"
    })


@app.route('/stop', methods=['POST', 'GET'])
def stop_robot():
    global is_navigating
    is_navigating = False
    velocity_publisher.publish(Twist())
    return jsonify({"status": "success", "message": "Stopped"}), 200


@app.route('/command/reset-home', methods=['POST'])
def handle_reset_home():
    global is_navigating
    is_navigating = False
    rospy.loginfo("Reset signal received: Stopping current task...")
    threading.Thread(target=my_robot.execute_home_sequence).start()
    return jsonify({
        "status": "success",
        "message": "Robot is executing home sequence and resetting node to 1"
    }), 200


@app.route('/amcl/set-pose', methods=['POST'])
def handle_set_pose():
    """
    ตั้งตำแหน่งเริ่มต้นให้ AMCL แบบ manual
    Body JSON: {"x": 0.0, "y": 0.0, "yaw": 0.0}   (yaw หน่วยเป็นองศา)

    ใช้เมื่อ:
    - เปิดเครื่องใหม่แล้ว AMCL หา particle ไม่เจอ
    - หุ่นถูกย้ายตำแหน่งโดยไม่ได้สั่งให้ขยับ
    """
    data   = request.json or {}
    x      = float(data.get("x",   0.0))
    y      = float(data.get("y",   0.0))
    yaw    = float(data.get("yaw", 0.0))

    threading.Thread(target=my_robot.set_initial_pose, args=(x, y, yaw)).start()
    return jsonify({
        "status":  "success",
        "message": f"Initial pose sent to AMCL: x={x}, y={y}, yaw={yaw}°"
    }), 200


@app.route('/amcl/status', methods=['GET'])
def handle_amcl_status():
    """ดูสถานะ AMCL แยกต่างหาก"""
    return jsonify({
        "amcl_ready":      my_robot.amcl_ready,
        "amcl_covariance": round(my_robot.amcl_covariance, 4),
        "amcl_position": {
            "x": round(my_robot.amcl_x, 3),
            "y": round(my_robot.amcl_y, 3),
        },
        "amcl_yaw_deg": round(math.degrees(my_robot.amcl_yaw), 2),
        "note": "covariance < 0.05 means AMCL is confident"
    })


# ==================================================
# MAIN
# ==================================================
if __name__ == "__main__":
    velocity_publisher = rospy.Publisher(
        '/mobile_base/commands/velocity', Twist, queue_size=10)

    my_robot = OdomRobot()

    current_location = 1
    is_navigating    = False

    print("--- Robot Server Ready on Port 5000 ---")
    print("  New endpoints:")
    print("    POST /amcl/set-pose   body: {x, y, yaw}")
    print("    GET  /amcl/status")
    print("    GET  /status          (now includes amcl + best_pose fields)")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
