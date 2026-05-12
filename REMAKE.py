#!/usr/bin/env python3
import rospy
import math
import signal
import sys
import threading
import json, os
import numpy as np

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import tf
from flask import Flask, request, jsonify

# ==================================================
# GLOBAL SETTINGS & STATE
# ==================================================
is_navigating    = False
current_location = 1
current_progress = 0
velocity_publisher = None


def signal_handler(sig, frame):
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
        derivative    = (error - self.last_error) / dt
        output        = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
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
        self.x, self.y, self.yaw             = 0.0, 0.0, 0.0
        self.offset_x, self.offset_y, self.offset_yaw = 0.0, 0.0, 0.0

        # --- gmapping TF ---
        # gmapping publish transform: map → odom → base_link
        # ใช้ tf listener ดึง pose ของ base_link ใน frame map
        self.tf_listener = tf.TransformListener()

        # --- LiDAR / Wall Avoidance ---
        self.scan_data   = None
        self.scan_lock   = threading.Lock()
        self.WALL_OFFSET = 0.50   # ระยะที่ต้องการห่างกำแพง (เมตร)
        self.WALL_GAIN   = 1.2    # ความไวในการแก้ (เพิ่ม=แก้เร็ว, ลด=แก้ช้า/ลด oscillate)

        # --- Subscribers ---
        rospy.Subscriber("/odom",  Odometry, self.odom_callback)
        rospy.Subscriber("/scan",  LaserScan, self.scan_callback)

        # --- PID ---
        self.pid_straight = PID(kp=1.8, ki=0.005, kd=0.1, min_val=-0.4, max_val=0.4)
        self.pid_rotate   = PID(kp=1.0, ki=0.01,  kd=0.1, min_val=-0.3, max_val=0.3)

        # Wait for odom
        rospy.loginfo("Waiting for odom data...")
        rospy.wait_for_message("/odom", Odometry)
        rospy.sleep(1)

        # --- Home Sequence ---
        self.reset_home()
        rospy.loginfo("=== START HOME SEQUENCE ===")
        is_navigating = True
        self.move_forward(2.5)
        self.rotate(math.radians(-90))
        self.move_forward(0.5)
        self.rotate(math.radians(180))
        self.reset_home()
        is_navigating = False
        rospy.loginfo("=== ROBOT READY (HOME=0,0,0) ===")

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

    def scan_callback(self, msg):
        with self.scan_lock:
            self.scan_data = msg

    # --------------------------------------------------
    # WALL AVOIDANCE (LiDAR)
    # --------------------------------------------------
    def get_wall_bias(self):
        """
        วัดระยะห่างกำแพงซ้าย/ขวาจาก LiDAR แล้วคืน angular bias
        เพื่อรักษาระยะ WALL_OFFSET จากกำแพงทั้งสองข้าง

          bias > 0 → เลี้ยวซ้าย  (หนีกำแพงขวา)
          bias < 0 → เลี้ยวขวา  (หนีกำแพงซ้าย)
          bias = 0 → ปลอดภัย ไม่ต้องปรับ
        """
        with self.scan_lock:
            if self.scan_data is None:
                return 0.0
            ranges   = np.array(self.scan_data.ranges, dtype=float)
            ang_min  = self.scan_data.angle_min
            ang_inc  = self.scan_data.angle_increment

        n      = len(ranges)
        angles = ang_min + np.arange(n) * ang_inc

        # กรองค่า invalid (0, inf, nan)
        ranges = np.where((ranges < 0.05) | np.isinf(ranges), np.nan, ranges)

        # ฝั่งซ้าย  30°–80°  |  ฝั่งขวา  -80°–-30°
        # (ปรับมุมถ้า LiDAR ติดตั้งหันต่างออกไป)
        left_mask  = (angles >  math.radians(30)) & (angles <  math.radians(80))
        right_mask = (angles > -math.radians(80)) & (angles < -math.radians(30))

        left_dist  = float(np.nanmin(ranges[left_mask]))  if left_mask.any()  else 9.9
        right_dist = float(np.nanmin(ranges[right_mask])) if right_mask.any() else 9.9

        S = self.WALL_OFFSET
        G = self.WALL_GAIN

        if left_dist < S and right_dist < S:
            # ช่องแคบ → อยู่กลาง
            bias = (left_dist - right_dist) * G * 0.5
        elif left_dist < S:
            # ใกล้กำแพงซ้าย → หักขวา
            bias = -(S - left_dist) * G
        elif right_dist < S:
            # ใกล้กำแพงขวา → หักซ้าย
            bias =  (S - right_dist) * G
        else:
            bias = 0.0

        return float(np.clip(bias, -0.35, 0.35))

    # --------------------------------------------------
    # GMAPPING POSE (via tf)
    # --------------------------------------------------
    def get_best_pose(self):
        """
        ดึง pose ของ base_link ใน frame map จาก gmapping ผ่าน tf
        ถ้า tf ยังไม่พร้อม fallback ใช้ odom
        คืนค่า (x, y, yaw, source)
        """
        try:
            self.tf_listener.waitForTransform(
                "/map", "/base_link", rospy.Time(0), rospy.Duration(0.5))
            (trans, rot) = self.tf_listener.lookupTransform(
                "/map", "/base_link", rospy.Time(0))
            (_, _, yaw) = euler_from_quaternion(rot)
            return trans[0], trans[1], yaw, "gmapping"

        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException):
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
        self.move_forward(2.5)
        self.rotate(math.radians(-90))
        self.move_forward(0.5)
        self.rotate(math.radians(180))
        self.reset_home()
        current_location = 1
        is_navigating    = False
        rospy.loginfo("--- Home Sequence Completed: Current Node is 1 ---")

    # --------------------------------------------------
    # MOTION PRIMITIVES
    # --------------------------------------------------
    def move_forward(self, distance, bias=0.0):
        """
        เดินตรงระยะ distance (เมตร)
        bias: manual angular offset (ใช้ใน path เก่า หรือส่ง 0.0 ได้เลย)
        wall_bias จาก LiDAR จะถูกรวมเข้าอัตโนมัติ
        """
        start_x, start_y = self.x, self.y
        target_yaw       = self.yaw
        rate             = rospy.Rate(20)
        LINEAR_SPEED     = 0.10

        current_linear_speed = 0.05
        accel      = 0.008
        min_speed  = 0.07
        decel_dist = 0.4

        self.pid_straight.integral   = 0.0
        self.pid_straight.last_error = 0.0

        while not rospy.is_shutdown() and is_navigating:
            traveled       = math.sqrt((self.x - start_x)**2 + (self.y - start_y)**2)
            remaining_dist = distance - traveled

            if traveled >= distance:
                break

            # accel / decel
            if remaining_dist > decel_dist:
                current_linear_speed = min(current_linear_speed + accel, LINEAR_SPEED)
            else:
                current_linear_speed = max(min_speed,
                                           (remaining_dist / decel_dist) * LINEAR_SPEED)

            error_yaw = math.atan2(math.sin(target_yaw - self.yaw),
                                   math.cos(target_yaw - self.yaw))

            # รวม PID correction + manual bias + wall avoidance
            wall_bias = self.get_wall_bias()

            twist = Twist()
            twist.linear.x  = current_linear_speed
            twist.angular.z = (self.pid_straight.compute(error_yaw, 0.05)
                               + bias
                               + wall_bias)
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

        # bias ถูกลบออกแล้ว — LiDAR จัดการระยะห่างกำแพงให้อัตโนมัติ
        paths = {
            (1, 2): [
                ("rotate", -90),
                ("move",   6.0),
                ("rotate", 90),
                ("move",  10.0),
                ("move",   5.2),
                ("rotate", 90),
                ("move",   1.0),
            ],
            (1, 3): [
                ("rotate", -90),
                ("move",   6.0),
                ("rotate", 90),
                ("move",  10.0),
                ("move",  10.0),
                ("move",   7.2),
                ("rotate", 90),
                ("move",   1.0),
            ],
            (2, 1): [
                ("rotate", 180),
                ("move",   1.0),
                ("rotate", -90),
                ("move",  10.0),
                ("move",   5.2),
                ("rotate", -90),
                ("move",   6.0),
                ("rotate", -90),
            ],
            (2, 3): [
                ("rotate", 180),
                ("move",   1.0),
                ("rotate", 90),
                ("move",  10.0),
                ("move",   6.0),
                ("rotate", 90),
                ("move",   1.0),
            ],
            (3, 1): [
                ("rotate", 180),
                ("move",   1.0),
                ("rotate", -90),
                ("move",  10.0),
                ("move",  10.0),
                ("move",   7.2),
                ("rotate", -90),
                ("move",   5.8),
                ("rotate", -90),
            ],
            (3, 2): [
                ("rotate", 180),
                ("move",   1.0),
                ("rotate", -90),
                ("move",  10.0),
                ("move",   2.5),
                ("rotate", -90),
                ("move",   1.0),
            ],
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
                dist = float(cmd[1])
                bias = float(cmd[2]) if len(cmd) > 2 else 0.0
                self.move_forward(dist, bias)

            elif action == "rotate":
                self.rotate(math.radians(cmd[1]))
                rospy.sleep(0.5)

            current_progress = (current_steps / total_steps) * 100
            rospy.loginfo(f"progress = {current_progress:.2f}%")

        self.pub.publish(Twist())
        return True


# ==================================================
# FLASK API SERVER
# ==================================================
app      = Flask(__name__)
my_robot = None


@app.route('/command', methods=['POST'])
def handle_command():
    global is_navigating, current_progress
    data   = request.json
    start  = data.get('start')
    target = data.get('target')

    if is_navigating:
        return jsonify({"status": "error", "message": "Robot is busy"}), 400

    def run_and_finish(s, t):
        global is_navigating, current_location, current_progress
        current_progress = 0
        is_navigating    = True
        success          = my_robot.execute_path(s, t)
        is_navigating    = False
        if success:
            current_location = t

    threading.Thread(target=run_and_finish, args=(start, target)).start()
    return jsonify({"status": "starting"}), 200


@app.route('/status', methods=['GET'])
def get_status():
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

        # gmapping pose (frame: map, via tf)
        "map_position": {
            "x": round(best_x, 3),
            "y": round(best_y, 3),
        },
        "map_yaw_deg": round(math.degrees(best_yaw), 2),
        "pose_source": pose_source,   # "gmapping" หรือ "odom"

        # Wall avoidance info
        "wall_offset_m": my_robot.WALL_OFFSET,
        "wall_gain":     my_robot.WALL_GAIN,
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
        "status":  "success",
        "message": "Robot is executing home sequence and resetting node to 1"
    }), 200


@app.route('/wall/config', methods=['POST'])
def set_wall_config():
    """
    ปรับ wall avoidance parameter แบบ runtime
    Body JSON: {"offset": 0.5, "gain": 1.2}
    """
    data = request.json or {}
    if "offset" in data:
        my_robot.WALL_OFFSET = float(data["offset"])
    if "gain" in data:
        my_robot.WALL_GAIN = float(data["gain"])
    return jsonify({
        "status":       "success",
        "wall_offset_m": my_robot.WALL_OFFSET,
        "wall_gain":     my_robot.WALL_GAIN,
    }), 200


@app.route('/wall/config', methods=['GET'])
def get_wall_config():
    return jsonify({
        "wall_offset_m": my_robot.WALL_OFFSET,
        "wall_gain":     my_robot.WALL_GAIN,
        "note": "POST to /wall/config with {offset, gain} to change"
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
    print("  Endpoints:")
    print("    POST /command          body: {start, target}")
    print("    POST /stop")
    print("    POST /command/reset-home")
    print("    GET  /status           (odom + gmapping tf + wall info)")
    print("    GET  /wall/config")
    print("    POST /wall/config      body: {offset, gain}")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
