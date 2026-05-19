#!/usr/bin/env python3
import rospy
import math
import signal
import sys
import threading
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from tf.transformations import euler_from_quaternion
from flask import Flask, request, jsonify
import actionlib

# ==================================================
# GLOBAL SETTINGS & STATE
# ==================================================
is_navigating    = False
current_location = 1
current_progress = 0
velocity_publisher = None

NODE_POSES = {
    1:  {"x":  2.379, "y":  -0.639, "yaw":  0.012},
    2:  {"x": 12.455, "y":  13.314, "yaw":  0.002},
    3:  {"x": 16.521, "y":  22.191, "yaw":  0.002},
}

def signal_handler(sig, frame):
    print("\n[INFO] Ctrl+C — Stopping Robot.")
    if velocity_publisher:
        velocity_publisher.publish(Twist())
    rospy.signal_shutdown("User Interrupted")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


# ==================================================
# PID CONTROLLER
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
# OBSTACLE MONITOR
# ตรวจจับสิ่งกีดขวางจาก /scan แบบ real-time
# แบ่ง scan เป็น 3 โซน: ซ้าย / หน้า / ขวา
# ==================================================
class ObstacleMonitor:
    def __init__(self):
        self.DANGER_DIST  = 0.50   # เมตร — อันตราย หยุดทันที
        self.WARNING_DIST = 0.80   # เมตร — เตือน ชะลอ

        self.front_dist = 999.0
        self.left_dist  = 999.0
        self.right_dist = 999.0
        self.lock = threading.Lock()

        rospy.Subscriber("/scan", LaserScan, self._scan_callback)
        rospy.loginfo("[ObstacleMonitor] Subscribed to /scan")

    def _scan_callback(self, msg):
        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return

        def safe_min(indices):
            vals = [ranges[i] for i in indices
                    if i < n
                    and not math.isnan(ranges[i])
                    and not math.isinf(ranges[i])
                    and ranges[i] > 0.01]
            return min(vals) if vals else 999.0

        # แบ่ง 3 โซน (360 ray, index 0 = หน้า)
        front_idx = list(range(0, 30)) + list(range(n - 30, n))  # ±30° หน้า
        left_idx  = list(range(30, 120))                          # 30°–120° ซ้าย
        right_idx = list(range(n - 120, n - 30))                  # 240°–330° ขวา

        with self.lock:
            self.front_dist = safe_min(front_idx)
            self.left_dist  = safe_min(left_idx)
            self.right_dist = safe_min(right_idx)

    def get_distances(self):
        with self.lock:
            return self.front_dist, self.left_dist, self.right_dist

    def is_front_blocked(self):
        return self.front_dist < self.DANGER_DIST

    def is_front_warning(self):
        return self.front_dist < self.WARNING_DIST

    def get_status(self):
        f, l, r = self.get_distances()
        return {
            "front_dist":    round(f, 3),
            "left_dist":     round(l, 3),
            "right_dist":    round(r, 3),
            "front_blocked": f < self.DANGER_DIST,
            "front_warning": f < self.WARNING_DIST,
            "safe_side":     "left" if l > r else "right",
        }


# ==================================================
# ROBOT CONTROL CLASS
# ==================================================
class OdomRobot:
    def __init__(self):
        global is_navigating
        rospy.init_node("odom_robot")
        self.pub = velocity_publisher

        # --- Odometry ---
        self.raw_x, self.raw_y, self.raw_yaw = 0.0, 0.0, 0.0
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.offset_x, self.offset_y, self.offset_yaw = 0.0, 0.0, 0.0

        # --- AMCL ---
        self.amcl_x, self.amcl_y, self.amcl_yaw = 0.0, 0.0, 0.0
        self.amcl_covariance = 1.0
        self.amcl_ready = False
        self.amcl_lock  = threading.Lock()

        # --- Obstacle Monitor ---
        self.obstacle = ObstacleMonitor()

        # --- move_base ---
        rospy.loginfo("[move_base] Connecting...")
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        mb_ok = self.move_base_client.wait_for_server(timeout=rospy.Duration(10.0))
        rospy.loginfo("[move_base] Connected!" if mb_ok else "[move_base] NOT available.")

        # --- Subscribers ---
        rospy.Subscriber("/odom",      Odometry,                  self.odom_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)

        self.pid_straight = PID(kp=1.8, ki=0.005, kd=0.1, min_val=-0.4, max_val=0.4)
        self.pid_rotate   = PID(kp=1.0, ki=0.01,  kd=0.1, min_val=-0.3, max_val=0.3)

        rospy.loginfo("Waiting for odom...")
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
        rospy.loginfo("=== ROBOT READY ===")

        self._wait_for_amcl(timeout=10.0)

        # ตั้ง AMCL = Node 1 ทันทีหลัง Home Sequence เสร็จ
        n1 = NODE_POSES[1]
        self.set_initial_pose(n1["x"], n1["y"], n1["yaw"])
        rospy.loginfo(f"[AMCL] Initial pose set to Node 1: x={n1['x']} y={n1['y']} yaw={n1['yaw']}°")

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
        with self.amcl_lock:
            self.amcl_x = msg.pose.pose.position.x
            self.amcl_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            (_, _, self.amcl_yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
            cov = msg.pose.covariance
            self.amcl_covariance = (cov[0] + cov[7]) / 2.0
            if self.amcl_covariance < 0.05:
                self.amcl_ready = True

    # --------------------------------------------------
    # AMCL HELPERS
    # --------------------------------------------------
    def _wait_for_amcl(self, timeout=10.0):
        rospy.loginfo("Waiting for /amcl_pose (%.0fs)..." % timeout)
        try:
            rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=timeout)
            rospy.loginfo("[AMCL] Running.")
        except rospy.ROSException:
            rospy.logwarn("[AMCL] Not received. Odometry only.")

    def set_initial_pose(self, x, y, yaw_deg):
        pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped,
                              queue_size=1, latch=True)
        rospy.sleep(0.5)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp    = rospy.Time.now()
        yr = math.radians(yaw_deg)
        msg.pose.pose.position.x    = x
        msg.pose.pose.position.y    = y
        msg.pose.pose.orientation.z = math.sin(yr / 2.0)
        msg.pose.pose.orientation.w = math.cos(yr / 2.0)
        cov = [0.0] * 36
        cov[0] = 0.25; cov[7] = 0.25; cov[35] = 0.068
        msg.pose.covariance = cov
        pub.publish(msg)
        rospy.loginfo(f"[AMCL] Pose set x={x:.2f} y={y:.2f} yaw={yaw_deg}°")

    def get_best_pose(self):
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
        rospy.loginfo("--- Home Sequence Start ---")
        is_navigating = True
        self.reset_home()
        self.move_forward(2.5)
        self.rotate(math.radians(-90))
        self.move_forward(0.5)
        self.rotate(math.radians(180))
        self.reset_home()
        current_location = 1
        is_navigating = False

        # ตั้ง AMCL = Node 1 ทันทีหลัง Home Sequence เสร็จ
        n1 = NODE_POSES[1]
        self.set_initial_pose(n1["x"], n1["y"], n1["yaw"])
        rospy.loginfo(f"[AMCL] Pose reset to Node 1: x={n1['x']} y={n1['y']} yaw={n1['yaw']}°")
        rospy.loginfo("--- Home Sequence Done: Node=1 ---")

    # --------------------------------------------------
    # MOTION PRIMITIVES  (Home Sequence เท่านั้น)
    # --------------------------------------------------
    def move_forward(self, distance, bias=0.0):
        start_x, start_y = self.x, self.y
        target_yaw = self.yaw
        rate = rospy.Rate(20)
        LINEAR_SPEED = 0.10
        cur_spd = 0.05
        self.pid_straight.integral = self.pid_straight.last_error = 0.0

        while not rospy.is_shutdown() and is_navigating:
            traveled = math.sqrt((self.x - start_x)**2 + (self.y - start_y)**2)
            rem = distance - traveled
            if traveled >= distance:
                break
            cur_spd = min(cur_spd + 0.008, LINEAR_SPEED) if rem > 0.4 \
                      else max(0.07, (rem / 0.4) * LINEAR_SPEED)
            err_yaw = math.atan2(math.sin(target_yaw - self.yaw),
                                 math.cos(target_yaw - self.yaw))
            t = Twist()
            t.linear.x  = cur_spd
            t.angular.z = self.pid_straight.compute(err_yaw, 0.05) + bias
            self.pub.publish(t)
            rate.sleep()

        self.pub.publish(Twist())
        rospy.sleep(0.3)

    def rotate(self, angle_rad):
        target_yaw = math.atan2(math.sin(self.yaw + angle_rad),
                                math.cos(self.yaw + angle_rad))
        rate = rospy.Rate(30)
        self.pid_rotate.integral = 0

        while not rospy.is_shutdown() and is_navigating:
            err = math.atan2(math.sin(target_yaw - self.yaw),
                             math.cos(target_yaw - self.yaw))
            if abs(err) < 0.005:
                break
            t = Twist()
            t.angular.z = self.pid_rotate.compute(err, 1.0 / 30.0)
            self.pub.publish(t)
            rate.sleep()

        self.pub.publish(Twist())
        rospy.sleep(0.3)

    # --------------------------------------------------
    # RECOVERY — เมื่อ move_base ส่ง ABORTED
    # ถอยหลัง แล้วหมุนหาด้านที่โล่งกว่า
    # --------------------------------------------------
    def _recovery_wiggle(self):
        rospy.logwarn("[Recovery] Starting wiggle recovery...")
        rate = rospy.Rate(10)

        f, l, r = self.obstacle.get_distances()
        rospy.loginfo(f"[Recovery] front={f:.2f}m left={l:.2f}m right={r:.2f}m")

        # ขั้น 1: ถอยหลัง 0.3 เมตร (~2 วินาที)
        rospy.loginfo("[Recovery] Step 1: Back up")
        t = Twist()
        t.linear.x = -0.10
        for _ in range(20):
            if not is_navigating:
                return
            self.pub.publish(t)
            rate.sleep()
        self.pub.publish(Twist())
        rospy.sleep(0.5)

        # ขั้น 2: หมุนไปด้านที่โล่งกว่า (~1 วินาที ≈ 23°)
        _, l, r = self.obstacle.get_distances()
        turn_dir = 1.0 if l > r else -1.0
        rospy.loginfo(f"[Recovery] Step 2: Rotate {'left' if turn_dir > 0 else 'right'}")
        t = Twist()
        t.angular.z = turn_dir * 0.4
        for _ in range(25):
            if not is_navigating:
                return
            self.pub.publish(t)
            rate.sleep()
        self.pub.publish(Twist())
        rospy.sleep(0.5)

        rospy.loginfo("[Recovery] Done — move_base will replan")

    # --------------------------------------------------
    # move_base NAVIGATION + obstacle retry loop
    # --------------------------------------------------
    def _build_goal(self, node_id):
        if node_id not in NODE_POSES:
            rospy.logerr(f"Node {node_id} not in NODE_POSES")
            return None
        p = NODE_POSES[node_id]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.pose.position.x = p["x"]
        goal.target_pose.pose.position.y = p["y"]
        yr = math.radians(p["yaw"])
        goal.target_pose.pose.orientation.z = math.sin(yr / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yr / 2.0)
        return goal

    def navigate_to_node(self, target_node):
        global is_navigating, current_progress

        goal = self._build_goal(target_node)
        if goal is None:
            return False

        tx = NODE_POSES[target_node]["x"]
        ty = NODE_POSES[target_node]["y"]
        rospy.loginfo(f"[Nav] → Node {target_node} ({tx:.2f}, {ty:.2f})")

        MAX_RETRIES = 3
        retry = 0

        while retry <= MAX_RETRIES and is_navigating:

            # ส่ง goal (ครั้งแรก หรือหลัง recovery)
            goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base_client.send_goal(goal)
            current_progress = 0
            rate = rospy.Rate(2)

            while not rospy.is_shutdown() and is_navigating:

                # แจ้งเตือน log เมื่อหน้าชนสิ่งกีดขวาง
                if self.obstacle.is_front_blocked():
                    rospy.logwarn(
                        f"[Obstacle] DANGER front={self.obstacle.front_dist:.2f}m "
                        f"— move_base replanning...")

                state = self.move_base_client.get_state()

                if state == GoalStatus.SUCCEEDED:
                    current_progress = 100
                    rospy.loginfo(f"[Nav] ✓ Reached Node {target_node}!")
                    return True

                elif state == GoalStatus.ABORTED:
                    retry += 1
                    rospy.logwarn(f"[Nav] ABORTED (retry {retry}/{MAX_RETRIES})")
                    if retry <= MAX_RETRIES:
                        self._recovery_wiggle()
                    break  # resend goal

                elif state in (GoalStatus.REJECTED, GoalStatus.PREEMPTED,
                               GoalStatus.LOST):
                    rospy.logwarn(f"[Nav] Failed state={state}")
                    return False

                # คำนวณ progress จากระยะเหลือ
                bx, by, _, _ = self.get_best_pose()
                dist_rem = math.sqrt((tx - bx)**2 + (ty - by)**2)
                src = NODE_POSES.get(current_location, {"x": bx, "y": by})
                dist_tot = math.sqrt((tx - src["x"])**2 + (ty - src["y"])**2)
                if dist_tot > 0.01:
                    current_progress = max(0, min(99,
                        (1 - dist_rem / dist_tot) * 100))

                rospy.loginfo(
                    f"[Nav] progress={current_progress:.1f}% "
                    f"dist={dist_rem:.2f}m "
                    f"| obstacle front={self.obstacle.front_dist:.2f}m "
                    f"left={self.obstacle.left_dist:.2f}m "
                    f"right={self.obstacle.right_dist:.2f}m")
                rate.sleep()

        rospy.logwarn(f"[Nav] Gave up after {MAX_RETRIES} retries.")
        return False

    def execute_path(self, start, target):
        return self.navigate_to_node(target)


# ==================================================
# FLASK API SERVER
# ==================================================
app = Flask(__name__)
my_robot = None


@app.route('/command', methods=['POST'])
def handle_command():
    global is_navigating, current_progress
    data   = request.json or {}
    start  = data.get('start')
    target = data.get('target')

    if is_navigating:
        return jsonify({"status": "error", "message": "Robot is busy"}), 400
    if target not in NODE_POSES:
        return jsonify({"status": "error",
                        "message": f"Node {target} not in NODE_POSES"}), 400

    def run_and_finish(s, t):
        global is_navigating, current_location, current_progress
        current_progress = 0
        is_navigating    = True
        success = my_robot.execute_path(s, t)
        is_navigating = False
        if success:
            current_location = t

    threading.Thread(target=run_and_finish, args=(start, target)).start()
    return jsonify({"status": "starting",
                    "target_node": target,
                    "target_pose": NODE_POSES[target]}), 200


@app.route('/status', methods=['GET'])
def get_status():
    bx, by, byaw, src = my_robot.get_best_pose()
    return jsonify({
        "is_navigating":    is_navigating,
        "current_location": current_location,
        "current_progress": round(current_progress, 1),
        "odom_position":    {"x": round(my_robot.x, 3), "y": round(my_robot.y, 3)},
        "odom_yaw_deg":     round(math.degrees(my_robot.yaw), 2),
        "amcl_position":    {"x": round(my_robot.amcl_x, 3), "y": round(my_robot.amcl_y, 3)},
        "amcl_yaw_deg":     round(math.degrees(my_robot.amcl_yaw), 2),
        "amcl_covariance":  round(my_robot.amcl_covariance, 4),
        "amcl_ready":       my_robot.amcl_ready,
        "best_position":    {"x": round(bx, 3), "y": round(by, 3)},
        "best_yaw_deg":     round(math.degrees(byaw), 2),
        "pose_source":      src,
        "obstacle":         my_robot.obstacle.get_status(),
    })


@app.route('/obstacle', methods=['GET'])
def get_obstacle():
    """ดูระยะสิ่งกีดขวาง real-time"""
    return jsonify(my_robot.obstacle.get_status())


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
    threading.Thread(target=my_robot.execute_home_sequence).start()
    return jsonify({"status": "success",
                    "message": "Home sequence started, node → 1"}), 200


@app.route('/amcl/set-pose', methods=['POST'])
def handle_set_pose():
    data = request.json or {}
    x   = float(data.get("x",   0.0))
    y   = float(data.get("y",   0.0))
    yaw = float(data.get("yaw", 0.0))
    threading.Thread(target=my_robot.set_initial_pose, args=(x, y, yaw)).start()
    return jsonify({"status": "success",
                    "message": f"Pose → x={x} y={y} yaw={yaw}°"}), 200


@app.route('/amcl/status', methods=['GET'])
def handle_amcl_status():
    return jsonify({
        "amcl_ready":      my_robot.amcl_ready,
        "amcl_covariance": round(my_robot.amcl_covariance, 4),
        "amcl_position":   {"x": round(my_robot.amcl_x, 3),
                            "y": round(my_robot.amcl_y, 3)},
        "amcl_yaw_deg":    round(math.degrees(my_robot.amcl_yaw), 2),
        "note":            "covariance < 0.05 = confident"
    })


@app.route('/nodes', methods=['GET'])
def get_nodes():
    return jsonify({"nodes": NODE_POSES})


@app.route('/nodes/<int:node_id>', methods=['POST'])
def update_node(node_id):
    data = request.json or {}
    NODE_POSES[node_id] = {
        "x":   float(data.get("x",   0.0)),
        "y":   float(data.get("y",   0.0)),
        "yaw": float(data.get("yaw", 0.0)),
    }
    return jsonify({"status": "updated", "node": node_id,
                    "pose": NODE_POSES[node_id]}), 200


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
    print("  POST /command              {start, target}")
    print("  GET  /status               (includes obstacle field)")
    print("  GET  /obstacle             real-time obstacle distances")
    print("  POST /stop")
    print("  POST /command/reset-home")
    print("  POST /amcl/set-pose        {x, y, yaw}")
    print("  GET  /amcl/status")
    print("  GET  /nodes")
    print("  POST /nodes/<id>           {x, y, yaw}")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
